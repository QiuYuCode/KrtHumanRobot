"""Map lifecycle coordination for the Web console."""

from __future__ import annotations

import os
import shutil
import tempfile
import threading
from dataclasses import asdict
from pathlib import Path
from typing import Any, Protocol

import yaml

from krt_task.robot_db import MapRecord, RobotDatabase, WaypointRecord

from krt_human_robot.adapters.navigation import NavigationResult


class NavigationAdapter(Protocol):
    def mapping_running(self) -> bool: ...

    def navigation_running(self) -> bool: ...

    def default_navigation_mode(self) -> str: ...

    def start_mapping(
        self, backend: str | None = None, *, rviz: bool = True
    ) -> NavigationResult: ...

    def save_mapping(self) -> NavigationResult: ...

    def start_navigation(
        self,
        map_yaml: str | None = None,
        pcd_map_path: str | None = None,
        mode: str | None = None,
        *,
        initial_pose: WaypointRecord | None = None,
        rviz: bool = True,
    ) -> NavigationResult: ...

    def stop_navigation(self) -> NavigationResult: ...


class MapManager:
    """Serializes mapping and navigation while persisting map selection."""

    def __init__(
        self,
        adapter: NavigationAdapter,
        database: RobotDatabase,
        map_root: str,
    ) -> None:
        self.adapter = adapter
        self.database = database
        self.map_root = Path(map_root).expanduser().resolve()
        self._lock = threading.RLock()
        self._mapping_map_id: str | None = None
        self._mapping_backend: str | None = None
        self._saving = False
        self._navigation_map_id: str | None = None
        self._last_error = ""

    @staticmethod
    def public_map(record: MapRecord) -> dict[str, Any]:
        return asdict(record)

    def list_maps(self) -> list[dict[str, Any]]:
        return [self.public_map(record) for record in self.database.list_maps()]

    def selected_map(self, *, required: bool = False) -> MapRecord | None:
        record = self.database.get_selected_map()
        if required and record is None:
            raise ValueError("请先选择地图")
        return record

    def start_mapping(
        self, name: str, backend: str, *, rviz: bool = True
    ) -> NavigationResult:
        with self._lock:
            if self._mapping_map_id is not None or self.adapter.mapping_running():
                return NavigationResult(False, "建图已经启动。")
            if self.adapter.navigation_running():
                return NavigationResult(False, "请先停止导航再开始建图。")
            record = self.database.create_map(name, backend)
            result = self.adapter.start_mapping(backend, rviz=rviz)
            if not result.success:
                self.database.fail_map(record.id, result.message)
                return result
            self._mapping_map_id = record.id
            self._mapping_backend = backend
            self._last_error = ""
            return NavigationResult(
                True,
                result.message,
                {"map": self.public_map(self.database.get_map(record.id))},
            )

    def finish_mapping(self) -> NavigationResult:
        with self._lock:
            if self._mapping_map_id is None:
                return NavigationResult(False, "当前没有由 Web 启动的建图任务。")
            if self._saving:
                return NavigationResult(False, "地图正在保存，请勿重复提交。")
            self._saving = True
            map_id = self._mapping_map_id

        try:
            result = self.adapter.save_mapping()
            if not result.success:
                self.database.fail_map(map_id, result.message)
                self._last_error = result.message
                return result
            paths = self._validated_saved_paths(result.data)
            record = self.database.complete_map(map_id, paths)
            with self._lock:
                self._mapping_map_id = None
                self._mapping_backend = None
                self._last_error = ""
            return NavigationResult(
                True,
                result.message,
                {"map": self.public_map(record)},
            )
        except Exception as exc:
            message = f"登记地图失败：{exc}"
            self.database.fail_map(map_id, message)
            self._last_error = message
            return NavigationResult(False, message)
        finally:
            with self._lock:
                self._saving = False

    def select_map(self, map_id: str) -> MapRecord:
        with self._lock:
            if self._mapping_map_id is not None or self._saving:
                raise RuntimeError("建图期间不能切换地图")
            if self.adapter.navigation_running() and self._navigation_map_id != str(
                map_id
            ):
                raise RuntimeError("请先停止导航再切换地图")
            return self.database.select_map(map_id)

    def delete_map(self, map_id: str) -> None:
        with self._lock:
            if self._mapping_map_id is not None or self._saving:
                raise RuntimeError("建图或保存期间不能删除地图")
            record = self.database.get_map(map_id)
            if self.database.list_cruise_schedules(record.id):
                raise RuntimeError("请先删除该地图的巡航计划")
            if record.selected:
                raise RuntimeError("当前地图不能删除，请先选择其他地图")
            if self._navigation_map_id == record.id:
                raise RuntimeError("导航使用中的地图不能删除")
            session_dir = (
                self._safe_path(record.session_dir) if record.session_dir else None
            )
            if session_dir == self.map_root:
                raise ValueError("地图归档目录无效")
            if session_dir is not None and session_dir.exists():
                if not session_dir.is_dir():
                    raise ValueError("地图归档路径不是目录")
                shutil.rmtree(session_dir)
            self.database.delete_map(record.id)

    def start_navigation(
        self,
        map_id: str | None = None,
        mode: str | None = None,
        *,
        initial_waypoint: str | None = None,
        rviz: bool = True,
    ) -> NavigationResult:
        with self._lock:
            if self._mapping_map_id is not None or self._saving:
                return NavigationResult(False, "建图期间不能启动导航。")
            record = (
                self.database.select_map(map_id)
                if map_id
                else self.selected_map(required=True)
            )
            if record is None:
                return NavigationResult(False, "请先选择地图。")
            effective_mode = mode or self.adapter.default_navigation_mode()
            self._validate_map_record(record, require_pcd=effective_mode == "3dloc")
            initial_pose = self._initial_waypoint(record.id, initial_waypoint)
            if initial_pose is None:
                return NavigationResult(False, "请选择当前地图的点位作为导航初始位姿。")
            if self.adapter.navigation_running():
                if self._navigation_map_id == record.id:
                    return NavigationResult(True, "当前地图的导航已经启动。")
                stopped = self.adapter.stop_navigation()
                if not stopped.success:
                    return stopped
            result = self.adapter.start_navigation(
                map_yaml=record.yaml_path,
                pcd_map_path=record.pcd_path,
                mode=mode,
                initial_pose=initial_pose,
                rviz=rviz,
            )
            if result.success:
                self._navigation_map_id = record.id
                return NavigationResult(
                    True, result.message, {"map": self.public_map(record)}
                )
            self._last_error = result.message
            return result

    def _initial_waypoint(
        self, map_id: str, waypoint_name: str | None
    ) -> WaypointRecord | None:
        name = str(waypoint_name or "").strip()
        if not name:
            return None
        for waypoint in self.database.list_waypoints(map_id):
            if waypoint.name == name and waypoint.frame_id == "map":
                return waypoint
        return None

    def stop_navigation(self) -> NavigationResult:
        with self._lock:
            result = self.adapter.stop_navigation()
            if result.success:
                self._navigation_map_id = None
            return result

    def editor_paths(self, map_id: str) -> tuple[MapRecord, Path, Path]:
        """Return the selected map files after enforcing archive containment."""
        with self._lock:
            record = self.database.get_map(map_id)
            selected = self.selected_map(required=True)
            if selected is None or selected.id != record.id:
                raise ValueError("只能编辑当前选中的地图")
            self._validate_map_record(record, require_pcd=False)
            return (
                record,
                self._safe_path(record.yaml_path),
                self._safe_path(record.pgm_path),
            )

    def preview_path(self, map_id: str) -> Path:
        record = self.database.get_map(map_id)
        if record.status != "ready":
            raise ValueError("只能预览已就绪的地图")
        path = self._safe_path(record.pgm_path)
        if not path.is_file():
            raise FileNotFoundError("地图 PGM 文件不存在")
        return path

    def replace_edited_map(
        self, map_id: str, yaml_bytes: bytes, pgm_bytes: bytes
    ) -> MapRecord:
        """Validate and atomically overwrite the selected map YAML and PGM."""
        with self._lock:
            if self._mapping_map_id is not None or self._saving:
                raise RuntimeError("建图期间不能编辑地图")
            if self.adapter.navigation_running():
                raise RuntimeError("请先停止导航再编辑地图")
            record, yaml_path, pgm_path = self.editor_paths(map_id)
            normalized_yaml = self._validated_editor_yaml(
                yaml_path, pgm_path, yaml_bytes
            )
            new_shape = self._pgm_shape(pgm_bytes)
            if new_shape != self._pgm_shape(pgm_path.read_bytes()):
                raise ValueError("编辑后的 PGM 尺寸必须与原地图一致")
            self._atomic_replace(pgm_path, pgm_bytes)
            self._atomic_replace(yaml_path, normalized_yaml)
            return self.database.update_map_files(record.id)

    def status(self) -> dict[str, Any]:
        selected = self.database.get_selected_map()
        with self._lock:
            if self._saving:
                state = "saving"
            elif self._mapping_map_id is not None and self.adapter.mapping_running():
                state = "mapping"
            elif self.adapter.navigation_running():
                state = "navigating"
            else:
                state = "idle"
            return {
                "state": state,
                "mapping_map_id": self._mapping_map_id,
                "mapping_backend": self._mapping_backend,
                "navigation_map_id": self._navigation_map_id,
                "selected_map_id": selected.id if selected else None,
                "last_error": self._last_error,
            }

    def _validated_saved_paths(self, data: dict[str, Any]) -> dict[str, str]:
        required = ("session_dir", "yaml_path", "pgm_path", "pcd_path", "metadata_path")
        paths = {key: str(data.get(key, "")) for key in required}
        for key, value in paths.items():
            if not value:
                raise ValueError(f"缺少 {key}")
            path = self._safe_path(value)
            if key == "session_dir":
                if not path.is_dir():
                    raise FileNotFoundError(path)
            elif not path.is_file() or path.stat().st_size == 0:
                raise FileNotFoundError(path)
            paths[key] = str(path)
        return paths

    def _validate_map_record(self, record: MapRecord, *, require_pcd: bool) -> None:
        if record.status != "ready":
            raise ValueError("只能加载已就绪的地图")
        yaml_path = self._safe_path(record.yaml_path)
        pgm_path = self._safe_path(record.pgm_path)
        if not yaml_path.is_file() or not pgm_path.is_file():
            raise FileNotFoundError("地图 YAML 或 PGM 文件不存在")
        if require_pcd and not self._safe_path(record.pcd_path).is_file():
            raise FileNotFoundError("3D PCD 地图不存在")

    def _validated_editor_yaml(
        self, current_path: Path, pgm_path: Path, payload: bytes
    ) -> bytes:
        try:
            edited = yaml.safe_load(payload.decode("utf-8"))
            current = yaml.safe_load(current_path.read_text(encoding="utf-8"))
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise ValueError("地图 YAML 格式无效") from exc
        if not isinstance(edited, dict) or not isinstance(current, dict):
            raise ValueError("地图 YAML 必须是键值对象")
        for key in ("resolution", "origin", "negate", "occupied_thresh", "free_thresh"):
            if edited.get(key) != current.get(key):
                raise ValueError(f"地图编辑器不能修改 YAML 字段: {key}")
        image_name = Path(str(edited.get("image", ""))).name
        if image_name != pgm_path.name:
            raise ValueError(f"YAML image 必须指向 {pgm_path.name}")
        edited["image"] = pgm_path.name
        return yaml.safe_dump(edited, allow_unicode=True, sort_keys=False).encode(
            "utf-8"
        )

    @staticmethod
    def _pgm_shape(payload: bytes) -> tuple[int, int, int]:
        if not payload.startswith(b"P5"):
            raise ValueError("仅支持二进制 P5 PGM 地图")
        index = 2
        tokens: list[int] = []
        while len(tokens) < 3:
            while index < len(payload) and payload[index] in b" \t\r\n":
                index += 1
            if index < len(payload) and payload[index] == ord("#"):
                while index < len(payload) and payload[index] not in b"\r\n":
                    index += 1
                continue
            start = index
            while index < len(payload) and payload[index] not in b" \t\r\n":
                index += 1
            if start == index:
                raise ValueError("PGM 文件头不完整")
            try:
                tokens.append(int(payload[start:index]))
            except ValueError as exc:
                raise ValueError("PGM 文件头包含无效数字") from exc
        width, height, max_value = tokens
        if width <= 0 or height <= 0 or not 0 < max_value <= 255:
            raise ValueError("PGM 尺寸或灰度范围无效")
        if index >= len(payload) or payload[index] not in b" \t\r\n":
            raise ValueError("PGM 文件头缺少数据分隔符")
        index += 2 if payload[index : index + 2] == b"\r\n" else 1
        if len(payload) - index != width * height:
            raise ValueError("PGM 像素数据长度与尺寸不匹配")
        return width, height, max_value

    @staticmethod
    def _atomic_replace(path: Path, payload: bytes) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            temporary.chmod(path.stat().st_mode)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _safe_path(self, value: str) -> Path:
        path = Path(value).expanduser().resolve()
        if path != self.map_root and self.map_root not in path.parents:
            raise ValueError("地图路径不在允许的目录中")
        return path
