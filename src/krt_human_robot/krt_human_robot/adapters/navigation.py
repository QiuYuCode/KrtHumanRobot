"""ROS CLI adapter for ranger_nav mapping and navigation."""

from __future__ import annotations

import logging
import math
import os
import shutil
import signal
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from krt_task.robot_db import WaypointRecord

try:
    from loguru import logger
except ImportError:
    logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class NavigationResult:
    success: bool
    message: str
    data: dict[str, Any] = field(default_factory=dict)


class RangerNavAdapter:
    """Owns ranger_nav launch processes started by the core tree."""

    def __init__(
        self,
        config: Any,
        *,
        popen: Callable[..., Any] = subprocess.Popen,
        run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._cfg = config.adapters.get("navigation", {})
        self._popen = popen
        self._run = run
        self._sleep = sleep
        self._mapping_process: Any | None = None
        self._navigation_process: Any | None = None
        self._cruise_process: Any | None = None
        self._cruise_lock = threading.Lock()
        self._mapping_backend: str | None = None

    def start_mapping(
        self, backend: str | None = None, *, rviz: bool = True
    ) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始建图。")
        if self._running(self._mapping_process):
            return NavigationResult(True, "建图已经启动。")

        backend = str(backend or self._backend()).strip().lower()
        launch_file = {
            "fast_lio": self._cfg.get("mapping_launch_fast_lio", "mapping.launch.py"),
            "spark_sam": self._cfg.get(
                "mapping_launch_spark_sam", "mapping_sam.launch.py"
            ),
        }.get(backend)
        if launch_file is None:
            return NavigationResult(False, f"未知建图后端：{backend}。")

        rviz_argument = "sam_rviz" if backend == "spark_sam" else "rviz"
        try:
            self._mapping_process = self._popen(
                [
                    "ros2",
                    "launch",
                    self._package(),
                    launch_file,
                    f"{rviz_argument}:={'true' if rviz else 'false'}",
                ],
                start_new_session=True,
            )
            self._mapping_backend = backend
        except OSError as exc:
            return NavigationResult(False, f"启动建图失败：{exc}。")
        process = self._mapping_process
        if process is None:
            return NavigationResult(False, "建图进程未创建。")
        logger.info(f"已启动建图 launch: backend={backend}, pid={process.pid}")
        return NavigationResult(True, "已开始建图。")

    def save_mapping(self) -> NavigationResult:
        if not self._running(self._mapping_process):
            return NavigationResult(False, "建图未启动，无法保存地图。")

        backend = self._mapping_backend or self._backend()
        if backend not in {"fast_lio", "spark_sam"}:
            return NavigationResult(False, f"未知建图后端：{backend}。")

        session_dir = self._create_map_session_dir()
        cloud_path = session_dir / str(
            self._cfg.get("session_cloud_filename", "cloud.pcd")
        )
        result = NavigationResult(False, "地图保存命令未执行。")
        source_pcd = Path()
        if backend == "fast_lio":
            result = self._call(
                [
                    "ros2",
                    "service",
                    "call",
                    self._cfg.get("fast_lio_save_service", "/map_save"),
                    "std_srvs/srv/Trigger",
                    "{}",
                ]
            )
            source_pcd = Path(
                self._expand(
                    self._cfg.get(
                        "fast_lio_temp_pcd",
                        self._cfg.get("fast_lio_pcd", "~/maps/scans.pcd"),
                    )
                )
            )
        elif backend == "spark_sam":
            seq_name = str(self._cfg.get("spark_sam_sequence_name", "ranger")).strip()
            result = self._call(
                [
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    self._cfg.get("spark_sam_save_topic", "/km_sam/save_dir"),
                    "std_msgs/msg/String",
                    f"data: '{session_dir}'",
                ]
            )
            source_pcd = session_dir / f"{seq_name}_map.pcd"

        if not result.success:
            return result
        if not self._wait_for_file(source_pcd):
            return NavigationResult(False, f"地图 PCD 未生成：{source_pcd}。")
        try:
            shutil.copy2(source_pcd, cloud_path)
        except OSError as exc:
            return NavigationResult(False, f"归档点云失败：{exc}。")
        self._write_map_metadata(
            session_dir=session_dir,
            backend=backend,
            source_pcd=source_pcd,
            cloud_path=cloud_path,
        )

        if not self._cfg.get("auto_convert_pcd", True):
            self._stop_mapping_after_save()
            return NavigationResult(
                True,
                f"地图已保存到 {session_dir}，建图已退出。",
                self._map_paths(session_dir, cloud_path),
            )

        result = self._convert_pcd(cloud_path, session_dir)
        if result.success:
            self._write_map_metadata(
                session_dir=session_dir,
                backend=backend,
                source_pcd=source_pcd,
                cloud_path=cloud_path,
            )
            self._stop_mapping_after_save()
            return NavigationResult(
                True,
                f"地图已保存并转换到 {session_dir}，建图已退出。",
                self._map_paths(session_dir, cloud_path),
            )
        return result

    def start_navigation(
        self,
        map_yaml: str | None = None,
        pcd_map_path: str | None = None,
        mode: str | None = None,
        *,
        initial_pose: WaypointRecord | None = None,
        rviz: bool = True,
    ) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始导航。")

        mode = str(mode or self._cfg.get("navigation_mode", "3dloc")).strip().lower()
        if mode not in {"3dloc", "amcl"}:
            return NavigationResult(False, f"未知导航定位模式：{mode}。")

        map_yaml = self._expand(
            map_yaml or self._cfg.get("map_yaml", "~/maps/map.yaml")
        )
        if not Path(map_yaml).is_file():
            return NavigationResult(False, "未找到地图文件，无法开始导航。")
        launch_file = str(
            self._cfg.get(
                "navigation_launch_3d" if mode == "3dloc" else "navigation_launch_2d",
                self._cfg.get("navigation_launch", "navigation.launch.py"),
            )
        )
        launch_args = [
            "ros2",
            "launch",
            self._package(),
            launch_file,
            f"map:={map_yaml}",
        ]
        if mode == "3dloc":
            pcd_map_path = self._expand(
                pcd_map_path
                or self._cfg.get(
                    "pcd_map_path",
                    self._cfg.get("fast_lio_pcd", "~/maps/scans.pcd"),
                )
            )
            if not Path(pcd_map_path).is_file():
                return NavigationResult(False, f"未找到 3D PCD 地图：{pcd_map_path}。")
            launch_args.append(f"pcd_map_path:={pcd_map_path}")
        launch_args.append(f"rviz:={'true' if rviz else 'false'}")
        if initial_pose is not None:
            yaw = math.atan2(
                2.0 * (initial_pose.qw * initial_pose.qz + initial_pose.qx * initial_pose.qy),
                1.0 - 2.0 * (initial_pose.qy ** 2 + initial_pose.qz ** 2),
            )
            launch_args.extend([
                "set_initial_pose:=true",
                f"initial_pose_x:={initial_pose.x}",
                f"initial_pose_y:={initial_pose.y}",
                f"initial_pose_yaw:={yaw}",
            ])
        if rviz:
            launch_args = [
                "/bin/bash",
                "-lc",
                'source "$KRT_WORKSPACE/deploy/systemd/krt-rviz-env.sh"; exec "$@"',
                "bash",
                *launch_args,
            ]
        if self._running(self._navigation_process):
            return NavigationResult(True, "导航已经启动。")

        self._stop_mapping()
        try:
            self._navigation_process = self._popen(
                launch_args,
                start_new_session=True,
            )
        except OSError as exc:
            return NavigationResult(False, f"启动导航失败：{exc}。")
        process = self._navigation_process
        if process is None:
            return NavigationResult(False, "导航进程未创建。")
        logger.info(
            f"已启动导航 launch: mode={mode}, launch={launch_file}, pid={process.pid}"
        )
        if mode == "3dloc" and initial_pose is not None:
            ready = self._wait_for_navigation_ready(process)
            if not ready.success:
                self._stop_navigation()
                return ready
        return NavigationResult(True, "已开始导航。")

    def mapping_running(self) -> bool:
        return self._running(self._mapping_process)

    def navigation_running(self) -> bool:
        return self._running(self._navigation_process)

    def default_navigation_mode(self) -> str:
        return str(self._cfg.get("navigation_mode", "3dloc")).strip().lower()

    def stop_navigation(self) -> NavigationResult:
        if self._running(self._cruise_process):
            self.stop_cruise()
        if not self._running(self._navigation_process):
            launch_files = {
                self._cfg.get("navigation_launch", "navigation.launch.py"),
                self._cfg.get("navigation_launch_2d", "navigation.launch.py"),
                self._cfg.get("navigation_launch_3d", "navigation_3dloc.launch.py"),
            }
            if any(self._stop_launches_by_name(str(name)) for name in launch_files):
                return NavigationResult(True, "导航已退出。")
            return NavigationResult(False, "导航未启动，无法结束导航。")
        self._stop_navigation()
        return NavigationResult(True, "导航已退出。")

    def mark_waypoint(
        self,
        name: str | None = None,
        *,
        routine: str = "",
        map_id: str | None = None,
    ) -> NavigationResult:
        cmd = self._waypoint_cmd(["mark"], map_id=map_id)
        if name:
            cmd.append(name)
        if routine:
            cmd.extend(["--routine", routine])
        result = self._call(cmd)
        if result.success:
            return NavigationResult(True, f"已保存点位{name or ''}。")
        return result

    def bind_waypoint(self, name: str, routine: str) -> NavigationResult:
        return self._call(self._waypoint_cmd(["bind", name, "--routine", routine]))

    def list_waypoints(self) -> NavigationResult:
        try:
            result = self._call_completed(self._waypoint_cmd(["list"]))
        except subprocess.TimeoutExpired:
            return NavigationResult(False, "ROS 命令超时。")
        except OSError as exc:
            return NavigationResult(False, f"ROS 命令执行失败：{exc}。")
        if result.returncode == 0:
            output = (result.stdout or "").strip()
            return NavigationResult(True, output or "没有保存的点位。")
        return self._completed_to_result(result)

    def remove_waypoint(self, name: str) -> NavigationResult:
        if not name:
            return NavigationResult(False, "请说明要删除的点位名称。")
        result = self._call(self._waypoint_cmd(["remove", name]))
        if result.success:
            return NavigationResult(True, f"已删除点位{name}。")
        return result

    def continue_waypoint_input(self) -> NavigationResult:
        result = self._call(self._waypoint_cmd(["continue_input"]))
        if result.success:
            return NavigationResult(True, "已继续执行。")
        return result

    def start_cruise(
        self,
        names: list[str] | None = None,
        *,
        repeat: int | None = None,
        loop: bool = False,
        map_id: str | None = None,
    ) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始巡航。")
        with self._cruise_lock:
            if self._running(self._cruise_process):
                return NavigationResult(False, "巡航已经在执行。")
            cmd = self._waypoint_cmd(["cruise"], map_id=map_id)
            if repeat is not None:
                cmd.extend(["--repeat", str(repeat)])
            if loop:
                cmd.append("--loop")
            cmd.extend(names or [])
            try:
                self._cruise_process = self._popen(cmd, start_new_session=True)
            except OSError as exc:
                return NavigationResult(False, f"启动巡航失败：{exc}。")
            process = self._cruise_process
            if process is None:
                return NavigationResult(False, "巡航进程未创建。")
            logger.info(f"已启动 waypoint 巡航: pid={process.pid}")
            return NavigationResult(True, "已开始巡航。")

    def stop_cruise(self) -> NavigationResult:
        process = self._cruise_process
        if process is None or not self._running(process):
            return NavigationResult(False, "巡航未启动，无法停止。")
        logger.info(f"正在停止 waypoint 巡航: pid={process.pid}")
        self._terminate_process_tree(process)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            logger.warning(f"巡航未及时退出，强制结束: pid={process.pid}")
            self._kill_process_tree(process)
            process.wait(timeout=3)
        return NavigationResult(True, "巡航已停止。")

    def _convert_pcd(self, pcd_path: Path, session_dir: Path) -> NavigationResult:
        map_prefix = str(self._cfg.get("session_map_prefix", "map"))
        out_prefix = session_dir / map_prefix
        params_path = session_dir / "pcd2pgm.yaml"
        self._write_pcd2pgm_params(pcd_path, params_path)

        process: Any | None = None
        try:
            process = self._popen(
                [
                    "ros2",
                    "run",
                    "pcd2pgm",
                    "pcd2pgm_node",
                    "--ros-args",
                    "--params-file",
                    str(params_path),
                ],
                start_new_session=True,
            )
            result = self._save_published_map(out_prefix, process)
            if not result.success:
                return result
            latest_result = self._update_latest_map(out_prefix)
            if not latest_result.success:
                return latest_result
            return NavigationResult(True, "地图已保存并转换完成。")
        except OSError as exc:
            return NavigationResult(False, f"启动 pcd2pgm 失败：{exc}。")
        finally:
            if process is not None and self._running(process):
                self._terminate_process_tree(process)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._kill_process_tree(process)
                    process.wait(timeout=3)

    def _write_pcd2pgm_params(self, pcd_path: Path, params_path: Path) -> None:
        lidar_height = self._config_float("pcd2pgm_lidar_height", 0.30)
        params = {
            "pcd2pgm": {
                "ros__parameters": {
                    "pcd_file": str(pcd_path),
                    "odom_to_lidar_odom": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "flag_pass_through": False,
                    "map_resolution": self._config_float("pcd2pgm_resolution", 0.05),
                    "map_topic_name": "map",
                    "thre_radius": self._config_float("pcd2pgm_ror_radius", 0.1),
                    "thres_point_count": self._config_int("pcd2pgm_ror_min_pts", 10),
                    "thre_z_min": -lidar_height
                    + self._config_float("pcd2pgm_z_min", 0.15),
                    "thre_z_max": -lidar_height
                    + self._config_float("pcd2pgm_z_max", 1.2),
                }
            }
        }
        with params_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(params, f, sort_keys=False)

    def _save_published_map(
        self,
        out_prefix: Path,
        pcd2pgm_process: Any,
    ) -> NavigationResult:
        timeout_s = self._config_float("map_save_wait_s", 30.0)
        deadline = time.monotonic() + timeout_s
        cmd = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-t",
            "map",
            "-f",
            str(out_prefix),
            "--fmt",
            "pgm",
            "--mode",
            "trinary",
        ]
        last_error = "等待 /map 生成超时。"
        while time.monotonic() < deadline:
            if pcd2pgm_process.poll() is not None:
                return NavigationResult(False, "pcd2pgm 节点提前退出，地图未生成。")
            try:
                completed = self._call_completed(cmd)
            except subprocess.TimeoutExpired:
                last_error = "map_saver_cli 超时。"
            except OSError as exc:
                last_error = f"map_saver_cli 执行失败：{exc}。"
            else:
                if completed.returncode == 0:
                    return NavigationResult(True, "地图已保存。")
                detail = (
                    (completed.stderr or completed.stdout or "").strip().splitlines()
                )
                if detail:
                    last_error = detail[-1]
            self._sleep(0.5)
        return NavigationResult(False, f"地图转换失败：{last_error}")

    def _create_map_session_dir(self) -> Path:
        archive_dir = Path(
            self._expand(
                self._cfg.get("map_archive_dir", self._cfg.get("map_dir", "~/maps"))
            )
        )
        timestamp_format = str(self._cfg.get("map_timestamp_format", "%Y%m%d_%H%M%S"))
        timestamp = datetime.now().strftime(timestamp_format)
        session_dir = archive_dir / timestamp
        suffix = 1
        while session_dir.exists():
            session_dir = archive_dir / f"{timestamp}_{suffix:02d}"
            suffix += 1
        session_dir.mkdir(parents=True, exist_ok=False)
        return session_dir

    def _wait_for_file(self, path: Path) -> bool:
        timeout_s = self._config_float("map_save_wait_s", 30.0)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if path.is_file() and path.stat().st_size > 0:
                return True
            self._sleep(0.2)
        return False

    def _update_latest_map(self, out_prefix: Path) -> NavigationResult:
        map_yaml = out_prefix.with_suffix(".yaml")
        map_pgm = out_prefix.with_suffix(".pgm")
        latest_yaml = Path(
            self._expand(
                self._cfg.get(
                    "latest_map_yaml",
                    self._cfg.get("map_yaml", "~/maps/map.yaml"),
                )
            )
        )
        latest_pgm = Path(
            self._expand(
                self._cfg.get("latest_map_pgm", str(latest_yaml.with_suffix(".pgm")))
            )
        )
        try:
            latest_yaml.parent.mkdir(parents=True, exist_ok=True)
            latest_pgm.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(map_yaml, latest_yaml)
            shutil.copy2(map_pgm, latest_pgm)
        except OSError as exc:
            return NavigationResult(False, f"更新最新地图失败：{exc}。")
        return NavigationResult(True, "最新地图已更新。")

    def _write_map_metadata(
        self,
        *,
        session_dir: Path,
        backend: str,
        source_pcd: Path,
        cloud_path: Path,
    ) -> None:
        map_prefix = str(self._cfg.get("session_map_prefix", "map"))
        map_yaml = session_dir / f"{map_prefix}.yaml"
        map_pgm = session_dir / f"{map_prefix}.pgm"
        metadata = {
            "backend": backend,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "source_pcd": str(source_pcd),
            "cloud_pcd": str(cloud_path),
            "map_yaml": str(map_yaml) if map_yaml.exists() else "",
            "map_pgm": str(map_pgm) if map_pgm.exists() else "",
        }
        metadata_path = session_dir / "metadata.yaml"
        with metadata_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, allow_unicode=True, sort_keys=False)

    def _map_paths(self, session_dir: Path, cloud_path: Path) -> dict[str, Any]:
        map_prefix = str(self._cfg.get("session_map_prefix", "map"))
        return {
            "session_dir": str(session_dir.resolve()),
            "yaml_path": str((session_dir / f"{map_prefix}.yaml").resolve()),
            "pgm_path": str((session_dir / f"{map_prefix}.pgm").resolve()),
            "pcd_path": str(cloud_path.resolve()),
            "metadata_path": str((session_dir / "metadata.yaml").resolve()),
            "backend": self._mapping_backend or self._backend(),
        }

    def _call(self, cmd: list[str]) -> NavigationResult:
        try:
            completed = self._call_completed(cmd)
        except subprocess.TimeoutExpired:
            return NavigationResult(False, "ROS 命令超时。")
        except OSError as exc:
            return NavigationResult(False, f"ROS 命令执行失败：{exc}。")
        return self._completed_to_result(completed)

    def _call_completed(self, cmd: list[str]) -> subprocess.CompletedProcess:
        return self._run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )

    def _wait_for_navigation_ready(self, process: Any) -> NavigationResult:
        timeout_s = self._config_float("navigation_ready_timeout_s", 30.0)
        if timeout_s <= 0:
            return NavigationResult(True, "导航就绪检查已跳过。")
        attempts = max(1, math.ceil(timeout_s))
        last_error = "导航 TF/话题尚未就绪。"
        diagnostic_cmd = ["ros2", "run", self._package(), "nav_tf_diagnostics"]
        for attempt in range(attempts):
            if not self._running(process):
                return NavigationResult(False, "导航 launch 在就绪前已退出。")
            try:
                completed = self._call_completed(diagnostic_cmd)
            except (OSError, subprocess.TimeoutExpired) as exc:
                last_error = f"导航诊断执行失败：{exc}"
            else:
                if completed.returncode == 0:
                    return NavigationResult(True, "导航 TF/话题已就绪。")
                detail = (completed.stderr or completed.stdout or "").strip()
                if detail:
                    last_error = detail.splitlines()[-1]
            if attempt < attempts - 1:
                self._sleep(1.0)
        return NavigationResult(False, f"导航启动超时：{last_error}")

    @staticmethod
    def _completed_to_result(
        completed: subprocess.CompletedProcess,
    ) -> NavigationResult:
        if completed.returncode == 0:
            return NavigationResult(True, "ROS 命令执行成功。")
        detail = (completed.stderr or completed.stdout or "").strip().splitlines()
        suffix = f"：{detail[-1]}" if detail else "。"
        return NavigationResult(False, f"ROS 命令执行失败{suffix}")

    def _waypoint_cmd(self, args: list[str], *, map_id: str | None = None) -> list[str]:
        cmd = ["ros2", "run", self._package(), "waypoint_manager"]
        options = {
            "--robot-db": self._cfg.get("robot_db"),
            "--navigate-action": self._cfg.get("navigate_action"),
            "--routine-action": self._cfg.get("waypoint_routine_action"),
            "--input-topic": self._cfg.get("waypoint_input_topic"),
            "--default-wait-ms": self._cfg.get("default_waypoint_wait_ms"),
        }
        if map_id:
            options["--map-id"] = map_id
        for key, value in options.items():
            if value is not None:
                cmd.extend([key, str(value)])
        cmd.extend(args)
        return cmd

    def _stop_mapping(self) -> None:
        process = self._mapping_process
        if process is None or not self._running(process):
            return
        logger.info(f"正在退出建图 launch: pid={process.pid}")
        self._terminate_process_tree(process)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            logger.warning(f"建图 launch 未及时退出，强制结束: pid={process.pid}")
            self._kill_process_tree(process)
            process.wait(timeout=3)
        logger.info(f"建图 launch 已退出: pid={process.pid}")
        self._mapping_backend = None

    def _stop_mapping_after_save(self) -> None:
        delay_s = self._config_float("mapping_stop_delay_s", 3.0)
        if delay_s > 0:
            logger.info(f"地图保存完成，{delay_s} 秒后退出建图 launch。")
            self._sleep(delay_s)
        self._stop_mapping()

    def _stop_navigation(self) -> None:
        process = self._navigation_process
        if process is None or not self._running(process):
            return
        logger.info(f"正在退出导航 launch: pid={process.pid}")
        self._terminate_process_tree(process)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            logger.warning(f"导航 launch 未及时退出，强制结束: pid={process.pid}")
            self._kill_process_tree(process)
            process.wait(timeout=3)
        logger.info(f"导航 launch 已退出: pid={process.pid}")

    def _stop_launches_by_name(self, launch_file: str) -> bool:
        cmd = ["ps", "-eo", "pid=,pgid=,cmd="]
        try:
            output = self._run(cmd, capture_output=True, text=True, check=False)
        except OSError as exc:
            logger.warning(f"无法查询残留 launch 进程: {exc}")
            return False
        if output.returncode != 0:
            return False

        stopped = False
        for line in output.stdout.splitlines():
            parts = line.strip().split(None, 2)
            if len(parts) != 3:
                continue
            pid_s, pgid_s, command = parts
            if f"ros2 launch {self._package()} {launch_file}" not in command:
                continue
            logger.info(f"正在退出残留 launch: pid={pid_s}, pgid={pgid_s}")
            try:
                process_group = int(pgid_s)
                os.killpg(process_group, signal.SIGTERM)
                stopped = True
            except ValueError:
                logger.warning(f"残留 launch 的进程组无效: pid={pid_s}, pgid={pgid_s}")
            except OSError as exc:
                if isinstance(exc, ProcessLookupError):
                    logger.debug(f"残留 launch 已退出: pid={pid_s}")
                else:
                    logger.warning(f"退出残留 launch 失败: pid={pid_s}, {exc}")
        return stopped

    @staticmethod
    def _terminate_process_tree(process: Any) -> None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except OSError as exc:
            if not isinstance(exc, ProcessLookupError):
                process.terminate()

    @staticmethod
    def _kill_process_tree(process: Any) -> None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except OSError as exc:
            if not isinstance(exc, ProcessLookupError):
                process.kill()

    def _config_float(self, key: str, default: float) -> float:
        try:
            return float(self._cfg.get(key, default))
        except (TypeError, ValueError):
            logger.warning(f"导航配置 {key} 无效，使用默认值 {default}")
            return default

    def _config_int(self, key: str, default: int) -> int:
        try:
            return int(self._cfg.get(key, default))
        except (TypeError, ValueError):
            logger.warning(f"导航配置 {key} 无效，使用默认值 {default}")
            return default

    def _enabled(self) -> bool:
        return bool(self._cfg.get("enabled", False))

    def _backend(self) -> str:
        return str(self._cfg.get("mapping_backend", "fast_lio")).strip()

    def _package(self) -> str:
        return str(self._cfg.get("launch_package", "ranger_nav")).strip()

    @staticmethod
    def _expand(path: str) -> str:
        return str(Path(path).expanduser())

    @staticmethod
    def _running(process: Any | None) -> bool:
        return process is not None and process.poll() is None
