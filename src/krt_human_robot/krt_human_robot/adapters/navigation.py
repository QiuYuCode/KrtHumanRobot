"""ROS CLI adapter for ranger_nav mapping and navigation."""

from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import yaml

try:
    from loguru import logger
except ImportError:
    logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class NavigationResult:
    success: bool
    message: str


class RangerNavAdapter:
    """Owns ranger_nav launch processes started by the core tree."""

    def __init__(
        self,
        config: Any,
        *,
        popen: Callable[..., subprocess.Popen] = subprocess.Popen,
        run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._cfg = config.adapters.get("navigation", {})
        self._popen = popen
        self._run = run
        self._sleep = sleep
        self._mapping_process: subprocess.Popen | None = None
        self._navigation_process: subprocess.Popen | None = None
        self._cruise_process: subprocess.Popen | None = None

    def start_mapping(self) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始建图。")
        if self._running(self._mapping_process):
            return NavigationResult(True, "建图已经启动。")

        backend = self._backend()
        launch_file = {
            "fast_lio": self._cfg.get("mapping_launch_fast_lio", "mapping.launch.py"),
            "spark_sam": self._cfg.get(
                "mapping_launch_spark_sam", "mapping_sam.launch.py"
            ),
        }.get(backend)
        if launch_file is None:
            return NavigationResult(False, f"未知建图后端：{backend}。")

        try:
            self._mapping_process = self._popen(
                ["ros2", "launch", self._package(), launch_file],
                start_new_session=True,
            )
        except OSError as exc:
            return NavigationResult(False, f"启动建图失败：{exc}。")
        logger.info(
            f"已启动建图 launch: backend={backend}, pid={self._mapping_process.pid}"
        )
        return NavigationResult(True, "已开始建图。")

    def save_mapping(self) -> NavigationResult:
        if not self._running(self._mapping_process):
            return NavigationResult(False, "建图未启动，无法保存地图。")

        backend = self._backend()
        if backend not in {"fast_lio", "spark_sam"}:
            return NavigationResult(False, f"未知建图后端：{backend}。")

        session_dir = self._create_map_session_dir()
        cloud_path = session_dir / str(
            self._cfg.get("session_cloud_filename", "cloud.pcd")
        )
        if backend == "fast_lio":
            result = self._call([
                "ros2",
                "service",
                "call",
                self._cfg.get("fast_lio_save_service", "/map_save"),
                "std_srvs/srv/Trigger",
                "{}",
            ])
            source_pcd = Path(
                self._expand(
                    self._cfg.get(
                        "fast_lio_temp_pcd",
                        self._cfg.get("fast_lio_pcd", "~/maps/scans.pcd"),
                    )
                )
            )
        elif backend == "spark_sam":
            result = self._call([
                "ros2",
                "topic",
                "pub",
                "--once",
                self._cfg.get("spark_sam_save_topic", "/km_sam/save_dir"),
                "std_msgs/msg/String",
                f"data: '{session_dir}'",
            ])
            seq_name = str(self._cfg.get("spark_sam_sequence_name", "ranger")).strip()
            source_pcd = (
                session_dir / seq_name / f"{seq_name}_map.pcd"
                if seq_name
                else session_dir / "map.pcd"
            )

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
            return NavigationResult(True, f"地图已保存到 {session_dir}，建图已退出。")

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
                True, f"地图已保存并转换到 {session_dir}，建图已退出。"
            )
        return result

    def start_navigation(self) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始导航。")

        map_yaml = self._expand(self._cfg.get("map_yaml", "~/maps/map.yaml"))
        if not Path(map_yaml).is_file():
            return NavigationResult(False, "未找到地图文件，无法开始导航。")
        if self._running(self._navigation_process):
            return NavigationResult(True, "导航已经启动。")

        self._stop_mapping()
        try:
            self._navigation_process = self._popen([
                "ros2",
                "launch",
                self._package(),
                self._cfg.get("navigation_launch", "navigation.launch.py"),
                f"map:={map_yaml}",
            ], start_new_session=True)
        except OSError as exc:
            return NavigationResult(False, f"启动导航失败：{exc}。")
        logger.info(f"已启动导航 launch: pid={self._navigation_process.pid}")
        return NavigationResult(True, "已开始导航。")

    def stop_navigation(self) -> NavigationResult:
        if self._running(self._cruise_process):
            self.stop_cruise()
        if not self._running(self._navigation_process):
            if self._stop_launches_by_name(
                self._cfg.get("navigation_launch", "navigation.launch.py")
            ):
                return NavigationResult(True, "导航已退出。")
            return NavigationResult(False, "导航未启动，无法结束导航。")
        self._stop_navigation()
        return NavigationResult(True, "导航已退出。")

    def mark_waypoint(
        self,
        name: str | None = None,
        *,
        task: str = "wait",
        task_args: str = "{}",
    ) -> NavigationResult:
        cmd = self._waypoint_cmd(["mark"])
        if name:
            cmd.append(name)
        cmd.extend(["--task", task, "--args", task_args])
        result = self._call(cmd)
        if result.success:
            return NavigationResult(True, f"已保存点位{name or ''}。")
        return result

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
    ) -> NavigationResult:
        if not self._enabled():
            return NavigationResult(False, "导航功能未启用，无法开始巡航。")
        if self._running(self._cruise_process):
            return NavigationResult(True, "巡航已经在执行。")
        cmd = self._waypoint_cmd(["cruise"])
        if repeat is not None:
            cmd.extend(["--repeat", str(repeat)])
        if loop:
            cmd.append("--loop")
        cmd.extend(names or [])
        try:
            self._cruise_process = self._popen(cmd, start_new_session=True)
        except OSError as exc:
            return NavigationResult(False, f"启动巡航失败：{exc}。")
        logger.info(f"已启动 waypoint 巡航: pid={self._cruise_process.pid}")
        return NavigationResult(True, "已开始巡航。")

    def stop_cruise(self) -> NavigationResult:
        process = self._cruise_process
        if not self._running(process):
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
        out_prefix = str(session_dir / map_prefix)
        cmd = [
            "ros2",
            "run",
            self._package(),
            "pcd2pgm",
            "--pcd",
            str(pcd_path),
            "--out",
            out_prefix,
            "--lidar-height",
            str(self._cfg.get("pcd2pgm_lidar_height", 0.30)),
            "--z-min",
            str(self._cfg.get("pcd2pgm_z_min", 0.15)),
            "--z-max",
            str(self._cfg.get("pcd2pgm_z_max", 1.2)),
            "--resolution",
            str(self._cfg.get("pcd2pgm_resolution", 0.05)),
            "--occ-thresh",
            str(self._cfg.get("pcd2pgm_occ_thresh", 1)),
            "--min-blob",
            str(self._cfg.get("pcd2pgm_min_blob", 2)),
        ]
        ror_radius = float(self._cfg.get("pcd2pgm_ror_radius", 0.0))
        if ror_radius > 0:
            cmd.extend([
                "--ror-radius",
                str(ror_radius),
                "--ror-min-pts",
                str(self._cfg.get("pcd2pgm_ror_min_pts", 5)),
            ])
        result = self._call(cmd)
        if result.success:
            latest_result = self._update_latest_map(Path(out_prefix))
            if not latest_result.success:
                return latest_result
            return NavigationResult(True, "地图已保存并转换完成。")
        return result

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
        timeout_s = float(self._cfg.get("map_save_wait_s", 30.0))
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
                self._cfg.get(
                    "latest_map_pgm", str(latest_yaml.with_suffix(".pgm"))
                )
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

    @staticmethod
    def _completed_to_result(
        completed: subprocess.CompletedProcess,
    ) -> NavigationResult:
        if completed.returncode == 0:
            return NavigationResult(True, "ROS 命令执行成功。")
        detail = (completed.stderr or completed.stdout or "").strip().splitlines()
        suffix = f"：{detail[-1]}" if detail else "。"
        return NavigationResult(False, f"ROS 命令执行失败{suffix}")

    def _waypoint_cmd(self, args: list[str]) -> list[str]:
        cmd = ["ros2", "run", self._package(), "waypoint_manager"]
        options = {
            "--file": self._cfg.get("waypoints_file"),
            "--navigate-action": self._cfg.get("navigate_action"),
            "--input-topic": self._cfg.get("waypoint_input_topic"),
            "--image-topic": self._cfg.get("waypoint_image_topic"),
            "--image-dir": self._cfg.get("waypoint_image_dir"),
            "--default-wait-ms": self._cfg.get("default_waypoint_wait_ms"),
            "--tts-service": self._cfg.get("waypoint_tts_service"),
            "--tts-timeout-s": self._cfg.get("waypoint_tts_timeout_s"),
            "--vision-service": self._cfg.get("waypoint_vision_service"),
            "--arm-action": self._cfg.get("waypoint_arm_action"),
        }
        for key, value in options.items():
            if value is not None:
                cmd.extend([key, str(value)])
        cmd.extend(args)
        return cmd

    def _stop_mapping(self) -> None:
        process = self._mapping_process
        if not self._running(process):
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

    def _stop_mapping_after_save(self) -> None:
        delay_s = float(self._cfg.get("mapping_stop_delay_s", 3.0))
        if delay_s > 0:
            logger.info(f"地图保存完成，{delay_s} 秒后退出建图 launch。")
            self._sleep(delay_s)
        self._stop_mapping()

    def _stop_navigation(self) -> None:
        process = self._navigation_process
        if not self._running(process):
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
                os.killpg(int(pgid_s), signal.SIGTERM)
                stopped = True
            except ProcessLookupError:
                pass
            except OSError as exc:
                logger.warning(f"退出残留 launch 失败: pid={pid_s}, {exc}")
        return stopped

    @staticmethod
    def _terminate_process_tree(process: subprocess.Popen) -> None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError:
            process.terminate()

    @staticmethod
    def _kill_process_tree(process: subprocess.Popen) -> None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError:
            process.kill()

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
    def _running(process: subprocess.Popen | None) -> bool:
        return process is not None and process.poll() is None
