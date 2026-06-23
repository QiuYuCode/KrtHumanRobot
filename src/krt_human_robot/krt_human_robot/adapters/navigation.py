"""ROS CLI adapter for ranger_nav mapping and navigation."""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

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
        if backend == "fast_lio":
            result = self._call([
                "ros2",
                "service",
                "call",
                self._cfg.get("fast_lio_save_service", "/map_save"),
                "std_srvs/srv/Trigger",
                "{}",
            ])
            pcd_path = self._expand(self._cfg.get("fast_lio_pcd", "~/maps/scans.pcd"))
        elif backend == "spark_sam":
            map_dir = self._expand(self._cfg.get("map_dir", "~/maps"))
            result = self._call([
                "ros2",
                "topic",
                "pub",
                "--once",
                self._cfg.get("spark_sam_save_topic", "/km_sam/save_dir"),
                "std_msgs/msg/String",
                f"data: '{map_dir}'",
            ])
            pcd_path = self._expand(
                self._cfg.get("spark_sam_pcd", "~/maps/ranger/ranger_map.pcd")
            )
        else:
            return NavigationResult(False, f"未知建图后端：{backend}。")

        if not result.success:
            return result
        if not self._cfg.get("auto_convert_pcd", True):
            self._stop_mapping_after_save()
            return NavigationResult(True, "地图已保存，建图已退出。")

        result = self._convert_pcd(pcd_path)
        if result.success:
            self._stop_mapping_after_save()
            return NavigationResult(True, "地图已保存并转换完成，建图已退出。")
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
        if not self._running(self._navigation_process):
            return NavigationResult(False, "导航未启动，无法结束导航。")
        self._stop_navigation()
        return NavigationResult(True, "导航已退出。")

    def _convert_pcd(self, pcd_path: str) -> NavigationResult:
        map_yaml = self._expand(self._cfg.get("map_yaml", "~/maps/map.yaml"))
        out_prefix = str(Path(map_yaml).with_suffix(""))
        result = self._call([
            "ros2",
            "run",
            self._package(),
            "pcd2pgm",
            "--pcd",
            pcd_path,
            "--out",
            out_prefix,
            "--lidar-height",
            "0.30",
            "--z-min",
            "0.15",
            "--z-max",
            "1.2",
            "--resolution",
            "0.05",
        ])
        if result.success:
            return NavigationResult(True, "地图已保存并转换完成。")
        return result

    def _call(self, cmd: list[str]) -> NavigationResult:
        try:
            completed = self._run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return NavigationResult(False, "ROS 命令超时。")
        except OSError as exc:
            return NavigationResult(False, f"ROS 命令执行失败：{exc}。")
        if completed.returncode == 0:
            return NavigationResult(True, "ROS 命令执行成功。")
        detail = (completed.stderr or completed.stdout or "").strip().splitlines()
        suffix = f"：{detail[-1]}" if detail else "。"
        return NavigationResult(False, f"ROS 命令执行失败{suffix}")

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
