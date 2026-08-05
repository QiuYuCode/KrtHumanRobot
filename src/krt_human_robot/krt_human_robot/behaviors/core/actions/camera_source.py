"""
多路相机源抽象层。

支持三种后端：
    - local : 本机 USB `/dev/videoN` (cv2.VideoCapture(index))
    - http  : 远端 FastAPI，URL 形如 {base}/snapshot/{camera_id}
    - ros   : rclpy 订阅 sensor_msgs/msg/Image (需 cv_bridge，ROS 2 apt 环境)

通过 `create_camera_source(config, camera_id)` 按 id 分发到对应实现，上层
vision.py / camera.py 只持有 CameraSource 接口。
"""

from __future__ import annotations

import platform
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from loguru import logger

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


# ============================================================================
# 规格
# ============================================================================

@dataclass
class CameraSpec:
    """单路相机配置。"""
    index: int | str = 0
    width: int = 640
    height: int = 480
    ros_topic: str = ""
    record_fps: float = 30.0


def _resolve_spec(config, camera_id: str) -> CameraSpec:
    raw = config.cameras.get(camera_id)
    if raw is None:
        raise ValueError(
            f"未定义相机 {camera_id!r}，可用: {list(config.cameras.keys())}"
        )
    index = raw.get("index", 0)
    # 支持两种写法：
    # 1) 数字索引: 0 / 2 / 4
    # 2) 设备路径: /dev/camera_left
    if isinstance(index, str):
        stripped = index.strip()
        index = int(stripped) if stripped.isdigit() else stripped

    return CameraSpec(
        index=index,
        width=int(raw.get("width", 640)),
        height=int(raw.get("height", 480)),
        ros_topic=str(raw.get("ros_topic", "")),
        record_fps=float(raw.get("record_fps", 30.0)),
    )


# ============================================================================
# 基类
# ============================================================================

class CameraSource:
    """相机源统一接口。子类需实现 `_grab_once` 与 `_open_stream`/`_close`。"""

    camera_id: str
    spec: CameraSpec

    def grab_frame(self, warmup_frames: int = 30) -> np.ndarray:
        """抓一帧 BGR numpy.ndarray。抓不到抛 RuntimeError。"""
        raise NotImplementedError

    def record_video(self, filepath: str, duration: float) -> float:
        """把流写入 mp4，返回实际录制秒数。"""
        raise NotImplementedError

    def close(self) -> None:
        pass


# ============================================================================
# LocalCameraSource
# ============================================================================

def _preferred_backend():
    if cv2 is None:
        return None
    system = platform.system()
    if system == "Windows":
        return cv2.CAP_DSHOW
    if system == "Linux":
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


class LocalCameraSource(CameraSource):
    """通过 cv2.VideoCapture(index_or_device_path) 直连本机 USB 摄像头。"""

    def __init__(self, camera_id: str, spec: CameraSpec):
        if cv2 is None:
            raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")
        self.camera_id = camera_id
        self.spec = spec

    def _open(self) -> "cv2.VideoCapture":
        cap = cv2.VideoCapture(self.spec.index, _preferred_backend())
        if not cap.isOpened():
            raise RuntimeError(
                f"相机 {self.camera_id} 打开失败 (source={self.spec.index})。"
            )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def grab_frame(self, warmup_frames: int = 30) -> np.ndarray:
        cap = self._open()
        try:
            frame = None
            for _ in range(max(1, warmup_frames)):
                ok, frame = cap.read()
                if not ok:
                    frame = None
            if frame is None:
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError(
                        f"相机 {self.camera_id} 无画面输出。"
                    )
            return frame
        finally:
            cap.release()

    def record_video(self, filepath: str, duration: float) -> float:
        cap = self._open()
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps <= 0:
                fps = self.spec.record_fps
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(filepath, fourcc, fps, (w, h))
            if not writer.isOpened():
                raise RuntimeError("视频录制初始化失败。")
            for _ in range(30):
                cap.read()
            start = time.monotonic()
            while time.monotonic() - start < duration:
                ok, frame = cap.read()
                if not ok or frame is None:
                    break
                writer.write(frame)
            writer.release()
            return time.monotonic() - start
        finally:
            cap.release()


# ============================================================================
# HttpCameraSource
# ============================================================================


class _RetriableHttpError(Exception):
    """远端 5xx 或连接异常，调用方可以据此决定是否重试。"""


def _extract_detail(resp) -> str:
    """尽力从 httpx.Response 里抽出 JSON 的 `detail` 字段，抽不到就返回原文。"""
    try:
        j = resp.json()
    except Exception:
        return (resp.text or "").strip()[:300]
    if isinstance(j, dict):
        detail = j.get("detail")
        if isinstance(detail, str):
            return detail
        if detail is not None:
            return str(detail)
    return str(j)[:300]


class HttpCameraSource(CameraSource):
    """
    通过远端 FastAPI 取流:
        GET {base}/snapshot/{camera_id} → JPEG
        GET {base}/stream/{camera_id}   → multipart MJPEG (cv2.VideoCapture 可直接解码)
    """

    def __init__(self, camera_id: str, spec: CameraSpec, config):
        if cv2 is None:
            raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")
        base = (config.camera_http_base_url or "").rstrip("/")
        if not base:
            raise RuntimeError(
                "camera_backend=http 时必须配置 camera_http_base_url。"
            )
        self.camera_id = camera_id
        self.spec = spec
        self._snapshot_url = f"{base}/snapshot/{camera_id}"
        self._stream_url = f"{base}/stream/{camera_id}"
        self._timeout = float(config.camera_http_timeout)

    def grab_frame(self, warmup_frames: int = 30) -> np.ndarray:
        # 远端 worker 刚启动 / 相机刚插上时第一次访问常返回 503，做有限重试。
        # 503 / 504 / 连接异常 → 重试；4xx → 直接抛，不重试。
        data = self._fetch_snapshot_with_retry(
            max_attempts=3, backoff=0.5,
        )
        arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(
                f"远端相机 {self.camera_id} 返回的数据无法解码 JPEG "
                f"(URL={self._snapshot_url}, bytes={len(data)})。"
            )
        return frame

    def _fetch_snapshot_with_retry(
        self, max_attempts: int = 3, backoff: float = 0.5,
    ) -> bytes:
        last_err: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            try:
                return self._fetch_snapshot_once()
            except _RetriableHttpError as e:
                last_err = e
                logger.warning(
                    "远端相机 {} 第 {}/{} 次拉取失败 (可重试): {}",
                    self.camera_id, attempt, max_attempts, e,
                )
                if attempt < max_attempts:
                    time.sleep(backoff * attempt)
                    continue
                raise RuntimeError(str(e)) from e
            except Exception as e:
                # 非可重试（4xx / JPEG 解码等）直接抛
                raise RuntimeError(str(e)) from e
        # 理论上走不到
        raise RuntimeError(
            f"远端相机 {self.camera_id} 拉取失败: {last_err}"
        )

    def _fetch_snapshot_once(self) -> bytes:
        try:
            import httpx
        except ImportError:  # pragma: no cover
            return self._fetch_via_urllib()

        try:
            r = httpx.get(self._snapshot_url, timeout=self._timeout)
        except httpx.RequestError as e:
            raise _RetriableHttpError(
                f"远端相机 {self.camera_id} 连接失败: {e} "
                f"(URL={self._snapshot_url})"
            ) from e

        if r.status_code >= 400:
            detail = _extract_detail(r)
            msg = (
                f"远端相机 {self.camera_id} HTTP {r.status_code}: {detail} "
                f"(URL={self._snapshot_url})"
            )
            if r.status_code in (500, 502, 503, 504):
                raise _RetriableHttpError(msg)
            raise RuntimeError(msg)
        return r.content

    def _fetch_via_urllib(self) -> bytes:  # pragma: no cover
        import urllib.error
        import urllib.request
        try:
            with urllib.request.urlopen(
                self._snapshot_url, timeout=self._timeout,
            ) as resp:
                return resp.read()
        except urllib.error.HTTPError as e:
            body = ""
            try:
                body = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            msg = (
                f"远端相机 {self.camera_id} HTTP {e.code}: {body or e.reason} "
                f"(URL={self._snapshot_url})"
            )
            if e.code in (500, 502, 503, 504):
                raise _RetriableHttpError(msg) from e
            raise RuntimeError(msg) from e
        except urllib.error.URLError as e:
            raise _RetriableHttpError(
                f"远端相机 {self.camera_id} 连接失败: {e.reason} "
                f"(URL={self._snapshot_url})"
            ) from e

    def record_video(self, filepath: str, duration: float) -> float:
        cap = cv2.VideoCapture(self._stream_url)
        if not cap.isOpened():
            raise RuntimeError(
                f"无法打开 MJPEG 流: {self._stream_url}"
            )
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps <= 0:
                fps = self.spec.record_fps
            # 远端 fps 在 multipart MJPEG 里通常拿不到，用 spec
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.spec.width
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.spec.height
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(filepath, fourcc, fps, (w, h))
            if not writer.isOpened():
                raise RuntimeError("视频录制初始化失败。")
            # 先吃掉若干帧等流稳定
            for _ in range(5):
                cap.read()
            start = time.monotonic()
            while time.monotonic() - start < duration:
                ok, frame = cap.read()
                if not ok or frame is None:
                    break
                writer.write(frame)
            writer.release()
            return time.monotonic() - start
        finally:
            cap.release()


# ============================================================================
# RosCameraSource
# ============================================================================

# rclpy / cv_bridge 是 ROS 2 apt 包，仅在 camera_backend=ros 时懒加载。
# 所有 RosCameraSource 共用一个 rclpy context + 后台 executor 线程。

_ROS_LOCK = threading.Lock()
_ROS_CTX = {
    "inited": False,
    "node": None,
    "executor": None,
    "thread": None,
}


def _ensure_ros_context(config) -> "Any":
    """懒初始化 rclpy + 单一节点 + 后台 spin 线程，返回共享 Node。"""
    with _ROS_LOCK:
        if _ROS_CTX["inited"]:
            return _ROS_CTX["node"]
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init(args=None)

        node = Node(config.camera_ros_node_name)
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def _spin():
            try:
                executor.spin()
            except Exception as e:  # pragma: no cover
                logger.error(f"[ros] executor spin 退出: {e}")

        t = threading.Thread(target=_spin, name="rclpy-spin", daemon=True)
        t.start()

        _ROS_CTX.update(
            inited=True, node=node, executor=executor, thread=t,
        )
        logger.info(f"[ros] rclpy 节点 {config.camera_ros_node_name} 就绪")
        return node


class RosCameraSource(CameraSource):
    """
    订阅原始或压缩 ROS 图像，内部缓存最新帧，grab_frame 返回 cache。
    record_video 周期性拉 cache 写 mp4 (FPS 取 spec.record_fps)。
    """

    def __init__(self, camera_id: str, spec: CameraSpec, config):
        if cv2 is None:
            raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")
        if not spec.ros_topic:
            raise RuntimeError(
                f"camera {camera_id} 未配置 ros_topic，无法使用 ROS 后端。"
            )

        try:
            from sensor_msgs.msg import CompressedImage, Image as RosImage
            from cv_bridge import CvBridge
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
            )
        except ImportError as e:  # pragma: no cover
            raise RuntimeError(
                "缺少 ROS 2 依赖 (rclpy / sensor_msgs / cv_bridge)。"
                " 请确认已 source /opt/ros/<distro>/setup.bash。"
            ) from e

        self.camera_id = camera_id
        self.spec = spec
        self._bridge = CvBridge()
        self._latest: np.ndarray | None = None
        self._latest_lock = threading.Lock()
        self._warmup_timeout = float(config.camera_ros_warmup_seconds)
        self._transport = str(config.camera_ros_transport).strip().lower()
        if self._transport not in {"raw", "compressed"}:
            raise ValueError(
                "camera_ros_transport 必须是 raw 或 compressed，"
                f"当前为 {self._transport!r}。"
            )

        node = _ensure_ros_context(config)

        def _cb(msg):
            try:
                if self._transport == "compressed":
                    data = np.frombuffer(msg.data, dtype=np.uint8)
                    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError("JPEG 数据无法解码")
                else:
                    frame = self._bridge.imgmsg_to_cv2(
                        msg, desired_encoding="bgr8"
                    )
            except Exception as e:  # pragma: no cover
                logger.warning(f"[ros] {camera_id} 图像解码失败: {e}")
                return
            with self._latest_lock:
                self._latest = frame

        qos = QoSProfile(
            depth=int(config.camera_ros_qos_depth),
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        message_type = CompressedImage if self._transport == "compressed" else RosImage
        topic = (
            f"{spec.ros_topic}/compressed"
            if self._transport == "compressed"
            else spec.ros_topic
        )
        self._sub = node.create_subscription(message_type, topic, _cb, qos)
        self._node = node

    def _wait_first_frame(self):
        deadline = time.monotonic() + self._warmup_timeout
        while time.monotonic() < deadline:
            with self._latest_lock:
                if self._latest is not None:
                    return
            time.sleep(0.02)
        raise RuntimeError(
            f"ROS 相机 {self.camera_id} 话题 {self.spec.ros_topic} "
            f"在 {self._warmup_timeout}s 内未收到帧。"
        )

    def grab_frame(self, warmup_frames: int = 30) -> np.ndarray:
        self._wait_first_frame()
        with self._latest_lock:
            return self._latest.copy()

    def record_video(self, filepath: str, duration: float) -> float:
        self._wait_first_frame()
        with self._latest_lock:
            h, w = self._latest.shape[:2]
        fps = self.spec.record_fps
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(filepath, fourcc, fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError("视频录制初始化失败。")
        try:
            period = 1.0 / max(1.0, fps)
            start = time.monotonic()
            next_t = start
            while time.monotonic() - start < duration:
                with self._latest_lock:
                    frame = None if self._latest is None else self._latest.copy()
                if frame is not None:
                    writer.write(frame)
                next_t += period
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
            writer.release()
            return time.monotonic() - start
        finally:
            writer.release()

    def close(self) -> None:
        try:
            if self._sub is not None and self._node is not None:
                self._node.destroy_subscription(self._sub)
                self._sub = None
        except Exception:  # pragma: no cover
            pass


# ============================================================================
# 工厂
# ============================================================================

def create_camera_source(config, camera_id: str | None = None) -> CameraSource:
    """
    按配置创建相机源。`camera_id=None` 时使用 config.default_camera。
    """
    cid = camera_id or config.default_camera
    spec = _resolve_spec(config, cid)
    backend = (config.camera_backend or "local").lower()
    if backend == "local":
        return LocalCameraSource(cid, spec)
    if backend == "http":
        return HttpCameraSource(cid, spec, config)
    if backend == "ros":
        return RosCameraSource(cid, spec, config)
    raise ValueError(f"未知 camera_backend: {backend!r}")


__all__ = [
    "CameraSpec",
    "CameraSource",
    "LocalCameraSource",
    "HttpCameraSource",
    "RosCameraSource",
    "create_camera_source",
]
