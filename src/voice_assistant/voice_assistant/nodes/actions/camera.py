"""相机动作节点 - 拍照 & 录制视频 (多路相机)"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import py_trees
from loguru import logger
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import RobotConfig
from voice_assistant.nodes.actions.camera_source import create_camera_source

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


# ============================================================================
# 内部工具
# ============================================================================

def _ensure_save_dir(save_dir: str) -> Path:
    p = Path(save_dir)
    p.mkdir(parents=True, exist_ok=True)
    return p


def _timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _resolve_camera_id(config: RobotConfig, camera_id: str | None) -> str:
    return camera_id or config.default_camera


# ============================================================================
# 独立执行函数
# ============================================================================

def capture_frame_as_base64(
    config: RobotConfig,
    save_copy: bool = True,
    camera_id: str | None = None,
) -> tuple[str, str | None]:
    """抓一帧并返回 base64 JPEG，可选同时落盘。

    Returns:
        (base64_str, filepath_or_none)

    Raises:
        RuntimeError: 相机不可用或编码失败。
    """
    import base64

    if cv2 is None:
        raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")

    cid = _resolve_camera_id(config, camera_id)
    src = create_camera_source(config, cid)
    try:
        frame = src.grab_frame()
    finally:
        src.close()

    ok, buf = cv2.imencode(".jpg", frame)
    if not ok:
        raise RuntimeError("JPEG 编码失败。")
    b64 = base64.b64encode(buf.tobytes()).decode("utf-8")

    filepath = None
    if save_copy:
        save_dir = _ensure_save_dir(config.camera_save_dir)
        filename = f"photo_{cid}_{_timestamp()}.jpg"
        filepath = str(save_dir / filename)
        cv2.imwrite(filepath, frame)
        logger.info("视觉拍照已保存 [{}]: {}", cid, filepath)

    return b64, filepath


def execute_take_photo(
    config: RobotConfig, camera_id: str | None = None
) -> str:
    """拍照核心逻辑。"""
    if cv2 is None:
        raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")

    cid = _resolve_camera_id(config, camera_id)
    src = create_camera_source(config, cid)
    try:
        frame = src.grab_frame()
    finally:
        src.close()

    save_dir = _ensure_save_dir(config.camera_save_dir)
    filename = f"photo_{cid}_{_timestamp()}.jpg"
    filepath = save_dir / filename
    cv2.imwrite(str(filepath), frame)
    logger.info("照片已保存 [{}]: {}", cid, filepath)
    return "拍照成功，照片已保存。"


def execute_record_video(
    config: RobotConfig,
    duration: float | None = None,
    camera_id: str | None = None,
) -> str:
    """录制视频核心逻辑。"""
    if cv2 is None:
        raise RuntimeError("相机依赖缺失，请先安装 opencv-python。")

    cid = _resolve_camera_id(config, camera_id)
    if duration is None:
        duration = config.camera_record_seconds

    save_dir = _ensure_save_dir(config.camera_save_dir)
    filename = f"video_{cid}_{_timestamp()}.mp4"
    filepath = save_dir / filename

    src = create_camera_source(config, cid)
    try:
        logger.info(
            "开始录制 [{}]: {:.0f}s → {}", cid, duration, filepath
        )
        elapsed = src.record_video(str(filepath), duration)
    finally:
        src.close()

    logger.info("视频已保存 [{}]: {} ({:.1f}s)", cid, filepath, elapsed)
    return f"视频录制完成，共 {elapsed:.0f} 秒，已保存。"


# ============================================================================
# 行为树节点
# ============================================================================

class TakePhotoAction(Behaviour):
    """
    拍照并保存到本地。

    intent == "take_photo" 时执行，使用 config.default_camera。
    """

    def __init__(self, name: str, config: RobotConfig, camera_id: str | None = None):
        super().__init__(name)
        self._config = config
        self._camera_id = camera_id

        self.blackboard = self.attach_blackboard_client(
            name="TakePhotoAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.intent != "take_photo":
            return Status.FAILURE

        self.logger.info("执行: 拍照")
        try:
            self.blackboard.response_text = execute_take_photo(
                self._config, camera_id=self._camera_id,
            )
            return Status.SUCCESS
        except RuntimeError as e:
            self.logger.error(str(e))
            self.blackboard.response_text = str(e)
            return Status.FAILURE


class RecordVideoAction(Behaviour):
    """
    录制视频并保存到本地。

    intent == "record_video" 时执行，使用 config.default_camera。
    录制时长由 config.camera_record_seconds 控制。
    """

    def __init__(self, name: str, config: RobotConfig, camera_id: str | None = None):
        super().__init__(name)
        self._config = config
        self._camera_id = camera_id

        self.blackboard = self.attach_blackboard_client(
            name="RecordVideoAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.intent != "record_video":
            return Status.FAILURE

        self.logger.info("执行: 录制视频")
        try:
            self.blackboard.response_text = execute_record_video(
                self._config, camera_id=self._camera_id,
            )
            return Status.SUCCESS
        except RuntimeError as e:
            self.logger.error(str(e))
            self.blackboard.response_text = str(e)
            return Status.FAILURE
