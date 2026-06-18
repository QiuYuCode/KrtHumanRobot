"""视觉理解动作节点 - 拍照 + VLM 多模态分析 (多路相机)"""

from __future__ import annotations

import threading
import time

import py_trees
from loguru import logger
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import RobotConfig
from voice_assistant.nodes.actions.camera import capture_frame_as_base64
from voice_assistant.nodes.actions.llm_dialog import (
    _create_chat_model,
    _invoke_with_timeout,
)
from voice_assistant.nodes.actions.robot_arm import _normalize_side, execute_robot_arm


# ============================================================================
# VLM 构造
# ============================================================================

def _create_vlm(config: RobotConfig):
    """根据 config.vlm_provider 创建 LangChain Chat Model (需支持多模态)。"""
    return _create_chat_model(
        provider=config.vlm_provider,
        model=config.vlm_model,
        base_url=config.vlm_base_url,
        api_key=config.vlm_api_key,
        timeout=config.vlm_request_timeout,
        max_retries=config.vlm_max_retries,
    )


def _create_local_vlm(config: RobotConfig):
    """创建本地 VLM 回退模型。"""
    return _create_chat_model(
        provider=config.local_vlm_provider,
        model=config.local_vlm_model,
        base_url=config.local_vlm_base_url,
        timeout=config.vlm_request_timeout,
        max_retries=0,
    )


def _is_cloud_vlm(config: RobotConfig) -> bool:
    return config.vlm_provider.lower() != config.local_vlm_provider.lower()


# ============================================================================
# 独立执行函数
# ============================================================================

def execute_describe_scene(
    config: RobotConfig,
    question: str = "请描述你看到的场景",
    camera_id: str | None = None,
) -> str:
    """拍照 + VLM 分析。`camera_id=None` 时使用 config.default_camera。"""
    from langchain_core.messages import HumanMessage, SystemMessage

    cid = camera_id or config.default_camera
    b64, filepath = capture_frame_as_base64(config, save_copy=True, camera_id=cid)

    messages = [
        SystemMessage(content=config.vlm_system_prompt),
        HumanMessage(content=[
            {"type": "text", "text": question},
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
            },
        ]),
    ]

    try:
        vlm = _create_vlm(config)
        desc = _invoke_with_timeout(vlm, messages, config.vlm_request_timeout).content
    except Exception as primary_err:
        if not (
            config.cloud_vlm_fallback_to_local and _is_cloud_vlm(config)
        ):
            raise
        logger.warning(
            "云端 VLM 调用失败，尝试本地回退: provider={}, model={}, error={}",
            config.vlm_provider, config.vlm_model, primary_err,
        )
        try:
            local_vlm = _create_local_vlm(config)
            desc = _invoke_with_timeout(
                local_vlm, messages, config.vlm_request_timeout,
            ).content
            logger.info(
                "本地 VLM 回退完成: provider={}, model={}",
                config.local_vlm_provider, config.local_vlm_model,
            )
        except Exception as fallback_err:
            logger.error(
                "本地 VLM 回退失败，模型可能不可用或不支持图片输入: "
                "provider={}, model={}, error={}",
                config.local_vlm_provider,
                config.local_vlm_model,
                fallback_err,
            )
            return "视觉分析暂时不可用。"

    logger.info(
        "视觉分析完成 [{}]: {} (图片: {})",
        cid, desc[:80], filepath or "未保存",
    )
    return desc


# ============================================================================
# 行为树节点
# ============================================================================

class DescribeSceneAction(Behaviour):
    """
    通用视觉理解动作: 根据 intent 决定触发条件与使用的相机。

    默认实例触发 intent == "describe_scene"，使用 config.default_camera。
    通过子类或参数可扩展出 describe_left_palm / describe_right_palm。
    掌心预置机械臂：默认后台回放 + 各手掌独立的 capture_delay_s 后拍照（见 DescribeLeft/RightPalmAction）。
    """

    INTENT: str = "describe_scene"
    CAMERA_ID: str | None = None

    def __init__(
        self,
        name: str,
        config: RobotConfig,
        intent: str | None = None,
        camera_id: str | None = None,
        preset_group: str | None = None,
        preset_arm_side: str | None = None,
        preset_capture_delay_s: float = 2.5,
        preset_wait_replay_finish: bool = False,
    ):
        super().__init__(name)
        self._config = config
        self._intent = intent or self.INTENT
        self._camera_id = camera_id if camera_id is not None else self.CAMERA_ID
        self._preset_group = (preset_group or "").strip() or None
        self._preset_arm_side = preset_arm_side
        self._preset_capture_delay_s = float(preset_capture_delay_s)
        self._preset_wait_replay_finish = bool(preset_wait_replay_finish)

        self.blackboard = self.attach_blackboard_client(
            name=name, namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def _run_preset_arm_before_vision(self) -> None:
        """若配置了 preset 动作组，则启动对应臂的示教回放（默认与拍照并行）。"""
        if not self._preset_group or not self._preset_arm_side:
            return
        if not self._config.robot_arm_enabled:
            self.logger.debug(
                f"掌心视觉预置动作跳过: robot_arm_enabled=False (group={self._preset_group})"
            )
            return
        cfg = self._config
        self.logger.info(
            f"掌心视觉前预置: arm={self._preset_arm_side} group={self._preset_group}"
        )

        def _replay() -> None:
            try:
                execute_robot_arm(
                    cfg,
                    action="",
                    arm_side=self._preset_arm_side,
                    operation="run_group",
                    group_name=self._preset_group,
                )
            except Exception as exc:
                logger.exception(
                    f"掌心视觉预置机械臂后台回放失败: arm={self._preset_arm_side} "
                    f"group={self._preset_group}: {exc}"
                )

        if self._preset_wait_replay_finish:
            _replay()
            return

        threading.Thread(
            target=_replay,
            name="palm-preset-arm-replay",
            daemon=True,
        ).start()
        delay = max(0.0, float(self._preset_capture_delay_s))
        if delay > 0:
            self.logger.info(
                f"掌心视觉预置: 回放并行进行中，{delay:.1f}s 后拍照"
            )
            time.sleep(delay)

    def update(self):
        if self.blackboard.intent != self._intent:
            return Status.FAILURE

        command = self.blackboard.user_command
        cid = self._camera_id or self._config.default_camera
        self.logger.info(f"执行: 视觉理解 intent={self._intent} camera={cid} ({command})")

        try:
            self._run_preset_arm_before_vision()
            desc = execute_describe_scene(
                self._config, question=command, camera_id=cid,
            )
            self.blackboard.response_text = desc
            return Status.SUCCESS
        except Exception as e:
            self.logger.error(f"视觉理解失败: {e}")
            self.blackboard.response_text = f"视觉分析失败: {e}"
            return Status.SUCCESS


class DescribeLeftPalmAction(DescribeSceneAction):
    """intent == 'describe_left_palm' 时，用左掌心相机触发 VLM 分析。"""

    INTENT = "describe_left_palm"
    CAMERA_ID = "left_palm"

    def __init__(
        self,
        name: str,
        config: RobotConfig,
        intent: str | None = None,
        camera_id: str | None = None,
    ):
        raw_side = (config.describe_left_palm_preset_arm_side or "left").strip()
        arm_side = _normalize_side(raw_side) or (
            raw_side.lower() if raw_side.lower() in ("left", "right") else "left"
        )
        preset = (config.describe_left_palm_preset_group or "").strip() or None
        super().__init__(
            name,
            config,
            intent=intent,
            camera_id=camera_id,
            preset_group=preset,
            preset_arm_side=arm_side,
            preset_capture_delay_s=float(config.describe_left_palm_preset_capture_delay_s),
            preset_wait_replay_finish=bool(config.describe_left_palm_preset_wait_replay_finish),
        )


class DescribeRightPalmAction(DescribeSceneAction):
    """intent == 'describe_right_palm' 时，用右掌心相机触发 VLM 分析。"""

    INTENT = "describe_right_palm"
    CAMERA_ID = "right_palm"

    def __init__(
        self,
        name: str,
        config: RobotConfig,
        intent: str | None = None,
        camera_id: str | None = None,
    ):
        raw_side = (config.describe_right_palm_preset_arm_side or "right").strip()
        arm_side = _normalize_side(raw_side) or (
            raw_side.lower() if raw_side.lower() in ("left", "right") else "right"
        )
        preset = (config.describe_right_palm_preset_group or "").strip() or None
        super().__init__(
            name,
            config,
            intent=intent,
            camera_id=camera_id,
            preset_group=preset,
            preset_arm_side=arm_side,
            preset_capture_delay_s=float(config.describe_right_palm_preset_capture_delay_s),
            preset_wait_replay_finish=bool(config.describe_right_palm_preset_wait_replay_finish),
        )
