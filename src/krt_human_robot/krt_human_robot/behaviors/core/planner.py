"""LLM 任务规划器 - 通过 Function Calling 将自然语言解析为多步动作计划"""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig
from krt_human_robot.behaviors.core.actions.llm_dialog import _create_llm
from krt_human_robot.behaviors.voice import speak_blocking
from voice_interfaces.srv import SynthesizeSpeech


# ============================================================================
# LangChain Tool 定义 (仅 schema，不包含执行逻辑)
# ============================================================================

def _build_planner_tools():
    """构建供 LLM bind_tools 使用的工具定义列表。"""
    from langchain_core.tools import tool

    @tool
    def take_photo() -> str:
        """拍摄一张照片并保存到本地。当用户要求拍照、拍张照、看一下时调用。"""
        return "take_photo called"

    @tool
    def record_video(duration: int = 10) -> str:
        """录制一段视频并保存到本地。

        Args:
            duration: 录制时长（秒），默认 10 秒。
        """
        return "record_video called"

    @tool
    def navigate(destination: str) -> str:
        """导航机器人前往指定位置。当用户要求去某个地方、前往、导航时调用。

        Args:
            destination: 目标位置名称，如"客厅"、"厨房"。
        """
        return "navigate called"

    @tool
    def control_robot_arm(
        arm_side: str,
        operation: str,
        group_name: str = "",
        action: str = "",
    ) -> str:
        """控制机械臂示教流程或执行动作组。

        Args:
            arm_side: 机械臂侧别，left 或 right。
            operation: 操作类型，enter_teach / exit_teach / run_group。
            group_name: 动作组名称。exit_teach/run_group 时建议传入。
            action: 原始动作描述，可选。
        """
        return "control_robot_arm called"

    @tool
    def control_gripper(hand: str, action: str) -> str:
        """控制左手或右手夹爪。

        Args:
            hand: 左右手，必须为 left 或 right。
            action: 夹爪动作，支持 shake、open、handshake。
        """
        return "control_gripper called"

    @tool
    def run_routine(routine_name: str) -> str:
        """执行已经保存的机器人流程，如唱跳表演、迎宾展示。

        Args:
            routine_name: robot.db 中的流程名称。
        """
        return "run_routine called"

    @tool
    def describe_scene(question: str = "请描述你看到的场景") -> str:
        """用相机拍照并分析画面内容。当用户要求看看、描述场景、识别物体时调用。

        Args:
            question: 用户想了解的具体问题，如"前面有什么"、"这是什么东西"。
        """
        return "describe_scene called"

    @tool
    def exit_conversation() -> str:
        """结束当前对话，回到待机状态。当用户说退出、结束、没事了时调用。"""
        return "exit_conversation called"

    return [
        take_photo,
        record_video,
        navigate,
        control_robot_arm,
        control_gripper,
        run_routine,
        describe_scene,
        exit_conversation,
    ]


# ============================================================================
# LLM 任务规划器 (行为树节点)
# ============================================================================

class LLMTaskPlanner(Behaviour):
    """
    基于 LLM Function Calling 的多指令任务规划器。

    替代 RecognizeIntent：将用户自然语言指令解析为一个或多个 tool_calls。
    - 有 tool_calls → 写 action_plan 列表到 blackboard，intent = "execute_plan"
    - 无 tool_calls → 纯对话回复，intent = "chat"，直接写 response_text
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self.conversation_history: list = []
        self.llm = None
        self.llm_with_tools = None
        self.tools = None
        self._node = None
        self._tts_client = None

        self.blackboard = self.attach_blackboard_client(
            name="LLMTaskPlanner", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="action_plan", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is not None:
            self._tts_client = self._node.create_client(
                SynthesizeSpeech, "/voice/tts/synthesize"
            )
        try:
            self.llm = _create_llm(self.config)
            self.tools = _build_planner_tools()
            self.llm_with_tools = self.llm.bind_tools(self.tools)
            self.logger.info(
                f"任务规划器就绪: provider={self.config.llm_provider}, "
                f"model={self.config.llm_model}, "
                f"tools={[t.name for t in self.tools]}"
            )
        except ImportError as e:
            self.logger.warning(f"LLM 依赖未安装: {e}")
        except Exception as e:
            self.logger.warning(f"任务规划器初始化失败: {e}")

    def update(self):
        command = self.blackboard.user_command

        if self.llm_with_tools is None:
            self.logger.warning("LLM 未就绪，回退为默认回复")
            self.blackboard.intent = "chat"
            self.blackboard.action_plan = []
            self.blackboard.response_text = "大模型未就绪，请稍后再试。"
            return Status.SUCCESS

        self.logger.info(f"规划指令: {command}")
        self._speak("好的，我先规划一下。")

        try:
            from langchain_core.messages import HumanMessage, SystemMessage

            messages = [
                SystemMessage(content=self.config.planner_system_prompt),
                *self.conversation_history,
                HumanMessage(content=command),
            ]

            response = self.llm_with_tools.invoke(messages)

            self.conversation_history.append(HumanMessage(content=command))
            self.conversation_history.append(response)

            max_msgs = self.config.llm_max_history * 2
            if len(self.conversation_history) > max_msgs:
                self.conversation_history = self.conversation_history[-max_msgs:]

            if response.tool_calls:
                plan = []
                has_exit = False
                for tc in response.tool_calls:
                    plan.append({
                        "name": tc["name"],
                        "args": tc.get("args", {}),
                    })
                    if tc["name"] == "exit_conversation":
                        has_exit = True

                self.blackboard.action_plan = plan
                self.blackboard.intent = "exit" if has_exit else "execute_plan"
                self.blackboard.response_text = ""

                plan_desc = ", ".join(
                    s["name"] + (f"({s['args']})" if s["args"] else "")
                    for s in plan
                )
                self.logger.info(f"规划结果: [{plan_desc}]")
            else:
                self.blackboard.action_plan = []
                self.blackboard.intent = "chat"
                self.blackboard.response_text = response.content

                self.logger.info("规划结果: 纯对话回复")

        except Exception as e:
            self.logger.error(f"LLM 规划失败: {e}")
            self.blackboard.action_plan = []
            self.blackboard.intent = "chat"
            self.blackboard.response_text = "抱歉，我暂时无法理解您的指令。"

        return Status.SUCCESS

    def _speak(self, text: str) -> None:
        speak_blocking(self._node, self._tts_client, text)
