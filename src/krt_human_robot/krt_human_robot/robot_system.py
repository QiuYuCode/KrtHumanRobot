"""Lifecycle control and dependency discovery for Web-managed robot systems."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable

from agx_action_group_interfaces.srv import StartTeach, StopTeach
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient
from voice_interfaces.action import PlayAudio
from voice_interfaces.srv import DescribeScene, SynthesizeSpeech


def provider_commands(requirements: set[str], config_file: str) -> dict[str, list[str]]:
    """Build the smallest existing launch commands for peripheral resources."""
    commands = {}
    if requirements & {"tts", "playback"}:
        commands["voice"] = [
            "ros2", "launch", "voice_assistant", "voice_stack.launch.py",
            "enable_capture:=false", "enable_kws:=false", "enable_vad:=false",
            "enable_asr:=false",
            f"enable_tts:={'true' if 'tts' in requirements else 'false'}",
            "enable_playback:=true", "enable_media:=false", "enable_volume:=false",
        ]
    if "camera:head" in requirements:
        commands["camera:head"] = [
            "ros2", "launch", "realsense2_camera", "rs_launch.py",
            "enable_color:=true", "enable_depth:=false",
        ]
    if requirements & {"camera:left_palm", "camera:right_palm"}:
        camera_command = [
            "ros2", "launch", "hand_camera_driver", "hand_cameras.launch.py",
            f"enable_left:={'true' if 'camera:left_palm' in requirements else 'false'}",
            f"enable_right:={'true' if 'camera:right_palm' in requirements else 'false'}",
        ]
        commands["camera:left_palm"] = camera_command
        commands["camera:right_palm"] = camera_command
    if "vision" in requirements:
        commands["vision"] = [
            "ros2", "launch", "krt_human_robot", "vision_service.launch.py",
            f"config_file:={config_file}",
        ]
    return commands


def collect_routine_requirements(spec: dict[str, Any]) -> set[str]:
    """Return managed resources referenced by a routine tree."""
    requirements = {"routine"}

    def visit(step: Any) -> None:
        if not isinstance(step, dict):
            return
        kind = str(step.get("type", ""))
        if kind in {"sequence", "parallel"}:
            for child in step.get("steps", []):
                visit(child)
        elif kind == "arm_group":
            requirements.add("action_group_stack")
            target = str(step.get("arm_target", ""))
            if target in {"left", "both"}:
                requirements.add("left_arm")
            if target in {"right", "both"}:
                requirements.add("right_arm")
        elif kind == "gripper":
            requirements.add("grippers")
        elif kind == "speak":
            requirements.update({"tts", "playback"})
        elif kind == "play_audio":
            requirements.add("playback")
        elif kind == "describe":
            camera_id = str(step.get("camera_id", "head")) or "head"
            requirements.update({"vision", f"camera:{camera_id}"})
        elif kind == "photo":
            topic = str(step.get("topic", "")).strip()
            requirements.add(f"topic:{topic}" if topic else "camera:head")

    visit(spec)
    return requirements


class LifecycleEndpoint:
    def __init__(self, node, name: str, future_result: Callable[[Any, float], Any]):
        self.name = name
        self._future_result = future_result
        self.state_client = node.create_client(GetState, f"{name}/get_state")
        self.change_client = node.create_client(ChangeState, f"{name}/change_state")

    def state(self) -> str:
        if not self.state_client.wait_for_service(timeout_sec=0.05):
            return "missing"
        response = self._future_result(
            self.state_client.call_async(GetState.Request()), 0.8
        )
        return str(response.current_state.label).lower()

    def transition(self, transition_id: int) -> None:
        if not self.change_client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"生命周期服务不可用: {self.name}")
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._future_result(
            self.change_client.call_async(request), 10.0
        )
        if not response.success:
            raise RuntimeError(f"生命周期转换失败: {self.name} ({transition_id})")

    def start(self) -> str:
        state = self.state()
        if state == "unconfigured":
            self.transition(Transition.TRANSITION_CONFIGURE)
            state = self.state()
        if state == "inactive":
            self.transition(Transition.TRANSITION_ACTIVATE)
            state = self.state()
        if state != "active":
            raise RuntimeError(f"{self.name} 无法从 {state} 启动")
        return state

    def stop(self) -> str:
        state = self.state()
        if state == "active":
            self.transition(Transition.TRANSITION_DEACTIVATE)
            state = self.state()
        if state == "inactive":
            self.transition(Transition.TRANSITION_CLEANUP)
            state = self.state()
        if state not in {"missing", "unconfigured"}:
            raise RuntimeError(f"{self.name} 无法从 {state} 关闭")
        return state


class RobotSystemController:
    """Launch missing nodes and drive their lifecycle transitions."""

    def __init__(
        self,
        node,
        robot_config,
        *,
        future_result: Callable[[Any, float], Any],
        robot_db: str,
        media_dir: str,
        config_file: str = "",
        popen: Callable[..., subprocess.Popen] = subprocess.Popen,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._popen = popen
        self._sleep = sleep
        self.node = node
        self.lock = threading.Lock()
        self.processes = []
        self.endpoints = {
            "left_arm": [LifecycleEndpoint(
                node, "/left/agx_arm_ctrl_single_node", future_result
            )],
            "right_arm": [LifecycleEndpoint(
                node, "/right/agx_arm_ctrl_single_node", future_result
            )],
            "action_group_stack": [
                LifecycleEndpoint(
                    node, "/agx_action_group/agx_action_group_runner", future_result
                ),
                LifecycleEndpoint(
                    node, "/agx_action_group/teach_action_group", future_result
                ),
            ],
            "routine": [LifecycleEndpoint(node, "/routine_runner", future_result)],
        }
        channels = getattr(robot_config, "robot_arm_channels", {})
        arm_type = str(getattr(robot_config, "robot_arm_robot", "nero"))
        speed = str(getattr(robot_config, "robot_arm_replay_speed_percent", 30))
        self.commands = {
            side + "_arm": [
                "ros2", "launch", "agx_arm_ctrl", "start_single_agx_arm.launch.py",
                f"namespace:={side}",
                f"can_port:={channels.get(side, 'can_' + side)}",
                f"arm_type:={arm_type}", f"speed_percent:={speed}",
                "auto_enable:=true", "autostart:=false",
            ]
            for side in ("left", "right")
        }
        self.commands["action_group_stack"] = [
            "ros2", "launch", "agx_action_group_runner", "teach_action_group.launch.py",
            f"robot_db:={robot_db}", "left_namespace:=/left",
            "right_namespace:=/right", "autostart:=false",
        ]
        self.commands["routine"] = [
            "ros2", "launch", "krt_task", "routine_runner.launch.py",
            f"robot_db:={robot_db}", f"media_dir:={media_dir}",
            "autostart:=false",
        ]
        self.start_teach_client = node.create_client(
            StartTeach, "/agx_action_group/start_teach"
        )
        self.stop_teach_client = node.create_client(
            StopTeach, "/agx_action_group/stop_teach"
        )
        self._future_result = future_result
        self.provider_processes = []
        self.provider_commands = provider_commands(set(), config_file)
        self._config_file = config_file
        self._camera_topics = {
            name: str(values.get("ros_topic", ""))
            for name, values in getattr(robot_config, "cameras", {}).items()
        }
        self.tts_client = node.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
        )
        self.playback_client = ActionClient(
            node, PlayAudio, "/voice/playback/play"
        )
        self.vision_client = node.create_client(
            DescribeScene, "/krt_human_robot/vision/describe_scene"
        )
        self.teaching = {"active": False, "arm_target": "", "group_name": ""}

    def _topic_ready(self, topic: str) -> bool:
        return bool(topic and self.node.get_publishers_info_by_topic(topic))

    def _provider_ready(self, requirement: str) -> bool:
        if requirement == "tts":
            return self.tts_client.wait_for_service(timeout_sec=0.0)
        if requirement == "playback":
            return self.playback_client.wait_for_server(timeout_sec=0.0)
        if requirement == "vision":
            return self.vision_client.wait_for_service(timeout_sec=0.0)
        if requirement.startswith("camera:"):
            return self._topic_ready(
                self._camera_topics.get(requirement.split(":", 1)[1], "")
            )
        if requirement.startswith("topic:"):
            return self._topic_ready(requirement.split(":", 1)[1])
        return True

    def ensure_providers(self, requirements: set[str]) -> None:
        provider_requirements = {
            item for item in requirements
            if item in {"tts", "playback", "vision"}
            or item.startswith(("camera:", "topic:"))
        }
        custom_missing = [
            item for item in provider_requirements
            if item.startswith("topic:") and not self._provider_ready(item)
        ]
        if custom_missing:
            raise RuntimeError(f"自定义话题不可用: {custom_missing[0][6:]}")
        missing = {
            item for item in provider_requirements if not self._provider_ready(item)
        }
        commands = provider_commands(missing, self._config_file)
        launched = set()
        for key in ("voice", "camera:head", "camera:left_palm",
                    "camera:right_palm", "vision"):
            command = commands.get(key)
            marker = tuple(command or [])
            if not command or marker in launched:
                continue
            launched.add(marker)
            self.provider_processes.append(
                self._popen(command, start_new_session=True)
            )
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            pending = [
                item for item in provider_requirements
                if not self._provider_ready(item)
            ]
            if not pending:
                return
            self._sleep(0.1)
        raise RuntimeError(f"资源启动超时: {', '.join(sorted(pending))}")

    def shutdown_owned_providers(self) -> None:
        """Stop only peripheral launch processes started by this Web process."""
        for process in reversed(self.provider_processes):
            if process.poll() is not None:
                continue
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        self.provider_processes.clear()

    def _states(self, component: str) -> list[str]:
        return [endpoint.state() for endpoint in self.endpoints[component]]

    def _launch_missing(self, component: str) -> None:
        process = self._popen(self.commands[component], start_new_session=True)
        self.processes.append(process)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if all(state != "missing" for state in self._states(component)):
                return
            if process.poll() is not None:
                break
            self._sleep(0.1)
        raise RuntimeError(f"{component} 节点启动超时")

    def control(self, component: str, enabled: bool) -> dict[str, Any]:
        selected = ["left_arm", "right_arm"] if component == "arms" else [component]
        if any(item not in self.endpoints for item in selected):
            raise ValueError("未知机器人系统组件")
        if not enabled and self.teaching["active"]:
            teaching_component = f"{self.teaching['arm_target']}_arm"
            if component in {"arms", "action_group_stack", teaching_component}:
                self.stop_teach()
        with self.lock:
            results = {}
            for item in selected:
                try:
                    if enabled and all(state == "missing" for state in self._states(item)):
                        self._launch_missing(item)
                    endpoints = self.endpoints[item]
                    ordered = endpoints if enabled else list(reversed(endpoints))
                    states = [endpoint.start() if enabled else endpoint.stop()
                              for endpoint in ordered]
                    results[item] = {"success": True, "states": states, "message": ""}
                except Exception as exc:
                    results[item] = {
                        "success": False, "states": self._states(item),
                        "message": str(exc),
                    }
            return {
                "success": all(result["success"] for result in results.values()),
                "components": results,
            }

    def status(self) -> dict[str, Any]:
        components = {}
        for name in self.endpoints:
            try:
                states = self._states(name)
                components[name] = {
                    "states": states,
                    "present": any(state != "missing" for state in states),
                    "active": bool(states) and all(state == "active" for state in states),
                    "error": "",
                }
            except Exception as exc:
                components[name] = {
                    "states": ["unknown"], "present": True,
                    "active": False, "error": str(exc),
                }
        providers = {
            name: self._provider_ready(name)
            for name in ("tts", "playback", "vision", "camera:head",
                         "camera:left_palm", "camera:right_palm")
        }
        return {
            "components": components, "teaching": dict(self.teaching),
            "providers": providers,
        }

    def start_teach(self, arm_target: str, group_name: str) -> dict[str, Any]:
        if arm_target not in {"left", "right"}:
            raise ValueError("arm_target 必须是 left 或 right")
        for component in (f"{arm_target}_arm", "action_group_stack"):
            result = self.control(component, True)
            if not result["success"]:
                detail = result["components"][component]["message"]
                raise RuntimeError(f"{component} 启动失败: {detail}")
        request = StartTeach.Request()
        request.arm_target = arm_target
        request.group_name = group_name.strip()
        if not self.start_teach_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("示教启动服务不可用")
        response = self._future_result(
            self.start_teach_client.call_async(request), 8.0
        )
        if not response.success:
            raise RuntimeError(response.message)
        self.teaching = {
            "active": True, "arm_target": arm_target,
            "group_name": request.group_name,
        }
        return dict(self.teaching)

    def stop_teach(self, group_name: str = "") -> dict[str, Any]:
        request = StopTeach.Request()
        request.arm_target = str(self.teaching.get("arm_target", ""))
        request.group_name = group_name.strip()
        if not self.stop_teach_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("示教停止服务不可用")
        response = self._future_result(
            self.stop_teach_client.call_async(request), 8.0
        )
        if not response.success:
            raise RuntimeError(response.message)
        self.teaching = {"active": False, "arm_target": "", "group_name": ""}
        return {
            "active": False, "group_name": response.group_name,
            "sample_count": int(response.sample_count),
        }
