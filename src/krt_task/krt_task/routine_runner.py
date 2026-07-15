"""Named routine action runner."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any

import rclpy
from krt_task.robot_db import RobotDatabase
from krt_task_interfaces.action import RunRoutine
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image


DEFAULT_TTS_SERVICE = "/voice/tts/synthesize"
DEFAULT_TTS_TIMEOUT_S = 60.0
DEFAULT_VISION_SERVICE = "/krt_human_robot/vision/describe_scene"
DEFAULT_PLAY_AUDIO_ACTION = "/voice/playback/play"
DEFAULT_ARM_ACTION = "/agx_action_group/run_action_group"
DEFAULT_HAND_ACTION_TEMPLATE = "/{side}/hand_control"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_IMAGE_DIR = "~/maps/routine_images"
DEFAULT_DESCRIBE_QUESTION = "请描述你看到的内容"
DESCRIBE_CAMERA_IDS = {"head", "left_palm", "right_palm"}
PARALLEL_TASKS = {"play_audio", "arm_group", "gripper", "wait"}


class RoutineCanceled(Exception):
    pass


@dataclass
class ParallelJob:
    name: str
    kind: str
    client: Any | None = None
    send_future: Future | None = None
    result_future: Future | None = None
    goal_handle: Any | None = None
    deadline: float | None = None
    success: bool | None = None


class RoutineRunnerNode(Node):
    """Runs named robot routines through ROS interfaces."""

    def __init__(self) -> None:
        super().__init__("routine_runner")
        self.declare_parameter("robot_db", "~/maps/krt_robot.db")
        self.declare_parameter("media_dir", "~/music")
        self.declare_parameter("tts_service", DEFAULT_TTS_SERVICE)
        self.declare_parameter("tts_timeout_s", DEFAULT_TTS_TIMEOUT_S)
        self.declare_parameter("vision_service", DEFAULT_VISION_SERVICE)
        self.declare_parameter("play_audio_action", DEFAULT_PLAY_AUDIO_ACTION)
        self.declare_parameter("arm_action", DEFAULT_ARM_ACTION)
        self.declare_parameter("hand_action_template", DEFAULT_HAND_ACTION_TEMPLATE)
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("image_dir", DEFAULT_IMAGE_DIR)
        self.declare_parameter("default_wait_ms", 200)
        self.database = RobotDatabase(str(self.get_parameter("robot_db").value))

        self._running = False
        self._lock = Lock()
        self._server = ActionServer(
            self,
            RunRoutine,
            "/krt_task/run_routine",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("routine_runner ready: action=/krt_task/run_routine")

    def _goal_callback(self, goal: RunRoutine.Goal) -> GoalResponse:
        if not goal.routine_name.strip():
            return GoalResponse.REJECT
        with self._lock:
            if self._running:
                return GoalResponse.REJECT
            self._running = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = RunRoutine.Result()
        name = goal_handle.request.routine_name.strip()
        try:
            task, args = self.load_routine(name)
            self.feedback(goal_handle, f"routine:{name}")
            ok = self.run_task(goal_handle, task, args)
            result.success = bool(ok)
            result.message = "routine completed" if ok else "routine failed"
            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except RoutineCanceled:
            result.success = False
            result.message = "routine canceled"
            goal_handle.canceled()
            return result
        except Exception as exc:  # pylint: disable=broad-except
            result.success = False
            result.message = str(exc)
            self.get_logger().error(f"routine failed: {exc}")
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._running = False

    def load_routine(self, name: str) -> tuple[str, dict[str, Any]]:
        spec = self.database.get_routine(name)
        task, args = parse_step(spec)
        if not task:
            raise ValueError(f"routine {name} 缺少 type")
        return task, args

    def run_task(self, goal_handle, task: str, args: dict[str, Any]) -> bool:
        self.check_cancel(goal_handle)
        self.feedback(goal_handle, task)
        if task == "wait":
            return self.wait_task(goal_handle, args)
        if task == "sequence":
            return self.run_sequence(goal_handle, args)
        if task == "parallel":
            return self.run_parallel(goal_handle, args)
        if task == "speak":
            return self.speak(goal_handle, args)
        if task == "describe":
            return self.describe(goal_handle, args)
        if task == "photo":
            return self.take_photo(goal_handle, args)
        if task == "play_audio":
            return self.wait_action_job(goal_handle, self.start_play_audio(args))
        if task == "arm_group":
            return self.wait_action_job(goal_handle, self.start_arm_group(args))
        if task == "gripper":
            return self.wait_action_job(goal_handle, self.start_gripper(args))
        raise ValueError(f"未知 routine task: {task}")

    def run_sequence(self, goal_handle, args: dict[str, Any]) -> bool:
        steps = args.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError("sequence.steps 必须是列表")
        for index, step in enumerate(steps, start=1):
            task, step_args = parse_step(step)
            if not task:
                raise ValueError(f"sequence 第 {index} 步缺少 type")
            self.feedback(goal_handle, f"sequence[{index}]:{task}")
            if not self.run_task(goal_handle, task, step_args):
                return False
        return True

    def run_parallel(self, goal_handle, args: dict[str, Any]) -> bool:
        if str(args.get("success_policy", "all")).strip() != "all":
            raise ValueError("parallel 当前仅支持 success_policy=all")
        steps = args.get("steps", [])
        if not isinstance(steps, list) or not steps:
            raise ValueError("parallel.steps 必须是非空列表")

        jobs: list[ParallelJob] = []
        try:
            for index, step in enumerate(steps, start=1):
                task, step_args = parse_step(step)
                if task not in PARALLEL_TASKS:
                    raise ValueError(
                        f"parallel 第 {index} 步不支持 {task!r}; "
                        f"可选: {', '.join(sorted(PARALLEL_TASKS))}"
                    )
                jobs.append(self.start_parallel_job(task, step_args, index))

            while rclpy.ok():
                self.check_cancel(goal_handle)
                for job in jobs:
                    self.poll_job(job)
                failed = [job for job in jobs if job.success is False]
                if failed:
                    self.cancel_jobs(jobs)
                    self.get_logger().error(f"parallel 子任务失败: {failed[0].name}")
                    return False
                if all(job.success is True for job in jobs):
                    return True
                time.sleep(0.05)
        except RoutineCanceled:
            self.cancel_jobs(jobs)
            raise

        self.cancel_jobs(jobs)
        return False

    def wait_task(self, goal_handle, args: dict[str, Any]) -> bool:
        deadline = time.monotonic() + max(
            0.0,
            int(args.get("wait_ms", int(self.get_parameter("default_wait_ms").value)))
            / 1000.0,
        )
        while time.monotonic() < deadline:
            self.check_cancel(goal_handle)
            time.sleep(0.05)
        return True

    def speak(self, goal_handle, args: dict[str, Any]) -> bool:
        from voice_interfaces.srv import SynthesizeSpeech

        text = str(args.get("text", "")).strip()
        if not text:
            raise ValueError("speak 缺少 text")
        client = self.create_client(
            SynthesizeSpeech, str(args.get("service", self.get_parameter("tts_service").value))
        )
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"TTS 服务不可用: {client.srv_name}")
        req = SynthesizeSpeech.Request()
        req.text = text
        req.language = str(args.get("language", "zh"))
        req.style = str(args.get("style", ""))
        req.priority = int(args.get("priority", 5))
        response = self.wait_future(
            goal_handle,
            client.call_async(req),
            float(args.get("timeout_s", self.get_parameter("tts_timeout_s").value)),
        )
        if response is None or not response.accepted:
            detail = getattr(response, "error_message", "") if response else ""
            self.get_logger().error(f"TTS 播报失败: {detail}")
            return False
        duration_s = max(0.0, float(response.estimated_duration_sec))
        # ponytail: estimated completion plus device drain; return the playback
        # action result from TTS if exact completion timing becomes necessary.
        return self.wait_task(
            goal_handle,
            {"wait_ms": int(round((duration_s + 0.1) * 1000))},
        )

    def describe(self, goal_handle, args: dict[str, Any]) -> bool:
        from voice_interfaces.srv import DescribeScene

        camera_id = str(args.get("camera_id", "head")).strip() or "head"
        if camera_id not in DESCRIBE_CAMERA_IDS:
            raise ValueError(f"describe camera_id 无效: {camera_id}")
        client = self.create_client(
            DescribeScene,
            str(args.get("service", self.get_parameter("vision_service").value)),
        )
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"视觉服务不可用: {client.srv_name}")
        req = DescribeScene.Request()
        req.camera_id = camera_id
        req.question = str(args.get("question", DEFAULT_DESCRIBE_QUESTION))
        response = self.wait_future(goal_handle, client.call_async(req), None)
        if response is None or not response.success:
            detail = getattr(response, "error_message", "") if response else ""
            self.get_logger().error(f"视觉理解失败: {detail}")
            return False
        text = str(response.description).strip()
        return bool(text) and self.speak(goal_handle, {"text": text})

    def take_photo(self, goal_handle, args: dict[str, Any]) -> bool:
        import cv2
        from cv_bridge import CvBridge

        image_topic = str(args.get("topic", self.get_parameter("image_topic").value))
        timeout_s = float(args.get("timeout_s", 5.0))
        image_dir = Path(str(args.get("dir", self.get_parameter("image_dir").value))).expanduser()
        image_dir.mkdir(parents=True, exist_ok=True)
        bridge = CvBridge()
        future: Future = Future()

        def callback(msg: Image) -> None:
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(Image, image_topic, callback, 10)
        msg = self.wait_future(goal_handle, future, timeout_s)
        self.destroy_subscription(sub)
        if msg is None:
            return False
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        filename = image_dir / f"routine_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        if not cv2.imwrite(str(filename), frame):
            self.get_logger().error(f"保存照片失败: {filename}")
            return False
        self.get_logger().info(f"已保存照片: {filename}")
        return True

    def start_parallel_job(
        self,
        task: str,
        args: dict[str, Any],
        index: int,
    ) -> ParallelJob:
        name = str(args.get("name", f"{task}_{index}"))
        if task == "wait":
            wait_ms = int(args.get("wait_ms", int(self.get_parameter("default_wait_ms").value)))
            return ParallelJob(
                name=name,
                kind="wait",
                deadline=time.monotonic() + max(0.0, wait_ms / 1000.0),
            )
        if task == "play_audio":
            job = self.start_play_audio(args)
        elif task == "arm_group":
            job = self.start_arm_group(args)
        elif task == "gripper":
            job = self.start_gripper(args)
        else:
            raise ValueError(f"parallel 不支持任务: {task}")
        job.name = name
        return job

    def start_play_audio(self, args: dict[str, Any]) -> ParallelJob:
        from voice_interfaces.action import PlayAudio

        media = self.database.get_media(str(args.get("media_key", "")))
        media_dir = Path(str(self.get_parameter("media_dir").value)).expanduser().resolve()
        path = (media_dir / media["filename"]).resolve()
        if media_dir not in path.parents or not path.is_file():
            raise FileNotFoundError(f"媒体文件不存在: {path}")
        goal = PlayAudio.Goal()
        goal.file_path = str(path)
        goal.priority = int(args.get("priority", 5))
        goal.preempt_lower_priority = bool(args.get("preempt_lower_priority", True))
        return self.start_action_job(
            "play_audio",
            PlayAudio,
            str(args.get("action", self.get_parameter("play_audio_action").value)),
            goal,
            float(args.get("server_timeout_s", 2.0)),
            optional_deadline(args),
        )

    def start_arm_group(self, args: dict[str, Any]) -> ParallelJob:
        from agx_action_group_interfaces.action import RunActionGroup

        group_name = str(args.get("group_name", "")).strip()
        if not group_name:
            raise ValueError("arm_group 缺少 group_name")
        goal = RunActionGroup.Goal()
        goal.group_name = group_name
        goal.repeat_count = int(args.get("repeat_count", 1))
        goal.arm_target = str(args.get("arm_target", "left"))
        return self.start_action_job(
            f"arm_group:{group_name}",
            RunActionGroup,
            str(args.get("action", self.get_parameter("arm_action").value)),
            goal,
            float(args.get("server_timeout_s", 2.0)),
            optional_deadline(args),
        )

    def start_gripper(self, args: dict[str, Any]) -> ParallelJob:
        from hands_control_interfaces.action import HandControl

        side = str(args.get("side", args.get("hand", ""))).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError("gripper 缺少 side=left/right")
        if "position" not in args:
            raise ValueError("gripper 缺少 position")
        goal = HandControl.Goal()
        goal.adapter_index = int(args.get("adapter_index", 0 if side == "left" else 1))
        goal.finger_id = int(args.get("finger_id", 0))
        goal.position = int(args["position"])
        goal.speed = int(args.get("speed", 500))
        goal.force = int(args.get("force", 85))
        goal.wait_time = int(args.get("wait_time", 10))
        action = str(
            args.get(
                "action",
                str(self.get_parameter("hand_action_template").value).format(side=side),
            )
        )
        return self.start_action_job(
            f"gripper:{side}",
            HandControl,
            action,
            goal,
            float(args.get("server_timeout_s", 2.0)),
            optional_deadline(args),
        )

    def start_action_job(
        self,
        label: str,
        action_type,
        action_name: str,
        goal,
        server_timeout_s: float,
        deadline: float | None,
    ) -> ParallelJob:
        client = ActionClient(self, action_type, action_name)
        if not client.wait_for_server(timeout_sec=server_timeout_s):
            raise RuntimeError(f"action 不可用: {action_name}")
        return ParallelJob(
            name=label,
            kind="action",
            client=client,
            send_future=client.send_goal_async(goal),
            deadline=deadline,
        )

    def wait_action_job(self, goal_handle, job: ParallelJob) -> bool:
        try:
            while rclpy.ok():
                self.check_cancel(goal_handle)
                self.poll_job(job)
                if job.success is not None:
                    if job.success is False:
                        self.cancel_jobs([job])
                    return job.success
                time.sleep(0.05)
        except RoutineCanceled:
            self.cancel_jobs([job])
            raise
        return False

    def poll_job(self, job: ParallelJob) -> None:
        if job.success is not None:
            return
        if job.kind == "wait":
            if job.deadline is None or time.monotonic() >= job.deadline:
                job.success = True
            return
        if job.deadline is not None and time.monotonic() > job.deadline:
            self.get_logger().error(f"{job.name} 执行超时")
            job.success = False
            return
        if job.result_future is not None:
            if not job.result_future.done():
                return
            result_msg = job.result_future.result()
            result = getattr(result_msg, "result", None)
            job.success = bool(result is not None and getattr(result, "success", False))
            if not job.success:
                detail = (
                    getattr(result, "message", "")
                    or getattr(result, "error_message", "")
                    or "unknown"
                )
                self.get_logger().error(f"{job.name} 执行失败: {detail}")
            return
        if job.send_future is None or not job.send_future.done():
            return
        job.goal_handle = job.send_future.result()
        if job.goal_handle is None or not job.goal_handle.accepted:
            self.get_logger().error(f"{job.name} 目标被拒绝")
            job.success = False
            return
        job.result_future = job.goal_handle.get_result_async()

    def cancel_jobs(self, jobs: list[ParallelJob]) -> None:
        for job in jobs:
            if job.goal_handle is None:
                continue
            if job.result_future is not None and job.result_future.done():
                continue
            try:
                job.goal_handle.cancel_goal_async()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warning(f"取消 {job.name} 失败: {exc}")

    def wait_future(self, goal_handle, future: Future, timeout: float | None) -> Any:
        deadline = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            self.check_cancel(goal_handle)
            if deadline is not None and time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result() if future.done() else None

    def feedback(self, goal_handle, current_step: str) -> None:
        msg = RunRoutine.Feedback()
        msg.current_step = current_step
        goal_handle.publish_feedback(msg)

    @staticmethod
    def check_cancel(goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise RoutineCanceled()


def parse_step(step: Any) -> tuple[str, dict[str, Any]]:
    if not isinstance(step, dict):
        return "", {}
    task = str(step.get("type", "")).strip()
    nested = step.get("args", {})
    args = dict(nested) if isinstance(nested, dict) else {}
    args.update({key: value for key, value in step.items() if key not in {"type", "args"}})
    return task, args


def optional_deadline(args: dict[str, Any]) -> float | None:
    timeout = float(args.get("timeout_s", 0.0))
    return time.monotonic() + timeout if timeout > 0.0 else None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoutineRunnerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
