from threading import Lock
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CompressedImage
from rclpy.action import GoalResponse

from krt_task.routine_runner import ParallelJob, RoutineCanceled, RoutineRunnerNode


def test_busy_runner_accepts_goal_for_explicit_busy_result():
    runner = SimpleNamespace(_active=True, _running=True, _lock=Lock())
    goal = SimpleNamespace(routine_name="第二个流程")

    assert RoutineRunnerNode._goal_callback(runner, goal) == GoalResponse.ACCEPT


class FakeClient:
    srv_name = "/voice/tts/synthesize"

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0

    def call_async(self, request):
        return request


class FakeRunner:
    def __init__(self):
        self.wait_calls = []

    def create_client(self, *_args):
        return FakeClient()

    def get_parameter(self, _name):
        return SimpleNamespace(value=None)

    def wait_future(self, _goal_handle, _future, _timeout):
        return SimpleNamespace(accepted=True, estimated_duration_sec=1.53)

    def wait_task(self, goal_handle, args):
        self.wait_calls.append((goal_handle, args))
        return True


def test_speak_waits_for_estimated_playback_before_next_step():
    runner = FakeRunner()
    goal_handle = object()

    assert RoutineRunnerNode.speak(runner, goal_handle, {
        "text": "开始播放",
        "service": "/voice/tts/synthesize",
        "timeout_s": 2.0,
    })
    assert runner.wait_calls == [(goal_handle, {"wait_ms": 1630})]


class FakeGripperRunner:
    def __init__(self):
        self.database = SimpleNamespace(get_gripper_action=lambda _name: {
            "targets": [
                {"side": "left", "finger_id": 0, "position": 0,
                 "speed": 240, "force": 85, "wait_time": 10},
                {"side": "right", "finger_id": 1, "position": 500,
                 "speed": 300, "force": 90, "wait_time": 12},
            ],
        })
        self.jobs = []

    def get_parameter(self, name):
        return SimpleNamespace(value={
            "hand_action_template": "/{side}/hand_control",
            "left_hand_adapter_index": 4,
            "right_hand_adapter_index": 7,
        }[name])

    def start_action_job(self, label, _action_type, action_name, goal,
                         server_timeout_s, deadline):
        job = SimpleNamespace(
            name=label, action_name=action_name, goal=goal,
            server_timeout_s=server_timeout_s, deadline=deadline,
        )
        self.jobs.append(job)
        return job

    def start_gripper(self, args):
        return RoutineRunnerNode.start_gripper(self, args)


def test_named_gripper_action_starts_one_job_per_hand():
    runner = FakeGripperRunner()

    jobs = RoutineRunnerNode.start_gripper_jobs(
        runner, {"action_name": "双手动作"}
    )

    assert jobs == runner.jobs
    assert [job.action_name for job in jobs] == [
        "/left/hand_control", "/right/hand_control",
    ]
    assert [job.goal.adapter_index for job in jobs] == [4, 7]
    assert jobs[1].goal.finger_id == 1


class FakeJobWaiter:
    def __init__(self, cancel=False):
        self.cancel = cancel
        self.canceled = []

    def check_cancel(self, _goal_handle):
        if self.cancel:
            raise RoutineCanceled()

    def poll_job(self, _job):
        pass

    def cancel_jobs(self, jobs):
        self.canceled = jobs


def test_wait_jobs_cancels_sibling_after_partial_failure(monkeypatch):
    monkeypatch.setattr("krt_task.routine_runner.rclpy.ok", lambda: True)
    runner = FakeJobWaiter()
    jobs = [ParallelJob("left", "action", success=False),
            ParallelJob("right", "action", success=None)]

    assert not RoutineRunnerNode.wait_jobs(runner, object(), jobs)
    assert runner.canceled == jobs


def test_wait_jobs_cancels_all_on_routine_cancel(monkeypatch):
    monkeypatch.setattr("krt_task.routine_runner.rclpy.ok", lambda: True)
    runner = FakeJobWaiter(cancel=True)
    jobs = [ParallelJob("left", "action")]

    with pytest.raises(RoutineCanceled):
        RoutineRunnerNode.wait_jobs(runner, object(), jobs)
    assert runner.canceled == jobs


class FakePhotoRunner:
    """Minimal routine runner surface for a compressed photo task."""

    def __init__(self, message):
        self.message = message
        self.subscription = object()
        self.subscription_args = None
        self.destroyed = []
        self.logged = []

    def get_parameter(self, name):
        return SimpleNamespace(value={
            "image_topic": "/camera/camera/color/image_raw",
            "image_transport": "compressed",
            "image_dir": "/unused",
        }[name])

    def create_subscription(self, message_type, topic, callback, _qos):
        self.subscription_args = (message_type, topic)
        callback(self.message)
        return self.subscription

    def wait_future(self, _goal_handle, future, _timeout):
        return future.result()

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def get_logger(self):
        return SimpleNamespace(
            info=self.logged.append,
            error=self.logged.append,
        )


def test_take_photo_saves_compressed_message(tmp_path, monkeypatch):
    """Routine photos subscribe to the compressed suffix and save decoded JPEG."""
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[:, :, 1] = 160
    ok, encoded = cv2.imencode(".jpg", frame)
    assert ok
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    runner = FakePhotoRunner(msg)
    saved = []
    monkeypatch.setattr(cv2, "imwrite", lambda path, image: saved.append(
        (path, image.copy())
    ) or True)

    assert RoutineRunnerNode.take_photo(
        runner, object(), {"dir": str(tmp_path)}
    )

    assert runner.subscription_args == (
        CompressedImage,
        "/camera/camera/color/image_raw/compressed",
    )
    assert runner.destroyed == [runner.subscription]
    assert saved[0][1].shape == frame.shape
