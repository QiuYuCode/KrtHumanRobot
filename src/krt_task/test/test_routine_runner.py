from types import SimpleNamespace

import pytest

from krt_task.routine_runner import ParallelJob, RoutineCanceled, RoutineRunnerNode


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
