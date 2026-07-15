from types import SimpleNamespace

from krt_task.routine_runner import RoutineRunnerNode


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
