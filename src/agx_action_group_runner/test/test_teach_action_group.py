import threading
from types import SimpleNamespace

from agx_action_group_runner.teach_action_group_node import TeachActionGroupNode


class Recorder:
    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True
        return [object()]


def test_stop_without_group_saves_unnamed_action_group():
    node = object.__new__(TeachActionGroupNode)
    recorder = Recorder()
    node.lock = threading.Lock()
    node.recorders = {"left": recorder}
    node.active_arm = "left"
    node.active_group = None
    node.database = SimpleNamespace(path="/tmp/robot.db")
    node._call_teach_mode = lambda _, enabled: (not enabled, "ok")
    saved = []
    node._write_group = lambda *args: saved.append(args)

    response = node._stop_cb(
        SimpleNamespace(arm_target="left", group_name=""), SimpleNamespace()
    )

    assert response.success
    assert response.sample_count == 1
    assert response.group_name.startswith("未命名-左臂-")
    assert recorder.stopped
    assert node.active_arm is None
    assert saved[0][0] == response.group_name
