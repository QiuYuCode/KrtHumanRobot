import threading
from types import SimpleNamespace

import pytest
from lifecycle_msgs.msg import State, Transition

from krt_human_robot.gripper_system import GripperSystemController


class ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result


class FakeLifecycleClient:
    def __init__(self, states, side, kind):
        self.states = states
        self.side = side
        self.kind = kind
        self.transitions = []

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.side in self.states

    def call_async(self, request):
        if self.kind == "state":
            label = self.states[self.side]
            state_id = {
                "unconfigured": State.PRIMARY_STATE_UNCONFIGURED,
                "inactive": State.PRIMARY_STATE_INACTIVE,
                "active": State.PRIMARY_STATE_ACTIVE,
            }[label]
            return ImmediateFuture(SimpleNamespace(
                current_state=SimpleNamespace(id=state_id, label=label)
            ))
        transition = request.transition.id
        self.transitions.append(transition)
        self.states[self.side] = {
            Transition.TRANSITION_CONFIGURE: "inactive",
            Transition.TRANSITION_ACTIVATE: "active",
            Transition.TRANSITION_DEACTIVATE: "inactive",
            Transition.TRANSITION_CLEANUP: "unconfigured",
        }[transition]
        return ImmediateFuture(SimpleNamespace(success=True))


class FakeParameterClient:
    def __init__(self, states, side, kind):
        self.states = states
        self.side = side
        self.kind = kind
        self.values = {}

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.side in self.states

    def call_async(self, request):
        if self.kind == "set":
            self.values.update({item.name: item.value for item in request.parameters})
            return ImmediateFuture(SimpleNamespace(results=[
                SimpleNamespace(successful=True, reason="") for _ in request.parameters
            ]))
        values = []
        for name in request.names:
            value = self.values.get(name)
            values.append(value or SimpleNamespace(
                string_value="ZLG_MINI", integer_value=0, bool_value=False
            ))
        return ImmediateFuture(SimpleNamespace(values=values))


class FakeNode:
    def __init__(self, states):
        self.states = states
        self.clients = {}

    def create_client(self, _service_type, name, **_kwargs):
        side = name.split("/")[1]
        if name.endswith("/get_state"):
            kind = "state"
            client = FakeLifecycleClient(self.states, side, kind)
        elif name.endswith("/change_state"):
            kind = "change"
            client = FakeLifecycleClient(self.states, side, kind)
        elif name.endswith("/get_parameters"):
            kind = "get"
            client = FakeParameterClient(self.states, side, kind)
        else:
            kind = "set"
            client = FakeParameterClient(self.states, side, kind)
        self.clients[(side, kind)] = client
        return client


class FakeDatabase:
    def __init__(self):
        self.settings = {
            side: {
                "side": side, "adapter_type": "ZLG_MINI",
                "adapter_index": index, "device_id": index + 1,
                "listen_enabled": False, "realtime_response_enabled": False,
            }
            for index, side in enumerate(("left", "right"))
        }

    def list_gripper_settings(self):
        return list(self.settings.values())

    def get_gripper_settings(self, side):
        return dict(self.settings[side])

    def update_gripper_settings(self, side, changes):
        self.settings[side].update(changes)


def make_controller(
        states, popen=lambda *_args, **_kwargs: None, settings_updated=None):
    node = FakeNode(states)
    controller = GripperSystemController(
        node,
        FakeDatabase(),
        hand_clients={
            side: SimpleNamespace(wait_for_server=lambda timeout_sec: timeout_sec >= 0)
            for side in ("left", "right")
        },
        future_result=lambda future, _timeout: future.result(),
        popen=popen,
        sleep=lambda _seconds: None,
        startup_timeout=0.01,
        settings_updated=settings_updated,
    )
    return controller, node


def test_existing_active_node_is_adopted_without_duplicate_launch():
    launches = []
    controller, _node = make_controller(
        {"left": "active", "right": "active"},
        popen=lambda *args, **kwargs: launches.append((args, kwargs)),
    )

    result = controller.control("both", True)

    assert result["success"] is True
    assert launches == []
    assert result["hands"]["left"]["state"] == "active"


def test_missing_side_is_launched_then_configured_and_activated():
    states = {"left": "active"}
    launches = []

    def popen(command, **kwargs):
        launches.append((command, kwargs))
        states["right"] = "unconfigured"
        return SimpleNamespace(poll=lambda: None, terminate=lambda: None)

    controller, node = make_controller(states, popen=popen)

    result = controller.control("right", True)

    assert result["success"] is True
    assert "enable_left:=false" in launches[0][0]
    assert "enable_right:=true" in launches[0][0]
    assert "right_hand_adapter_index:=1" in launches[0][0]
    assert "right_hand_device_id:=2" in launches[0][0]
    transitions = node.clients[("right", "change")].transitions
    assert transitions == [
        Transition.TRANSITION_CONFIGURE,
        Transition.TRANSITION_ACTIVATE,
    ]


def test_stop_active_hand_deactivates_and_cleans_up():
    controller, node = make_controller({"left": "active"})

    result = controller.control("left", False)

    assert result["success"] is True
    assert node.clients[("left", "change")].transitions == [
        Transition.TRANSITION_DEACTIVATE,
        Transition.TRANSITION_CLEANUP,
    ]
    assert result["hands"]["left"]["state"] == "unconfigured"


def test_hardware_settings_restart_only_the_owned_hand():
    states = {"left": "unconfigured", "right": "active"}
    launched = []

    def popen(command, **_kwargs):
        side = "left" if "enable_left:=true" in command else "right"
        launched.append(side)
        states[side] = "unconfigured"
        return SimpleNamespace(pid=200 + len(launched), poll=lambda: None)

    controller, node = make_controller(states, popen=popen)
    old_process = SimpleNamespace(pid=100, poll=lambda: None)
    controller.processes = {"left": old_process}
    stopped = []

    def terminate(process):
        stopped.append(process.pid)
        states.pop("left")

    controller._terminate = terminate

    result = controller.update_settings("left", {"adapter_index": 4})

    assert stopped == [100]
    assert launched == ["left"]
    assert states == {"left": "active", "right": "active"}
    assert result["settings"]["adapter_index"] == 4
    assert result["restart"]["state"] == "active"
    assert node.clients[("left", "set")].values[
        "adapter_index"
    ].integer_value == 4


def test_hardware_settings_reject_unowned_existing_node_before_saving():
    controller, _node = make_controller({"left": "unconfigured"})

    with pytest.raises(RuntimeError, match="不由 Web 管理"):
        controller.update_settings("left", {"adapter_index": 4})

    assert controller.database.get_gripper_settings("left")["adapter_index"] == 0


def test_hardware_settings_sync_bridge_before_restart_can_fail():
    synced = []
    controller, _node = make_controller(
        {}, settings_updated=lambda side, settings: synced.append(
            (side, settings["adapter_index"])
        )
    )
    controller._restart_side = lambda _side: (
        (_ for _ in ()).throw(RuntimeError("start failed"))
    )

    with pytest.raises(RuntimeError, match="start failed"):
        controller.update_settings("left", {"adapter_index": 4})

    assert synced == [("left", 4)]


def test_runtime_update_waits_for_hardware_restart_lock():
    entered_restart = threading.Event()
    release_restart = threading.Event()
    runtime_finished = threading.Event()
    controller, _node = make_controller({"left": "unconfigured"})
    controller.processes = {
        "left": SimpleNamespace(pid=100, poll=lambda: None)
    }

    def restart(_side):
        entered_restart.set()
        release_restart.wait(1.0)
        return {"success": True, "state": "active", "message": "已重启"}

    controller._restart_side = restart
    settings_thread = threading.Thread(
        target=controller.update_settings,
        args=("left", {"adapter_index": 4}),
    )
    runtime_thread = threading.Thread(
        target=lambda: (
            controller.update_runtime("left", {"listen_enabled": True}),
            runtime_finished.set(),
        )
    )

    settings_thread.start()
    assert entered_restart.wait(1.0)
    runtime_thread.start()
    assert not runtime_finished.wait(0.05)
    release_restart.set()
    settings_thread.join(1.0)
    runtime_thread.join(1.0)

    assert runtime_finished.is_set()
    assert controller.database.get_gripper_settings("left")["listen_enabled"] is True


def test_start_both_reports_each_hand_when_second_launch_fails():
    states = {}

    def popen(command, **_kwargs):
        if "enable_left:=true" in command:
            states["left"] = "unconfigured"
        return SimpleNamespace(pid=100, poll=lambda: None, terminate=lambda: None)

    controller, _node = make_controller(states, popen=popen)

    result = controller.control("both", True)

    assert result["success"] is False
    assert result["hands"]["left"]["success"] is True
    assert result["hands"]["right"]["success"] is False


def test_empty_hardware_settings_do_not_restart():
    controller, _node = make_controller({"left": "unconfigured"})
    controller.processes = {
        "left": SimpleNamespace(pid=100, poll=lambda: None)
    }
    controller._restart_side = lambda _side: (
        (_ for _ in ()).throw(AssertionError("must not restart"))
    )

    result = controller.update_settings("left", {})

    assert result["settings"]["adapter_index"] == 0
    assert result["restart"]["message"] == "参数未修改"


def test_status_keeps_other_hand_when_one_state_query_fails():
    controller, _node = make_controller({"left": "active", "right": "active"})
    original_state = controller._state
    controller._state = lambda side: (
        (_ for _ in ()).throw(RuntimeError("left timeout"))
        if side == "left" else original_state(side)
    )

    status = controller.status()

    assert status["hands"]["left"]["lifecycle_state"] == "unknown"
    assert status["hands"]["left"]["error"] == "left timeout"
    assert status["hands"]["right"]["lifecycle_state"] == "active"


def test_control_keeps_transition_error_when_followup_state_query_times_out():
    controller, _node = make_controller({"left": "unconfigured"})
    state_results = iter(["unconfigured", TimeoutError("state timeout")])

    def state(_side):
        result = next(state_results)
        if isinstance(result, Exception):
            raise result
        return result

    controller._state = state
    controller._start_side = lambda _side: (
        (_ for _ in ()).throw(TimeoutError("configure timeout"))
    )

    result = controller.control("left", True)

    assert result["success"] is False
    assert result["hands"]["left"] == {
        "success": False,
        "state": "unknown",
        "message": "configure timeout",
    }
