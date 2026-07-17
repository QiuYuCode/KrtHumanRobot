from types import SimpleNamespace

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


def make_controller(states, popen=lambda *_args, **_kwargs: None):
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
