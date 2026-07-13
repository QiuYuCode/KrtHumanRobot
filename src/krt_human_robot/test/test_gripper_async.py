from types import SimpleNamespace

from krt_human_robot.behaviors.core.actions.gripper import (
    GripperAction,
    _HandSpec,
)


def test_async_gripper_uses_one_goal_for_all_fingers():
    action = object.__new__(GripperAction)
    action._config = SimpleNamespace(
        gripper_finger_ids=[1, 2, 3],
        gripper_control_mode=85,
        gripper_exec_delay_ms=10,
        gripper_inter_finger_delay=0.04,
    )
    action._side = "left"
    action._manager = SimpleNamespace(
        _hand_spec=lambda side: _HandSpec(side, 0, 1, False)
    )

    goals = action._build_goals([(1000, 280, 0.4)])

    assert len(goals) == 1
    assert goals[0][0].finger_id == 0
    assert goals[0][0].position == 1000
    assert goals[0][1] == 0.4
