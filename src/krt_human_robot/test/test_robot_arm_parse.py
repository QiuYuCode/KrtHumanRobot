import sys
import types


fake_loguru = types.ModuleType("loguru")
fake_loguru.logger = types.SimpleNamespace(
    debug=lambda *args, **kwargs: None,
    info=lambda *args, **kwargs: None,
    warning=lambda *args, **kwargs: None,
    error=lambda *args, **kwargs: None,
    exception=lambda *args, **kwargs: None,
)
sys.modules.setdefault("loguru", fake_loguru)

from krt_human_robot.behaviors.core.actions.robot_arm import parse_robot_arm_command


def test_parse_teach_commands():
    start = parse_robot_arm_command("左臂进入示教")
    assert start.arm_side == "left"
    assert start.operation == "enter_teach"

    stop = parse_robot_arm_command("右臂保存动作组敬礼")
    assert stop.arm_side == "right"
    assert stop.operation == "exit_teach"
    assert stop.group_name == "敬礼"

    run = parse_robot_arm_command("左臂回放动作组挥手")
    assert run.arm_side == "left"
    assert run.operation == "run_group"
    assert run.group_name == "挥手"
