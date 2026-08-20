from pathlib import Path


def test_robot_launch_passes_matching_hand_adapter_indices():
    launch = (Path(__file__).parents[1] / "launch/robot.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("left_hand_adapter_index", default_value="1")' in launch
    assert 'DeclareLaunchArgument("right_hand_adapter_index", default_value="0")' in launch
    assert '"left_hand_adapter_index": LaunchConfiguration(' in launch
    assert '"right_hand_adapter_index": LaunchConfiguration(' in launch


def test_routine_describe_uses_camera_select_and_surfaces_failure_message():
    template = Path(__file__).parents[1] / "templates/console.html"
    source = template.read_text(encoding="utf-8")

    assert "['head','头部']" in source
    assert "['left_palm','左手']" in source
    assert "['right_palm','右手']" in source
    assert "if(s.status==='failed'&&s.message)msg(s.message)" in source
