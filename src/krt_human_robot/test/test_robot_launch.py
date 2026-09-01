from pathlib import Path


def test_robot_launch_passes_matching_hand_adapter_indices():
    launch = (Path(__file__).parents[1] / "launch/robot.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("left_hand_adapter_index", default_value="1")' in launch
    assert 'DeclareLaunchArgument("right_hand_adapter_index", default_value="0")' in launch
    assert '"left_hand_adapter_index": LaunchConfiguration(' in launch
    assert '"right_hand_adapter_index": LaunchConfiguration(' in launch


def test_robot_launch_passes_shared_database_to_core_process():
    launch = (Path(__file__).parents[1] / "launch/robot.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"KRT_ROBOT_DB": robot_db' in launch


def test_core_runner_never_syncs_python_dependencies_at_boot():
    """DNS loss during boot must not stop the core behavior tree."""
    runner = Path(__file__).parents[1] / "scripts/run_krt_human_robot_node.sh"
    source = runner.read_text(encoding="utf-8")

    assert "exec uv run --no-sync python -m" in source


def test_robot_launch_delays_arm_drivers_until_controllers_settle():
    """Arm drivers must not query firmware while USB CAN hardware is still booting."""
    launch = (Path(__file__).parents[1] / "launch/robot.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("arm_start_delay_s", default_value="15.0")' in launch
    assert 'period=LaunchConfiguration("arm_start_delay_s"), actions=arm_actions' in launch


def test_routine_describe_uses_camera_select_and_surfaces_failure_message():
    template = Path(__file__).parents[1] / "templates/console.html"
    source = template.read_text(encoding="utf-8")
    compact = "".join(source.split())

    assert '["head","头部"]' in compact
    assert '["left_palm","左手"]' in compact
    assert '["right_palm","右手"]' in compact
    assert 'if(s.status==="failed"&&s.message)msg(s.message)' in compact
