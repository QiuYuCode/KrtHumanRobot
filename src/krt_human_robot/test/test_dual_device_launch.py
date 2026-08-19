"""Static contract tests for the dual-device launch entry points."""

from pathlib import Path


WORKSPACE_SRC = Path(__file__).parents[2]
X86_LAUNCH = Path(__file__).parents[1] / "launch" / "x86_bringup.launch.py"
JETSON_LAUNCH = (
    WORKSPACE_SRC / "hand_camera_driver" / "launch" / "jetson_cameras.launch.py"
)
NAV3D_LAUNCH = (
    WORKSPACE_SRC / "ranger_nav" / "launch" / "navigation_3dloc.launch.py"
)


def test_x86_bringup_keeps_navigation_opt_in_and_excludes_cameras():
    """The x86 entry point defaults navigation off but can explicitly enable it."""
    source = X86_LAUNCH.read_text(encoding="utf-8")

    assert '"enable_camera_stack": "false"' in source
    assert '"enable_arm_control_stack": "true"' in source
    assert '"enable_web_console": "true"' in source
    assert '"enable_hands_control_stack": "false"' in source
    assert '"enable_navigation", default_value="false"' in source
    assert 'IfCondition(LaunchConfiguration("enable_navigation"))' in source
    assert '"rviz": "false"' in source
    assert "navigation_3dloc.launch.py" in source
    assert "navigation.launch.py" in source


def test_jetson_bringup_contains_only_realsense_and_hand_cameras():
    """The Jetson entry point owns the three cameras at the target profile."""
    source = JETSON_LAUNCH.read_text(encoding="utf-8")

    assert "realsense2_camera" in source
    assert "rs_launch.py" in source
    assert "hand_cameras.launch.py" in source
    assert 'executable="compressed_image_relay"' in source
    assert '"input_topic": "/camera/camera/color/image_raw"' in source
    assert '"output_topic": "/camera/camera/color/image_jpeg"' in source
    assert '"rgb_camera.color_profile": "640x480x15"' in source
    assert '"fps": "15.0"' in source
    assert '"jpeg_quality": "70"' in source
    for forbidden in ("agx_arm_ctrl", "ranger_nav", "voice_assistant"):
        assert forbidden not in source


def test_3d_navigation_clears_inherited_namespace():
    """Arm launch namespaces must not become the Nav2 parameter root key."""
    source = NAV3D_LAUNCH.read_text(encoding="utf-8")

    assert "'namespace': ''" in source
