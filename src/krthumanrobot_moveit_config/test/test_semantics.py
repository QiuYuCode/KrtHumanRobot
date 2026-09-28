"""Validate MoveIt groups and controller joints against the robot model."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent / "krthumanrobot_urdf" / "config" / "robot_moveit.urdf"


def test_srdf_is_current_and_groups_match_controllers():
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "generate_semantics.py"),
            "--check",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    urdf = ET.parse(MODEL).getroot()
    srdf = ET.parse(ROOT / "config" / "robot.srdf").getroot()
    controllers = yaml.safe_load(
        (ROOT / "config" / "moveit_controllers.yaml").read_text()
    )
    names = {joint.get("name") for joint in urdf.findall("joint")}
    assert {group.get("name") for group in srdf.findall("group")} == {
        "left_arm",
        "right_arm",
        "both_arms",
    }
    for side in ("left", "right"):
        controller_joints = set(
            controllers["moveit_simple_controller_manager"][
                f"{side}_arm_controller"
            ]["joints"]
        )
        group = srdf.find(f"group[@name='{side}_arm']")
        chain = group.find("chain")
        assert chain.get("base_link") == f"{side}_arm_base_link"
        assert chain.get("tip_link") == f"{side}_hand_base_link"
        assert controller_joints == {
            f"{side}_arm_link{index}_joint" for index in range(1, 8)
        }
        assert controller_joints <= names
