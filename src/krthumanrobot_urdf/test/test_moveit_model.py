"""Checks for the reproducible MoveIt description."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "robot.urdf"
GENERATED = ROOT / "config" / "robot_moveit.urdf"


def test_generated_model_matches_source():
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "generate_moveit_model.py"),
            "--check",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_moveit_model_has_installable_meshes_and_simplified_collisions():
    source = ET.parse(SOURCE).getroot()
    model = ET.parse(GENERATED).getroot()
    assert {link.get("name") for link in model.findall("link")} - {
        "base_footprint"
    } == {link.get("name") for link in source.findall("link")}
    assert {joint.get("name") for joint in model.findall("joint")} - {
        "base_footprint_joint"
    } == {joint.get("name") for joint in source.findall("joint")}
    assert all(
        mesh.get("filename").startswith("package://krthumanrobot_urdf/meshes/")
        for mesh in model.findall(".//visual/geometry/mesh")
    )
    assert len(model.findall(".//collision/geometry/mesh")) < len(
        source.findall(".//collision/geometry/mesh")
    )
    assert len(
        model.find("link[@name='base_link']").findall("collision")
    ) == len(source.find("link[@name='base_link']").findall("collision"))
    assert model.find("ros2_control") is not None


def test_ground_frame_places_wheel_bottoms_on_ground():
    model = ET.parse(GENERATED).getroot()
    joint = model.find("joint[@name='base_footprint_joint']")
    assert joint is not None
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "base_footprint"
    assert joint.find("child").get("link") == "base_link"
    xyz = [float(value) for value in joint.find("origin").get("xyz").split()]
    assert xyz[:2] == [0.0, 0.0]
    # CAD wheel centers are -0.60627435 m; lowest mesh vertex is
    # -0.66769819 m in base_link. Check against that independent measurement.
    assert abs(xyz[2] - 0.66769819) < 1e-6
    children = {j.find("child").get("link") for j in model.findall("joint")}
    assert {link.get("name") for link in model.findall("link")} - children == {
        "base_footprint"
    }


def test_shoulder_pitch_uses_horizontal_zero():
    for side in ("left", "right"):
        joint = ET.parse(GENERATED).getroot().find(
            f"joint[@name='{side}_arm_link2_joint']"
        )
        limit = joint.find("limit")
        assert abs(float(limit.get("lower")) - (-1.8107963)) < 1e-6
        assert abs(float(limit.get("upper")) - 1.66920367) < 1e-6
