#!/usr/bin/env python3
"""Generate SRDF groups and structural collision exemptions from the URDF."""

import argparse
from itertools import combinations
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent / "krthumanrobot_urdf" / "config" / "robot_moveit.urdf"
OUTPUT = ROOT / "config" / "robot.srdf"


def build():
    urdf = ET.parse(MODEL).getroot()
    robot = ET.Element("robot", name=urdf.get("name"))
    joint_elements = urdf.findall("joint")
    for side in ("left", "right"):
        group = ET.SubElement(robot, "group", name=f"{side}_arm")
        ET.SubElement(
            group,
            "chain",
            base_link=f"{side}_arm_base_link",
            tip_link=f"{side}_hand_base_link",
        )
        state = ET.SubElement(
            robot, "group_state", name="home", group=f"{side}_arm"
        )
        for index in range(1, 8):
            ET.SubElement(
                state,
                "joint",
                name=f"{side}_arm_link{index}_joint",
                value="0",
            )
    both = ET.SubElement(robot, "group", name="both_arms")
    ET.SubElement(both, "group", name="left_arm")
    ET.SubElement(both, "group", name="right_arm")
    state = ET.SubElement(robot, "group_state", name="home", group="both_arms")
    for side in ("left", "right"):
        for index in range(1, 8):
            ET.SubElement(
                state,
                "joint",
                name=f"{side}_arm_link{index}_joint",
            value="0",
            )

    # Linked bodies share a joint frame; those collision pairs are structural.
    pairs = set()
    fixed_parents = {}
    for joint in joint_elements:
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        pairs.add(tuple(sorted((parent, child))))
        if joint.get("type") == "fixed":
            fixed_parents[child] = parent
    rigid_groups = {}
    for link in urdf.findall("link"):
        name = link.get("name")
        root = name
        while root in fixed_parents:
            root = fixed_parents[root]
        rigid_groups.setdefault(root, []).append(name)
    fixed_pairs = {
        tuple(sorted(pair))
        for links in rigid_groups.values()
        for pair in combinations(links, 2)
    }
    pairs.update(fixed_pairs)
    # The exported wrist shells overlap during normal link7 rotation. These
    # internal covers are separated by link6, and are not useful safety pairs.
    wrist_pairs = {
        (f"{side}_arm_link5", f"{side}_arm_link7")
        for side in ("left", "right")
    }
    pairs.update(wrist_pairs)
    for left, right in sorted(pairs):
        ET.SubElement(
            robot,
            "disable_collisions",
            link1=left,
            link2=right,
            reason=(
                "Fixed"
                if (left, right) in fixed_pairs
                else "Never"
                if (left, right) in wrist_pairs
                else "Adjacent"
            ),
        )
    ET.indent(robot, space="  ")
    return ET.tostring(
        robot, encoding="unicode", xml_declaration=True
    ).encode()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    content = build()
    if args.check:
        assert OUTPUT.read_bytes() == content, "SRDF is stale"
    else:
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_bytes(content)


if __name__ == "__main__":
    main()
