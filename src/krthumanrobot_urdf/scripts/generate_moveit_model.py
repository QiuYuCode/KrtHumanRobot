#!/usr/bin/env python3
"""Create a portable planning URDF and one convex collision mesh per link."""

import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "robot.urdf"
OUTPUT = ROOT / "config" / "robot_moveit.urdf"
COLLISION_DIR = ROOT / "meshes" / "collision"
# A single hull closes the recess between wrist links 5 and 7. Their source
# collision pieces retain that necessary clearance at the home pose.
DETAILED_COLLISION_LINKS = {
    "left_arm_link5",
    "left_arm_link7",
    "right_arm_link5",
    "right_arm_link7",
}

# The CAD export uses -1.5 rad for the horizontal shoulder-pitch pose. MoveIt
# exposes that pose as zero while preserving the same physical transform.
ARM_LINK2_ZERO_OFFSET = 1.5
INITIAL_SHOULDER_PITCH = math.pi / 2.0


def stl_vertices(path):
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) == 84 + triangle_count * 50:
        triangles = np.ndarray(
            (triangle_count,),
            dtype=np.dtype(
                [
                    ("normal", "<f4", 3),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            ),
            buffer=data,
            offset=84,
        )
        return triangles["vertices"].reshape(-1, 3).astype(np.float64)
    vertices = []
    for line in data.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == b"vertex":
            vertices.append([float(value) for value in parts[1:]])
    if not vertices:
        raise ValueError(f"No STL vertices: {path}")
    return np.asarray(vertices, dtype=np.float64)


def transformed_vertices(collision):
    mesh = collision.find("geometry/mesh")
    vertices = stl_vertices(ROOT / mesh.get("filename"))
    scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
    origin = collision.find("origin")
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
    return Rotation.from_euler("xyz", rpy).apply(vertices * scale) + xyz


def convex_stl(vertices):
    points = np.unique(np.round(vertices, 4), axis=0)
    try:
        hull = ConvexHull(points)
        faces = points[hull.simplices].astype("<f4")
    except QhullError:
        low = points.min(axis=0) - 0.001
        high = points.max(axis=0) + 0.001
        corners = np.array(
            [
                [x, y, z]
                for x in (low[0], high[0])
                for y in (low[1], high[1])
                for z in (low[2], high[2])
            ]
        )
        faces = corners[ConvexHull(corners).simplices].astype("<f4")
    record = np.zeros(
        len(faces),
        dtype=np.dtype(
            [
                ("normal", "<f4", 3),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ]
        ),
    )
    record["vertices"] = faces
    return (
        b"MoveIt collision convex hull".ljust(80, b"\0")
        + struct.pack("<I", len(faces))
        + record.tobytes()
    )


def build():
    robot = ET.parse(SOURCE).getroot()
    for side in ("left", "right"):
        joint = robot.find(f"joint[@name='{side}_arm_link2_joint']")
        origin = joint.find("origin")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        # T_new(q) = T_old(q - offset), so compose the origin with the
        # inverse offset before publishing the generated planning model.
        rotation = Rotation.from_euler("xyz", rpy)
        rotation = rotation * Rotation.from_euler(
            "z", -ARM_LINK2_ZERO_OFFSET
        )
        origin.set(
            "rpy",
            " ".join(f"{value:.9f}" for value in rotation.as_euler("xyz")),
        )
        limit = joint.find("limit")
        for bound in ("lower", "upper"):
            limit.set(
                bound,
                str(float(limit.get(bound)) + ARM_LINK2_ZERO_OFFSET),
            )
    # Establish a ground reference from the original wheel meshes at zero
    # wheel rotation, before collision geometry is simplified.
    wheel_bottoms = []
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        joint = robot.find(f"joint[@name='{name}_wheel_joint']")
        if joint.find("parent").get("link") != "base_link":
            raise ValueError("Wheel joint must be relative to base_link")
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rotation = Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        )
        link = robot.find(f"link[@name='{joint.find('child').get('link')}']")
        vertices = np.concatenate(
            [
                rotation.apply(transformed_vertices(c)) + xyz
                for c in link.findall("collision")
            ]
        )
        wheel_bottoms.append(vertices[:, 2].min())
    if max(wheel_bottoms) - min(wheel_bottoms) > 0.001:
        raise ValueError("Wheel bottoms differ by more than 1 mm")
    robot.insert(0, ET.Element("link", name="base_footprint"))
    ground_joint = ET.SubElement(
        robot, "joint", name="base_footprint_joint", type="fixed"
    )
    ET.SubElement(ground_joint, "parent", link="base_footprint")
    ET.SubElement(ground_joint, "child", link="base_link")
    ET.SubElement(
        ground_joint,
        "origin",
        xyz=f"0 0 {-min(wheel_bottoms):.9f}",
        rpy="0 0 0",
    )
    generated_meshes = {}
    for link in robot.findall("link"):
        collisions = link.findall("collision")
        if not collisions:
            continue
        name = link.get("name")
        if name in DETAILED_COLLISION_LINKS:
            for collision in collisions:
                mesh = collision.find("geometry/mesh")
                mesh.set(
                    "filename",
                    "package://krthumanrobot_urdf/" + mesh.get("filename"),
                )
            continue
        # The chassis is concave: a single hull fills the arm clearances.
        # Keep its CAD part boundaries while simplifying each part separately.
        pieces = (
            [transformed_vertices(c) for c in collisions]
            if name == "base_link"
            else [
                np.concatenate([transformed_vertices(c) for c in collisions])
            ]
        )
        for collision in collisions:
            link.remove(collision)
        for index, vertices in enumerate(pieces):
            mesh_name = name if index == 0 else f"{name}_part_{index:03d}"
            generated_meshes[mesh_name] = convex_stl(vertices)
            collision = ET.SubElement(link, "collision")
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                geometry,
                "mesh",
                filename=(
                    "package://krthumanrobot_urdf/meshes/collision/"
                    f"{mesh_name}.stl"
                ),
            )
    for mesh in robot.findall(".//visual/geometry/mesh"):
        mesh.set(
            "filename", "package://krthumanrobot_urdf/" + mesh.get("filename")
        )

    control = ET.SubElement(
        robot, "ros2_control", name="KrtMockSystem", type="system"
    )
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    for joint in robot.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        item = ET.SubElement(control, "joint", name=joint.get("name"))
        ET.SubElement(item, "command_interface", name="position")
        state = ET.SubElement(item, "state_interface", name="position")
        initial = (
            INITIAL_SHOULDER_PITCH
            if joint.get("name", "").endswith("arm_link2_joint")
            else 0.0
        )
        ET.SubElement(state, "param", name="initial_value").text = str(initial)
        ET.SubElement(item, "state_interface", name="velocity")
    ET.indent(robot, space="  ")
    content = ET.tostring(
        robot, encoding="unicode", xml_declaration=True
    ).encode()
    return content, generated_meshes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    content, meshes = build()
    if args.check:
        assert OUTPUT.read_bytes() == content, "MoveIt URDF is stale"
        assert {path.stem for path in COLLISION_DIR.glob("*.stl")} == set(
            meshes
        ), "Collision mesh set is stale"
        for name, data in meshes.items():
            assert (COLLISION_DIR / f"{name}.stl").read_bytes() == data, name
        return
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    COLLISION_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(content)
    for name, data in meshes.items():
        (COLLISION_DIR / f"{name}.stl").write_bytes(data)


if __name__ == "__main__":
    main()
