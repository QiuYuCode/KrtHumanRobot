from glob import glob
import os

from setuptools import find_packages, setup

package_name = "krt_human_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="create",
    maintainer_email="r465755956@gmail.com",
    description="KrtHumanRobot core behavior tree entry point",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "krt_human_robot_node = krt_human_robot.robot_node:main",
        ],
    },
)
