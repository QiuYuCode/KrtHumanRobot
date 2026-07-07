from glob import glob
import os
from setuptools import find_packages, setup

package_name = "agx_action_group_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@todo.todo",
    description="Run named AGX action groups via ROS 2 action",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "action_group_runner = agx_action_group_runner.runner_node:main",
            "teach_action_group = agx_action_group_runner.teach_action_group_node:main",
            "teach_record_replay = agx_action_group_runner.teach_record_replay_node:main",
        ],
    },
)
