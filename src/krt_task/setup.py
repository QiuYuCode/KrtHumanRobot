import os
from glob import glob

from setuptools import find_packages, setup


package_name = "krt_task"

setup(
    name=package_name,
    version="0.1.0",
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
    description="Routine runner for KRT robot task sequences",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "routine_runner = krt_task.routine_runner:main",
            "krt_robot_data = krt_task.data_cli:main",
        ],
    },
)
