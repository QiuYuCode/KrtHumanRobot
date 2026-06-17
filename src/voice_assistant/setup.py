from setuptools import find_packages, setup
import os
from glob import glob

package_name = "voice_assistant"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (
            os.path.join("share", package_name, "scripts"),
            glob("scripts/*.sh"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="create",
    maintainer_email="r465755956@gmail.com",
    description="基于 py_trees + py_trees_ros 的语音对话助手",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "voice_node = voice_assistant.voice_node:main",
        ],
    },
)
