from glob import glob
import os

from setuptools import find_packages, setup

package_name = "hand_camera_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="create",
    maintainer_email="r465755956@gmail.com",
    description="USB camera driver nodes for left and right hand cameras.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "usb_camera_node = hand_camera_driver.usb_camera_node:main",
        ],
    },
)
