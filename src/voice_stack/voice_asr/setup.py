from setuptools import find_packages, setup

package_name = "voice_asr"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="create",
    maintainer_email="r465755956@gmail.com",
    description="ASR 节点骨架，后续实现本地/云端双后端。",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_asr_node = voice_asr.asr_node:main",
        ],
    },
)
