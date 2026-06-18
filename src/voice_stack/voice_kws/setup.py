from setuptools import find_packages, setup

package_name = "voice_kws"

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
    description="关键词唤醒节点骨架，后续接入 sherpa KWS。",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_kws_node = voice_kws.kws_node:main",
        ],
    },
)
