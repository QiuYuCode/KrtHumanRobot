from setuptools import find_packages, setup


package_name = "voice_test_tools"

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
    description="Standalone clients for voice subsystem testing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "asr_test_client = voice_test_tools.asr_test_client:main",
            "kws_test_client = voice_test_tools.kws_test_client:main",
            "play_test_client = voice_test_tools.play_test_client:main",
            "tts_test_client = voice_test_tools.tts_test_client:main",
            "vad_test_client = voice_test_tools.vad_test_client:main",
            "volume_test_client = voice_test_tools.volume_test_client:main",
        ],
    },
)
