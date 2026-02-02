from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'device_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='quiyucode@163.com',
    description='外部设备测试功能包 - 测试摄像头、麦克风、扬声器',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_test = device_test.camera_test_node:main',
            'usb_camera_test = device_test.usb_camera_test_node:main',
            'realsense_test = device_test.realsense_test_node:main',
            'mic_test = device_test.mic_test_node:main',
            'speaker_test = device_test.speaker_test_node:main',
            'device_list = device_test.device_list:main',
        ],
    },
)
