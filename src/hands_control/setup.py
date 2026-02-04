from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hands_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create',
    maintainer_email='r465755956@gmail.com',
    description='ROS2 control package for dual DexHand021S devices',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_control_server = hands_control.hand_control_server:main',
            'hand_control_client = hands_control.hand_control_client:main',
        ],
    },
)
