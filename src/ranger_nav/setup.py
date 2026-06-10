import os
from glob import glob

from setuptools import setup

package_name = 'ranger_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='Ranger 底盘 + MID360 + FAST-LIO 的 3D 建图与 Nav2 导航集成包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pcd2pgm = ranger_nav.pcd2pgm:main',
        ],
    },
)
