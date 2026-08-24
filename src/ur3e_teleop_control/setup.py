import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3e_teleop_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thijs',
    maintainer_email='t.welling@student.tue.nl',
    description='Teleoperation of UR3e using 3D Systems Touch and MoveIt Servo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'touch_publisher = ur3e_teleop_control.touch_publisher:main',
            'task_space_controller = ur3e_teleop_control.task_space_controller:main',
        ],
    },
)
