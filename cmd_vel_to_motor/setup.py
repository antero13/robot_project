from glob import glob
import os

from setuptools import setup

package_name = 'cmd_vel_to_motor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antero13',
    maintainer_email='antero13@todo.todo',
    description='Convert cmd_vel Twist commands to ros_robot_controller motor rps commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_motor = cmd_vel_to_motor.cmd_vel_to_motor_node:main',
        ],
    },
)
