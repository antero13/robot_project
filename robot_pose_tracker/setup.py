from glob import glob
import os

from setuptools import setup


package_name = 'robot_pose_tracker'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antero13',
    maintainer_email='antero13@todo.todo',
    description='Track a planar robot pose from cmd_vel and IMU yaw rate.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_pose_tracker = robot_pose_tracker.robot_pose_tracker_node:main',
            'arena_visualizer = robot_pose_tracker.arena_visualizer_node:main',
        ],
    },
)
