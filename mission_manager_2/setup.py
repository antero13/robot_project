from glob import glob
import os

from setuptools import setup


package_name = 'mission_manager_2'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antero13',
    maintainer_email='antero13@todo.todo',
    description='Pose-guided fixed-lane mission manager with YOLO pickup and wall correction.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager_2 = mission_manager_2.mission_manager_2_node:main',
        ],
    },
)
