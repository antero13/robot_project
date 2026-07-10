from glob import glob
from setuptools import setup

package_name = 'timed_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antero13',
    maintainer_email='antero13@todo.todo',
    description='Open-loop timed distance and angle motion commands using cmd_vel.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'timed_motion_node = timed_motion.timed_motion_node:main',
        ],
    },
)
