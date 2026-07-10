from glob import glob
from setuptools import setup

package_name = 'rl_model_policy'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Runs the trained RL avoid/search policy and publishes cmd_vel.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rl_model_policy = rl_model_policy.rl_model_policy_node:main',
        ],
    },
)
