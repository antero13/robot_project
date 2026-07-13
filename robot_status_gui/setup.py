from glob import glob
import os

from setuptools import setup


package_name = "robot_status_gui"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antero13",
    maintainer_email="antero13@todo.todo",
    description="Operator status GUI for the AI Robot Challenge runtime.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_status_gui = robot_status_gui.status_gui_node:main",
        ],
    },
)
