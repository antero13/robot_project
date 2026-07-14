from glob import glob
import os

from setuptools import setup


package_name = "bbox_zone_controller"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antero13",
    maintainer_email="antero13@todo.todo",
    description="Control the robot from the largest YOLO bounding box and four image zones.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bbox_zone_controller = bbox_zone_controller.bbox_zone_controller_node:main",
        ],
    },
)
