from glob import glob
import os

from setuptools import find_packages, setup

package_name = "ros2_yolo_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AI ROBOT",
    maintainer_email="user@example.com",
    description="Run a YOLO .pt model on ROS camera images.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_camera_node = ros2_yolo_detector.yolo_camera_node:main",
            "detections_to_target_node = ros2_yolo_detector.detections_to_target_node:main",
        ],
    },
)
