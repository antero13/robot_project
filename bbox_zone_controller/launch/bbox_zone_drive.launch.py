from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_robot_controller"),
                "launch",
                "ros_robot_controller.launch.xml",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_robot_controller")),
    )
    camera_and_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros2_yolo_detector"),
                "launch",
                "v4l2_yolo_camera.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_camera")),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "video_device": LaunchConfiguration("video_device"),
            "image_width": LaunchConfiguration("image_width"),
            "image_height": LaunchConfiguration("image_height"),
            "time_per_frame_denominator": LaunchConfiguration("camera_fps"),
            "confidence": LaunchConfiguration("confidence"),
            "imgsz": LaunchConfiguration("imgsz"),
            "device": LaunchConfiguration("yolo_device"),
            "publish_annotated": LaunchConfiguration("publish_annotated"),
            "publish_target": "false",
        }.items(),
    )
    motor_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cmd_vel_to_motor"),
                "launch",
                "cmd_vel_to_motor.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("launch_motor_bridge")),
    )

    controller = Node(
        package="bbox_zone_controller",
        executable="bbox_zone_controller",
        name="bbox_zone_controller",
        output="screen",
        parameters=[{
            "active_on_start": ParameterValue(
                LaunchConfiguration("active_on_start"), value_type=bool
            ),
            "dry_run": ParameterValue(
                LaunchConfiguration("dry_run"), value_type=bool
            ),
            "min_confidence": ParameterValue(
                LaunchConfiguration("confidence"), value_type=float
            ),
            "fallback_image_width": ParameterValue(
                LaunchConfiguration("image_width"), value_type=float
            ),
            "fallback_image_height": ParameterValue(
                LaunchConfiguration("image_height"), value_type=float
            ),
            "target_classes": LaunchConfiguration("target_classes"),
            "point1_x": ParameterValue(LaunchConfiguration("point1_x"), value_type=float),
            "point1_y": ParameterValue(LaunchConfiguration("point1_y"), value_type=float),
            "point2_x": ParameterValue(LaunchConfiguration("point2_x"), value_type=float),
            "point2_y": ParameterValue(LaunchConfiguration("point2_y"), value_type=float),
            "point3_x": ParameterValue(LaunchConfiguration("point3_x"), value_type=float),
            "point3_y": ParameterValue(LaunchConfiguration("point3_y"), value_type=float),
            "point4_x": ParameterValue(LaunchConfiguration("point4_x"), value_type=float),
            "point4_y": ParameterValue(LaunchConfiguration("point4_y"), value_type=float),
            "straight_linear_x": ParameterValue(
                LaunchConfiguration("straight_linear_x"), value_type=float
            ),
            "avoid_turn_linear_x": ParameterValue(
                LaunchConfiguration("avoid_turn_linear_x"), value_type=float
            ),
            "avoid_turn_angular_z": ParameterValue(
                LaunchConfiguration("avoid_turn_angular_z"), value_type=float
            ),
            "target_forward_linear_x": ParameterValue(
                LaunchConfiguration("target_forward_linear_x"), value_type=float
            ),
            "target_center_tolerance": ParameterValue(
                LaunchConfiguration("target_center_tolerance"), value_type=float
            ),
            "target_angular_gain": ParameterValue(
                LaunchConfiguration("target_angular_gain"), value_type=float
            ),
            "target_min_angular_z": ParameterValue(
                LaunchConfiguration("target_min_angular_z"), value_type=float
            ),
            "target_max_angular_z": ParameterValue(
                LaunchConfiguration("target_max_angular_z"), value_type=float
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros2_yolo_detector"),
                "models",
                "best.pt",
            ]),
        ),
        DeclareLaunchArgument(
            "video_device",
            default_value=(
                "/dev/v4l/by-path/"
                "platform-3610000.usb-usb-0:2.1:1.0-video-index0"
            ),
        ),
        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="10"),
        DeclareLaunchArgument("confidence", default_value="0.25"),
        DeclareLaunchArgument("imgsz", default_value="800"),
        DeclareLaunchArgument("yolo_device", default_value=""),
        DeclareLaunchArgument("publish_annotated", default_value="false"),
        DeclareLaunchArgument(
            "target_classes",
            default_value="12,20,6,8,apple,banana,orange,pineapple",
        ),
        DeclareLaunchArgument("point1_x", default_value="-0.8600"),
        DeclareLaunchArgument("point1_y", default_value="0.9900"),
        DeclareLaunchArgument("point2_x", default_value="-0.7600"),
        DeclareLaunchArgument("point2_y", default_value="0.8333"),
        DeclareLaunchArgument("point3_x", default_value="0.7375"),
        DeclareLaunchArgument("point3_y", default_value="0.9933"),
        DeclareLaunchArgument("point4_x", default_value="0.5825"),
        DeclareLaunchArgument("point4_y", default_value="0.7367"),
        DeclareLaunchArgument("straight_linear_x", default_value="0.10"),
        DeclareLaunchArgument("avoid_turn_linear_x", default_value="0.06"),
        DeclareLaunchArgument("avoid_turn_angular_z", default_value="0.45"),
        DeclareLaunchArgument("target_forward_linear_x", default_value="0.08"),
        DeclareLaunchArgument("target_center_tolerance", default_value="0.10"),
        DeclareLaunchArgument("target_angular_gain", default_value="0.80"),
        DeclareLaunchArgument("target_min_angular_z", default_value="0.10"),
        DeclareLaunchArgument("target_max_angular_z", default_value="0.45"),
        DeclareLaunchArgument("active_on_start", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("launch_robot_controller", default_value="true"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("launch_motor_bridge", default_value="true"),
        robot_controller,
        camera_and_yolo,
        motor_bridge,
        controller,
    ])
