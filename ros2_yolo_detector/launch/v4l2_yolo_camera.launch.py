from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ros2_yolo_detector"), "models", "best.pt"]
                ),
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
            ),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="720"),
            DeclareLaunchArgument("time_per_frame_numerator", default_value="1"),
            DeclareLaunchArgument("time_per_frame_denominator", default_value="10"),
            DeclareLaunchArgument("pixel_format", default_value="YUYV"),
            DeclareLaunchArgument("output_encoding", default_value="rgb8"),
            DeclareLaunchArgument("power_line_frequency", default_value="2"),
            DeclareLaunchArgument("auto_exposure", default_value="1"),
            DeclareLaunchArgument("exposure_time_absolute", default_value="200"),
            DeclareLaunchArgument("gain", default_value="20"),
            DeclareLaunchArgument("confidence", default_value="0.25"),
            DeclareLaunchArgument("iou", default_value="0.45"),
            DeclareLaunchArgument("device", default_value=""),
            DeclareLaunchArgument("tracker_enabled", default_value="true"),
            DeclareLaunchArgument("tracker_config", default_value="bytetrack.yaml"),
            DeclareLaunchArgument("tracker_persist", default_value="true"),
            DeclareLaunchArgument("publish_annotated", default_value="false"),
            DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
            DeclareLaunchArgument("target_topic", default_value="/target_object"),
            DeclareLaunchArgument("target_label_topic", default_value="/target_label"),
            DeclareLaunchArgument("avoid_topic", default_value="/avoid_object"),
            DeclareLaunchArgument("avoid_label_topic", default_value="/avoid_label"),
            DeclareLaunchArgument("avoid_objects_topic", default_value="/avoid_objects"),
            DeclareLaunchArgument("target_classes", default_value=""),
            DeclareLaunchArgument("avoid_classes", default_value=""),
            DeclareLaunchArgument("target_lock_enabled", default_value="true"),
            DeclareLaunchArgument("target_lock_timeout_s", default_value="0.7"),
            DeclareLaunchArgument("target_lock_iou_threshold", default_value="0.20"),
            DeclareLaunchArgument("target_lock_x_margin", default_value="0.30"),
            DeclareLaunchArgument("target_lock_y_margin", default_value="0.20"),
            DeclareLaunchArgument("target_switch_y_margin", default_value="0.12"),
            DeclareLaunchArgument("target_switch_score_margin", default_value="0.25"),
            DeclareLaunchArgument("target_center_weight", default_value="0.25"),
            DeclareLaunchArgument("avoid_target_iou_threshold", default_value="0.35"),
            DeclareLaunchArgument("publish_target", default_value="true"),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                output="screen",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_size": [
                            ParameterValue(LaunchConfiguration("image_width"), value_type=int),
                            ParameterValue(LaunchConfiguration("image_height"), value_type=int),
                        ],
                        "time_per_frame": [
                            ParameterValue(LaunchConfiguration("time_per_frame_numerator"), value_type=int),
                            ParameterValue(LaunchConfiguration("time_per_frame_denominator"), value_type=int),
                        ],
                        "pixel_format": LaunchConfiguration("pixel_format"),
                        "output_encoding": LaunchConfiguration("output_encoding"),
                        "power_line_frequency": ParameterValue(
                            LaunchConfiguration("power_line_frequency"), value_type=int
                        ),
                        "auto_exposure": ParameterValue(LaunchConfiguration("auto_exposure"), value_type=int),
                        "exposure_time_absolute": ParameterValue(
                            LaunchConfiguration("exposure_time_absolute"), value_type=int
                        ),
                        "gain": ParameterValue(LaunchConfiguration("gain"), value_type=int),
                    }
                ],
            ),
            Node(
                package="ros2_yolo_detector",
                executable="yolo_camera_node",
                name="yolo_camera_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "input_mode": "topic",
                        "image_topic": LaunchConfiguration("image_topic"),
                        "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
                        "device": LaunchConfiguration("device"),
                        "tracker_enabled": ParameterValue(
                            LaunchConfiguration("tracker_enabled"),
                            value_type=bool,
                        ),
                        "tracker_config": LaunchConfiguration("tracker_config"),
                        "tracker_persist": ParameterValue(
                            LaunchConfiguration("tracker_persist"),
                            value_type=bool,
                        ),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "publish_annotated": ParameterValue(
                            LaunchConfiguration("publish_annotated"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="ros2_yolo_detector",
                executable="detections_to_target_node",
                name="detections_to_target_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_target")),
                parameters=[
                    {
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "target_topic": LaunchConfiguration("target_topic"),
                        "target_label_topic": LaunchConfiguration("target_label_topic"),
                        "avoid_topic": LaunchConfiguration("avoid_topic"),
                        "avoid_label_topic": LaunchConfiguration("avoid_label_topic"),
                        "avoid_objects_topic": LaunchConfiguration("avoid_objects_topic"),
                        "target_classes": LaunchConfiguration("target_classes"),
                        "avoid_classes": LaunchConfiguration("avoid_classes"),
                        "target_lock_enabled": ParameterValue(
                            LaunchConfiguration("target_lock_enabled"),
                            value_type=bool,
                        ),
                        "target_lock_timeout_s": ParameterValue(
                            LaunchConfiguration("target_lock_timeout_s"),
                            value_type=float,
                        ),
                        "target_lock_iou_threshold": ParameterValue(
                            LaunchConfiguration("target_lock_iou_threshold"),
                            value_type=float,
                        ),
                        "target_lock_x_margin": ParameterValue(
                            LaunchConfiguration("target_lock_x_margin"),
                            value_type=float,
                        ),
                        "target_lock_y_margin": ParameterValue(
                            LaunchConfiguration("target_lock_y_margin"),
                            value_type=float,
                        ),
                        "target_switch_y_margin": ParameterValue(
                            LaunchConfiguration("target_switch_y_margin"),
                            value_type=float,
                        ),
                        "target_switch_score_margin": ParameterValue(
                            LaunchConfiguration("target_switch_score_margin"),
                            value_type=float,
                        ),
                        "target_center_weight": ParameterValue(
                            LaunchConfiguration("target_center_weight"),
                            value_type=float,
                        ),
                        "avoid_target_iou_threshold": ParameterValue(
                            LaunchConfiguration("avoid_target_iou_threshold"),
                            value_type=float,
                        ),
                        "min_confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=float),
                        "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=float),
                    }
                ],
            ),
        ]
    )
