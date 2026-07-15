from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_camera(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera_node",
            output="screen",
            parameters=[
                {
                    "video_device": value("video_device"),
                    "image_size": [
                        int(value("image_width")),
                        int(value("image_height")),
                    ],
                    "time_per_frame": [
                        int(value("time_per_frame_numerator")),
                        int(value("time_per_frame_denominator")),
                    ],
                    "pixel_format": value("pixel_format"),
                    "output_encoding": value("output_encoding"),
                    "power_line_frequency": int(value("power_line_frequency")),
                    "auto_exposure": int(value("auto_exposure")),
                    "exposure_time_absolute": int(value("exposure_time_absolute")),
                    "gain": int(value("gain")),
                }
            ],
        )
    ]


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
            DeclareLaunchArgument("imgsz", default_value="800"),
            DeclareLaunchArgument("correction_enabled", default_value="true"),
            DeclareLaunchArgument("correction_gamma", default_value="0.65"),
            DeclareLaunchArgument("correction_clahe_clip_limit", default_value="1.2"),
            DeclareLaunchArgument("correction_clahe_tile_grid", default_value="8"),
            DeclareLaunchArgument("correction_chroma_gain", default_value="1.3"),
            DeclareLaunchArgument("correction_backend", default_value="auto"),
            DeclareLaunchArgument("correction_device", default_value="cuda:0"),
            DeclareLaunchArgument("performance_log_interval_s", default_value="5.0"),
            DeclareLaunchArgument("publish_annotated", default_value="false"),
            DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
            DeclareLaunchArgument("target_topic", default_value="/target_object"),
            DeclareLaunchArgument("target_label_topic", default_value="/target_label"),
            DeclareLaunchArgument(
                "target_visibility_topic",
                default_value="/target_visible",
            ),
            DeclareLaunchArgument(
                "target_center_y_topic",
                default_value="/target_center_y",
            ),
            DeclareLaunchArgument("avoid_topic", default_value="/avoid_object"),
            DeclareLaunchArgument("avoid_label_topic", default_value="/avoid_label"),
            DeclareLaunchArgument("avoid_objects_topic", default_value="/avoid_objects"),
            DeclareLaunchArgument("target_classes", default_value=""),
            DeclareLaunchArgument("avoid_classes", default_value=""),
            DeclareLaunchArgument("target_center_weight", default_value="0.25"),
            DeclareLaunchArgument("avoid_target_iou_threshold", default_value="0.35"),
            DeclareLaunchArgument("publish_target", default_value="true"),
            OpaqueFunction(function=launch_camera),
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
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                        "correction_enabled": ParameterValue(
                            LaunchConfiguration("correction_enabled"), value_type=bool
                        ),
                        "correction_gamma": ParameterValue(
                            LaunchConfiguration("correction_gamma"), value_type=float
                        ),
                        "correction_clahe_clip_limit": ParameterValue(
                            LaunchConfiguration("correction_clahe_clip_limit"), value_type=float
                        ),
                        "correction_clahe_tile_grid": ParameterValue(
                            LaunchConfiguration("correction_clahe_tile_grid"), value_type=int
                        ),
                        "correction_chroma_gain": ParameterValue(
                            LaunchConfiguration("correction_chroma_gain"), value_type=float
                        ),
                        "correction_backend": LaunchConfiguration(
                            "correction_backend"
                        ),
                        "correction_device": LaunchConfiguration(
                            "correction_device"
                        ),
                        "performance_log_interval_s": ParameterValue(
                            LaunchConfiguration("performance_log_interval_s"),
                            value_type=float,
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
                        "target_visibility_topic": LaunchConfiguration(
                            "target_visibility_topic"
                        ),
                        "target_center_y_topic": LaunchConfiguration(
                            "target_center_y_topic"
                        ),
                        "avoid_topic": LaunchConfiguration("avoid_topic"),
                        "avoid_label_topic": LaunchConfiguration("avoid_label_topic"),
                        "avoid_objects_topic": LaunchConfiguration("avoid_objects_topic"),
                        "target_classes": ParameterValue(
                            LaunchConfiguration("target_classes"), value_type=str
                        ),
                        "avoid_classes": ParameterValue(
                            LaunchConfiguration("avoid_classes"), value_type=str
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
