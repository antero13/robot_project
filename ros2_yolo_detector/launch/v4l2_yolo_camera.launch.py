from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="best.pt"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
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
            DeclareLaunchArgument("publish_annotated", default_value="false"),
            DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
            DeclareLaunchArgument("target_topic", default_value="/target_object"),
            DeclareLaunchArgument("target_label_topic", default_value="/target_label"),
            DeclareLaunchArgument("avoid_topic", default_value="/avoid_object"),
            DeclareLaunchArgument("avoid_label_topic", default_value="/avoid_label"),
            DeclareLaunchArgument("target_classes", default_value=""),
            DeclareLaunchArgument("avoid_classes", default_value=""),
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
                        "target_classes": LaunchConfiguration("target_classes"),
                        "avoid_classes": LaunchConfiguration("avoid_classes"),
                        "min_confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=float),
                        "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=float),
                    }
                ],
            ),
        ]
    )
