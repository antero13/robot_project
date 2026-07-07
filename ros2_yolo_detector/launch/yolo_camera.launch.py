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
            DeclareLaunchArgument("input_mode", default_value="topic"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("raw_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
            DeclareLaunchArgument("target_topic", default_value="/target_object"),
            DeclareLaunchArgument("target_label_topic", default_value="/target_label"),
            DeclareLaunchArgument("avoid_topic", default_value="/avoid_object"),
            DeclareLaunchArgument("avoid_label_topic", default_value="/avoid_label"),
            DeclareLaunchArgument("target_classes", default_value=""),
            DeclareLaunchArgument("avoid_classes", default_value=""),
            DeclareLaunchArgument("publish_target", default_value="true"),
            DeclareLaunchArgument("confidence", default_value="0.25"),
            DeclareLaunchArgument("iou", default_value="0.45"),
            DeclareLaunchArgument("device", default_value=""),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("camera_width", default_value="0"),
            DeclareLaunchArgument("camera_height", default_value="0"),
            DeclareLaunchArgument("camera_fps", default_value="30.0"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera"),
            DeclareLaunchArgument("camera_auto_exposure", default_value="false"),
            DeclareLaunchArgument("camera_exposure", default_value="200.0"),
            DeclareLaunchArgument("camera_manual_auto_exposure_value", default_value="1.0"),
            DeclareLaunchArgument("camera_auto_exposure_value", default_value="3.0"),
            DeclareLaunchArgument("publish_raw", default_value="false"),
            DeclareLaunchArgument("publish_annotated", default_value="true"),
            Node(
                package="ros2_yolo_detector",
                executable="yolo_camera_node",
                name="yolo_camera_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "input_mode": LaunchConfiguration("input_mode"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "raw_topic": LaunchConfiguration("raw_topic"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
                        "device": LaunchConfiguration("device"),
                        "camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
                        "camera_width": ParameterValue(LaunchConfiguration("camera_width"), value_type=int),
                        "camera_height": ParameterValue(LaunchConfiguration("camera_height"), value_type=int),
                        "camera_fps": ParameterValue(LaunchConfiguration("camera_fps"), value_type=float),
                        "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                        "camera_auto_exposure": ParameterValue(
                            LaunchConfiguration("camera_auto_exposure"), value_type=bool
                        ),
                        "camera_exposure": ParameterValue(LaunchConfiguration("camera_exposure"), value_type=float),
                        "camera_manual_auto_exposure_value": ParameterValue(
                            LaunchConfiguration("camera_manual_auto_exposure_value"), value_type=float
                        ),
                        "camera_auto_exposure_value": ParameterValue(
                            LaunchConfiguration("camera_auto_exposure_value"), value_type=float
                        ),
                        "publish_raw": ParameterValue(LaunchConfiguration("publish_raw"), value_type=bool),
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
                        "image_width": ParameterValue(LaunchConfiguration("camera_width"), value_type=float),
                        "image_height": ParameterValue(LaunchConfiguration("camera_height"), value_type=float),
                    }
                ],
            ),
        ]
    )
