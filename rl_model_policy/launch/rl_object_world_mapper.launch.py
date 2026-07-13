from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    float_arguments = {
        "pose_timeout_s": "0.5",
        "retention_s": "180.0",
        "confirmation_window_s": "1.0",
        "horizontal_fov_deg": "80.0",
        "vertical_fov_deg": "50.0",
        "camera_height_m": "0.18",
        "camera_pitch_deg": "15.0",
        "object_center_height_m": "0.04",
        "max_range_m": "4.5",
        "max_x_error": "0.35",
        "max_y_error": "0.22",
        "arena_half_extent_m": "2.0",
        "pickup_remove_radius_m": "0.75",
    }
    declarations = [
        DeclareLaunchArgument("detections_topic", default_value="/yolo/detections"),
        DeclareLaunchArgument("odometry_topic", default_value="/odom"),
        DeclareLaunchArgument("output_topic", default_value="/rl_estimated_objects"),
        DeclareLaunchArgument(
            "policy_state_topic",
            default_value="/rl_model_policy_state",
        ),
        DeclareLaunchArgument("target_classes", default_value=""),
        DeclareLaunchArgument("avoid_classes", default_value=""),
        DeclareLaunchArgument("min_confirmations", default_value="2"),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value=default)
        for name, default in float_arguments.items()
    )

    parameters = {
        "detections_topic": LaunchConfiguration("detections_topic"),
        "odometry_topic": LaunchConfiguration("odometry_topic"),
        "output_topic": LaunchConfiguration("output_topic"),
        "policy_state_topic": LaunchConfiguration("policy_state_topic"),
        "target_classes": LaunchConfiguration("target_classes"),
        "avoid_classes": LaunchConfiguration("avoid_classes"),
        "min_confirmations": ParameterValue(
            LaunchConfiguration("min_confirmations"),
            value_type=int,
        ),
    }
    parameters.update(
        {
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in float_arguments
        }
    )

    return LaunchDescription(
        declarations
        + [
            Node(
                package="rl_model_policy",
                executable="rl_object_world_mapper",
                name="rl_object_world_mapper",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
