from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("odometry_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "policy_state_topic",
                default_value="/rl_model_policy_state",
            ),
            DeclareLaunchArgument(
                "estimated_objects_topic",
                default_value="/rl_estimated_objects",
            ),
            DeclareLaunchArgument(
                "control_topic",
                default_value="/rl_model_policy_control",
            ),
            DeclareLaunchArgument("pose_offset_x", default_value="2.0"),
            DeclareLaunchArgument("pose_offset_y", default_value="2.0"),
            Node(
                package="robot_status_gui",
                executable="robot_status_gui",
                name="robot_status_gui",
                output="screen",
                parameters=[
                    {
                        "odometry_topic": LaunchConfiguration("odometry_topic"),
                        "policy_state_topic": LaunchConfiguration(
                            "policy_state_topic"
                        ),
                        "estimated_objects_topic": LaunchConfiguration(
                            "estimated_objects_topic"
                        ),
                        "control_topic": LaunchConfiguration("control_topic"),
                        "pose_offset_x": ParameterValue(
                            LaunchConfiguration("pose_offset_x"),
                            value_type=float,
                        ),
                        "pose_offset_y": ParameterValue(
                            LaunchConfiguration("pose_offset_y"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
