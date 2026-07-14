from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("wall_distance_sensor"),
                "launch",
                "wall_distance_angle.launch.py",
            ])
        ),
        launch_arguments={
            "driver_backend": LaunchConfiguration("driver_backend"),
            "left_i2c_bus": LaunchConfiguration("left_i2c_bus"),
            "right_i2c_bus": LaunchConfiguration("right_i2c_bus"),
            "sensor_separation_m": LaunchConfiguration("sensor_separation_m"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("driver_backend", default_value="vl53l1x"),
        DeclareLaunchArgument("left_i2c_bus", default_value="7"),
        DeclareLaunchArgument("right_i2c_bus", default_value="1"),
        DeclareLaunchArgument("sensor_separation_m", default_value="0.29"),
        DeclareLaunchArgument("launch_motor_bridge", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("angle_tolerance_deg", default_value="2.0"),
        DeclareLaunchArgument("alignment_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("minimum_angular_z", default_value="0.05"),
        DeclareLaunchArgument("maximum_angular_z", default_value="0.20"),
        DeclareLaunchArgument("minimum_wall_distance_m", default_value="0.35"),
        DeclareLaunchArgument("maximum_wall_distance_m", default_value="2.0"),
        sensor_launch,
        Node(
            package="wall_distance_sensor",
            executable="wall_alignment_test_node",
            name="wall_alignment_test",
            output="screen",
            parameters=[{
                "auto_start": ParameterValue(
                    LaunchConfiguration("auto_start"),
                    value_type=bool,
                ),
                "angle_tolerance_deg": ParameterValue(
                    LaunchConfiguration("angle_tolerance_deg"),
                    value_type=float,
                ),
                "alignment_timeout_s": ParameterValue(
                    LaunchConfiguration("alignment_timeout_s"),
                    value_type=float,
                ),
                "minimum_angular_z": ParameterValue(
                    LaunchConfiguration("minimum_angular_z"),
                    value_type=float,
                ),
                "maximum_angular_z": ParameterValue(
                    LaunchConfiguration("maximum_angular_z"),
                    value_type=float,
                ),
                "minimum_wall_distance_m": ParameterValue(
                    LaunchConfiguration("minimum_wall_distance_m"),
                    value_type=float,
                ),
                "maximum_wall_distance_m": ParameterValue(
                    LaunchConfiguration("maximum_wall_distance_m"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="cmd_vel_to_motor",
            executable="cmd_vel_to_motor",
            name="wall_alignment_cmd_vel_to_motor",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_motor_bridge")),
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "motor_topic": "/ros_robot_controller/set_motor",
                "wheel_radius_m": 0.05,
                "wheel_separation_m": 0.32,
                "left_motor_ids": [4, 3],
                "right_motor_ids": [2, 1],
                "left_motor_signs": [1.0, 1.0],
                "right_motor_signs": [-1.0, -1.0],
                "max_rps": 2.0,
                "publish_rate_hz": 20.0,
                "command_timeout_s": 0.5,
            }],
        ),
    ])
