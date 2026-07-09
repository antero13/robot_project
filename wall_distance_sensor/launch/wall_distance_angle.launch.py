from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("driver_backend", default_value="vl53l1x"),
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("default_address", default_value="0x29"),
            DeclareLaunchArgument("left_address", default_value="0x2A"),
            DeclareLaunchArgument("right_address", default_value="0x2B"),
            DeclareLaunchArgument("left_xshut_pin", default_value="-1"),
            DeclareLaunchArgument("right_xshut_pin", default_value="-1"),
            DeclareLaunchArgument("xshut_pin_mode", default_value="BOARD"),
            DeclareLaunchArgument("ranging_mode", default_value="1"),
            DeclareLaunchArgument("distance_scale_m", default_value="0.001"),
            DeclareLaunchArgument("sensor_separation_m", default_value="0.29"),
            DeclareLaunchArgument("safe_distance_m", default_value="0.15"),
            DeclareLaunchArgument("min_valid_distance_m", default_value="0.02"),
            DeclareLaunchArgument("max_valid_distance_m", default_value="4.00"),
            DeclareLaunchArgument("filter_window_size", default_value="3"),
            DeclareLaunchArgument("update_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("field_of_view_rad", default_value="0.47"),
            DeclareLaunchArgument("measurement_frame_id", default_value="front_wall_sensors"),
            DeclareLaunchArgument("left_frame_id", default_value="front_left_tof"),
            DeclareLaunchArgument("right_frame_id", default_value="front_right_tof"),
            DeclareLaunchArgument("distance_angle_topic", default_value="/wall/distance_angle"),
            DeclareLaunchArgument("measurement_json_topic", default_value="/wall/measurement_json"),
            DeclareLaunchArgument("left_range_topic", default_value="/wall/left_range"),
            DeclareLaunchArgument("right_range_topic", default_value="/wall/right_range"),
            DeclareLaunchArgument("mock_left_distance_m", default_value="0.50"),
            DeclareLaunchArgument("mock_right_distance_m", default_value="0.50"),
            Node(
                package="wall_distance_sensor",
                executable="wall_distance_angle_node",
                name="wall_distance_angle_node",
                output="screen",
                parameters=[
                    {
                        "driver_backend": LaunchConfiguration("driver_backend"),
                        "i2c_bus": ParameterValue(LaunchConfiguration("i2c_bus"), value_type=int),
                        "default_address": LaunchConfiguration("default_address"),
                        "left_address": LaunchConfiguration("left_address"),
                        "right_address": LaunchConfiguration("right_address"),
                        "left_xshut_pin": ParameterValue(LaunchConfiguration("left_xshut_pin"), value_type=int),
                        "right_xshut_pin": ParameterValue(LaunchConfiguration("right_xshut_pin"), value_type=int),
                        "xshut_pin_mode": LaunchConfiguration("xshut_pin_mode"),
                        "ranging_mode": ParameterValue(LaunchConfiguration("ranging_mode"), value_type=int),
                        "distance_scale_m": ParameterValue(
                            LaunchConfiguration("distance_scale_m"),
                            value_type=float,
                        ),
                        "sensor_separation_m": ParameterValue(
                            LaunchConfiguration("sensor_separation_m"),
                            value_type=float,
                        ),
                        "safe_distance_m": ParameterValue(LaunchConfiguration("safe_distance_m"), value_type=float),
                        "min_valid_distance_m": ParameterValue(
                            LaunchConfiguration("min_valid_distance_m"),
                            value_type=float,
                        ),
                        "max_valid_distance_m": ParameterValue(
                            LaunchConfiguration("max_valid_distance_m"),
                            value_type=float,
                        ),
                        "filter_window_size": ParameterValue(
                            LaunchConfiguration("filter_window_size"),
                            value_type=int,
                        ),
                        "update_rate_hz": ParameterValue(LaunchConfiguration("update_rate_hz"), value_type=float),
                        "field_of_view_rad": ParameterValue(
                            LaunchConfiguration("field_of_view_rad"),
                            value_type=float,
                        ),
                        "measurement_frame_id": LaunchConfiguration("measurement_frame_id"),
                        "left_frame_id": LaunchConfiguration("left_frame_id"),
                        "right_frame_id": LaunchConfiguration("right_frame_id"),
                        "distance_angle_topic": LaunchConfiguration("distance_angle_topic"),
                        "measurement_json_topic": LaunchConfiguration("measurement_json_topic"),
                        "left_range_topic": LaunchConfiguration("left_range_topic"),
                        "right_range_topic": LaunchConfiguration("right_range_topic"),
                        "mock_left_distance_m": ParameterValue(
                            LaunchConfiguration("mock_left_distance_m"),
                            value_type=float,
                        ),
                        "mock_right_distance_m": ParameterValue(
                            LaunchConfiguration("mock_right_distance_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
