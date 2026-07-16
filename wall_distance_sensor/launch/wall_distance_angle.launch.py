from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument("driver_backend", default_value="vl53l1x"),
        DeclareLaunchArgument("left_i2c_bus", default_value="1"),
        DeclareLaunchArgument("right_i2c_bus", default_value="7"),
        DeclareLaunchArgument(
            "left_address",
            default_value="41",
            description="Integer I2C address; 41 decimal is 0x29.",
        ),
        DeclareLaunchArgument(
            "right_address",
            default_value="41",
            description="Integer I2C address; 41 decimal is 0x29.",
        ),
        DeclareLaunchArgument(
            "ranging_mode",
            default_value="3",
            description="1=short, 2=medium, 3=long.",
        ),
        DeclareLaunchArgument("distance_scale_m", default_value="0.001"),
        DeclareLaunchArgument("timing_budget_us", default_value="41000"),
        DeclareLaunchArgument(
            "inter_measurement_period_ms",
            default_value="50",
        ),
        DeclareLaunchArgument("sensor_separation_m", default_value="0.29"),
        DeclareLaunchArgument("safe_distance_m", default_value="0.15"),
        DeclareLaunchArgument("min_valid_distance_m", default_value="0.02"),
        DeclareLaunchArgument("max_valid_distance_m", default_value="4.00"),
        DeclareLaunchArgument("filter_window_size", default_value="3"),
        DeclareLaunchArgument("update_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("input_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("max_pair_skew_s", default_value="0.10"),
        DeclareLaunchArgument("field_of_view_rad", default_value="0.47"),
        DeclareLaunchArgument(
            "measurement_frame_id",
            default_value="front_wall_sensors",
        ),
        DeclareLaunchArgument("left_frame_id", default_value="front_left_tof"),
        DeclareLaunchArgument("right_frame_id", default_value="front_right_tof"),
        DeclareLaunchArgument(
            "distance_angle_topic",
            default_value="/wall/distance_angle",
        ),
        DeclareLaunchArgument(
            "measurement_json_topic",
            default_value="/wall/measurement_json",
        ),
        DeclareLaunchArgument("left_range_topic", default_value="/wall/left_range"),
        DeclareLaunchArgument(
            "right_range_topic",
            default_value="/wall/right_range",
        ),
        DeclareLaunchArgument("mock_left_distance_m", default_value="0.50"),
        DeclareLaunchArgument("mock_right_distance_m", default_value="0.50"),
    ]

    common_sensor_parameters = {
        "driver_backend": LaunchConfiguration("driver_backend"),
        "ranging_mode": typed("ranging_mode", int),
        "distance_scale_m": typed("distance_scale_m", float),
        "timing_budget_us": typed("timing_budget_us", int),
        "inter_measurement_period_ms": typed(
            "inter_measurement_period_ms",
            int,
        ),
        "min_valid_distance_m": typed("min_valid_distance_m", float),
        "max_valid_distance_m": typed("max_valid_distance_m", float),
        "filter_window_size": typed("filter_window_size", int),
        "update_rate_hz": typed("update_rate_hz", float),
        "field_of_view_rad": typed("field_of_view_rad", float),
    }

    left_sensor = Node(
        package="wall_distance_sensor",
        executable="vl53l1x_range_node",
        name="left_wall_tof",
        output="screen",
        parameters=[{
            **common_sensor_parameters,
            "i2c_bus": typed("left_i2c_bus", int),
            "address": typed("left_address", int),
            "frame_id": LaunchConfiguration("left_frame_id"),
            "range_topic": LaunchConfiguration("left_range_topic"),
            "mock_distance_m": typed("mock_left_distance_m", float),
        }],
    )

    right_sensor = Node(
        package="wall_distance_sensor",
        executable="vl53l1x_range_node",
        name="right_wall_tof",
        output="screen",
        parameters=[{
            **common_sensor_parameters,
            "i2c_bus": typed("right_i2c_bus", int),
            "address": typed("right_address", int),
            "frame_id": LaunchConfiguration("right_frame_id"),
            "range_topic": LaunchConfiguration("right_range_topic"),
            "mock_distance_m": typed("mock_right_distance_m", float),
        }],
    )

    aggregator = Node(
        package="wall_distance_sensor",
        executable="wall_distance_aggregator_node",
        name="wall_distance_aggregator",
        output="screen",
        parameters=[{
            "left_range_topic": LaunchConfiguration("left_range_topic"),
            "right_range_topic": LaunchConfiguration("right_range_topic"),
            "distance_angle_topic": LaunchConfiguration("distance_angle_topic"),
            "measurement_json_topic": LaunchConfiguration(
                "measurement_json_topic"
            ),
            "measurement_frame_id": LaunchConfiguration("measurement_frame_id"),
            "sensor_separation_m": typed("sensor_separation_m", float),
            "safe_distance_m": typed("safe_distance_m", float),
            "input_timeout_s": typed("input_timeout_s", float),
            "max_pair_skew_s": typed("max_pair_skew_s", float),
            "update_rate_hz": typed("update_rate_hz", float),
        }],
    )

    return LaunchDescription([
        *declarations,
        left_sensor,
        right_sensor,
        aggregator,
    ])
