from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/ros_robot_controller/imu_raw',
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('linear_scale', default_value='1.0'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('gyro_calibration_duration_s', default_value='2.0'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='robot_pose_tracker',
            executable='robot_pose_tracker',
            name='robot_pose_tracker',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'initial_x': ParameterValue(
                    LaunchConfiguration('initial_x'),
                    value_type=float,
                ),
                'initial_y': ParameterValue(
                    LaunchConfiguration('initial_y'),
                    value_type=float,
                ),
                'initial_yaw_deg': ParameterValue(
                    LaunchConfiguration('initial_yaw_deg'),
                    value_type=float,
                ),
                'linear_scale': ParameterValue(
                    LaunchConfiguration('linear_scale'),
                    value_type=float,
                ),
                'imu_yaw_sign': ParameterValue(
                    LaunchConfiguration('imu_yaw_sign'),
                    value_type=float,
                ),
                'gyro_calibration_duration_s': ParameterValue(
                    LaunchConfiguration('gyro_calibration_duration_s'),
                    value_type=float,
                ),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'),
                    value_type=bool,
                ),
            }],
        ),
    ])
