from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_pose_tracker')
    tracker_launch = PathJoinSubstitution(
        [package_share, 'launch', 'robot_pose_tracker.launch.py']
    )
    rviz_config = PathJoinSubstitution(
        [package_share, 'config', 'arena_pose.rviz']
    )

    start_tracker = LaunchConfiguration('start_tracker')
    start_rviz = LaunchConfiguration('start_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('start_tracker', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            condition=IfCondition(start_tracker),
            launch_arguments={
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw_deg': LaunchConfiguration('initial_yaw_deg'),
                'linear_scale': LaunchConfiguration('linear_scale'),
                'imu_yaw_sign': LaunchConfiguration('imu_yaw_sign'),
                'gyro_calibration_duration_s': LaunchConfiguration(
                    'gyro_calibration_duration_s'
                ),
            }.items(),
        ),
        Node(
            package='robot_pose_tracker',
            executable='arena_visualizer',
            name='arena_visualizer',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='arena_pose_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(start_rviz),
            output='screen',
        ),
    ])
