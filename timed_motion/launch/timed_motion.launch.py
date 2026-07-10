from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='timed_motion',
            executable='timed_motion_node',
            name='timed_motion',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'drive_distance_topic': '/drive_distance',
                'turn_angle_topic': '/turn_angle',
                'linear_speed_mps': 0.10,
                'angular_speed_radps': 0.50,
                'distance_scale': 1.0,
                'angle_scale': 1.0,
                'publish_rate_hz': 20.0,
                'stop_publish_count': 5,
            }],
        ),
    ])
