from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_centering',
            executable='target_centering',
            name='target_centering',
            output='screen',
            parameters=[{
                'target_topic': '/target_object',
                'avoid_topic': '/avoid_object',
                'cmd_vel_topic': '/cmd_vel',
                'center_tolerance': 0.08,
                'angular_kp': 0.8,
                'angular_kd': 0.08,
                'max_angular_z': 0.45,
                'min_angular_z': 0.12,
                'target_timeout_s': 0.5,
                'filter_window_size': 5,
                'min_consecutive_detections': 2,
                'lost_hold_s': 0.2,
                'avoid_enabled': True,
                'avoid_timeout_s': 0.5,
                'avoid_area_ratio': 0.60,
                'avoid_center_band': 0.65,
                'avoid_angular_z': 0.35,
                'avoid_only_if_closer_than_target': True,
                'avoid_closer_ratio': 1.05,
                'search_when_lost': False,
                'search_angular_z': 0.25,
                'publish_rate_hz': 20.0,
            }],
        ),
    ])
