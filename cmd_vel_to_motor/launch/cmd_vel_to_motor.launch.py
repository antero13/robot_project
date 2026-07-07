from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_to_motor',
            executable='cmd_vel_to_motor',
            name='cmd_vel_to_motor',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'motor_topic': '/ros_robot_controller/set_motor',
                'wheel_radius_m': 0.03,
                'wheel_separation_m': 0.18,
                'left_motor_ids': [4, 3],
                'right_motor_ids': [2, 1],
                'left_motor_signs': [1.0, 1.0],
                'right_motor_signs': [-1.0, -1.0],
                'max_rps': 2.0,
                'publish_rate_hz': 20.0,
                'command_timeout_s': 0.5,
            }],
        ),
    ])
