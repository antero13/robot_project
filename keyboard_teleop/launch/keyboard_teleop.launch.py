from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_teleop',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            prefix='',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'linear_speed': 0.10,
                'angular_speed': 0.50,
                'linear_step': 0.02,
                'angular_step': 0.10,
                'publish_rate_hz': 20.0,
                'key_timeout_s': 0.0,
                'bus_servo_topic': '/ros_robot_controller/bus_servo/set_state',
                'bus_servo_id': 1,
                'bus_servo_min_position': 0,
                'bus_servo_max_position': 1000,
                'bus_servo_center_position': 500,
                'bus_servo_step': 50,
                'bus_servo_duration_s': 0.4,
            }],
        ),
    ])
