from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'mission_control_topic': '/mission_control',
                'mission_state_topic': '/mission_state',
                'target_object_topic': '/target_object',
                'pwm_servo_topic': '/ros_robot_controller/pwm_servo/set_state',
                'bus_servo_topic': '/ros_robot_controller/bus_servo/set_state',
                'timer_rate_hz': 20.0,

                'leave_start_linear_x': 0.08,
                'leave_start_angular_z': 0.0,
                'leave_start_duration_s': 1.5,
                'search_linear_x': 0.06,
                'search_forward_angular_z': 0.10,
                'search_forward_duration_s': 1.4,
                'search_angular_z': 0.35,
                'search_turn_duration_s': 0.9,
                'search_turn_direction': 1.0,
                'search_alternate_turn_direction': False,
                'search_duration_s': 150.0,
                'approach_max_linear_x': 0.10,
                'approach_min_linear_x': 0.03,
                'approach_angular_gain': 0.35,
                'approach_max_angular_z': 0.20,
                'center_tolerance': 0.18,
                'grab_area_ratio': 0.10,
                'target_timeout_s': 1.5,
                'back_out_linear_x': -0.08,

                'grab_duration_s': 1.0,
                'back_out_duration_s': 1.5,

                'gripper_enabled': False,
                'gripper_type': 'pwm',
                'gripper_servo_id': 1,
                'gripper_open_position': 1500,
                'gripper_closed_position': 1000,
                'gripper_move_duration_s': 0.5,
            }],
        ),
    ])
