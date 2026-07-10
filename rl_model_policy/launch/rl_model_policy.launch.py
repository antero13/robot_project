from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    speed_scale = LaunchConfiguration('speed_scale')
    dry_run = LaunchConfiguration('dry_run')
    gripper_enabled = LaunchConfiguration('gripper_enabled')
    gripper_type = LaunchConfiguration('gripper_type')
    gripper_servo_id = LaunchConfiguration('gripper_servo_id')
    gripper_open_position = LaunchConfiguration('gripper_open_position')
    gripper_closed_position = LaunchConfiguration('gripper_closed_position')
    gripper_move_duration_s = LaunchConfiguration('gripper_move_duration_s')
    grab_center_tolerance = LaunchConfiguration('grab_center_tolerance')
    grab_area_ratio = LaunchConfiguration('grab_area_ratio')
    final_forward_linear_x = LaunchConfiguration('final_forward_linear_x')
    final_forward_duration_s = LaunchConfiguration('final_forward_duration_s')
    grab_duration_s = LaunchConfiguration('grab_duration_s')
    stop_after_grab = LaunchConfiguration('stop_after_grab')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_manager'),
                'models',
                'rl_avoid_search_best.pt',
            ]),
            description='Path to the trained skrl best_agent.pt file.',
        ),
        DeclareLaunchArgument(
            'speed_scale',
            default_value='0.50',
            description='Scale applied to learned linear/angular velocity commands.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, publish state but do not publish cmd_vel.',
        ),
        DeclareLaunchArgument('gripper_enabled', default_value='true'),
        DeclareLaunchArgument('gripper_type', default_value='bus'),
        DeclareLaunchArgument('gripper_servo_id', default_value='1'),
        DeclareLaunchArgument('gripper_open_position', default_value='1000'),
        DeclareLaunchArgument('gripper_closed_position', default_value='250'),
        DeclareLaunchArgument('gripper_move_duration_s', default_value='0.5'),
        DeclareLaunchArgument('grab_center_tolerance', default_value='0.12'),
        DeclareLaunchArgument('grab_area_ratio', default_value='0.50'),
        DeclareLaunchArgument('final_forward_linear_x', default_value='0.06'),
        DeclareLaunchArgument('final_forward_duration_s', default_value='1.6'),
        DeclareLaunchArgument('grab_duration_s', default_value='1.0'),
        DeclareLaunchArgument('stop_after_grab', default_value='true'),
        Node(
            package='rl_model_policy',
            executable='rl_model_policy',
            name='rl_model_policy',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'cmd_vel_topic': '/cmd_vel',
                'target_object_topic': '/target_object',
                'avoid_object_topic': '/avoid_object',
                'avoid_objects_topic': '/avoid_objects',
                'control_topic': '/rl_model_policy_control',
                'state_topic': '/rl_model_policy_state',

                'active_on_start': False,
                'dry_run': dry_run,
                'timer_rate_hz': 20.0,
                'target_timeout_s': 0.5,
                'avoid_timeout_s': 0.5,
                'episode_length_s': 18.0,

                'avoid_area_ratio': 0.20,
                'avoid_center_band': 0.75,
                'avoid_center_corridor': 0.30,
                'avoid_vfh_center_weight': 2.0,
                'avoid_only_if_closer_than_target': False,
                'avoid_closer_ratio': 0.85,

                # These match the training environment before speed_scale.
                'max_forward_speed': 0.20,
                'max_reverse_speed': 0.05,
                'max_angular_speed': 0.80,
                'speed_scale': speed_scale,
                'max_linear_action_delta': 0.25,
                'max_angular_action_delta': 0.16,
                'action_filter_alpha': 0.60,
                'publish_stop_when_inactive': True,

                'gripper_enabled': ParameterValue(gripper_enabled, value_type=bool),
                'gripper_type': gripper_type,
                'gripper_servo_id': ParameterValue(gripper_servo_id, value_type=int),
                'gripper_open_position': ParameterValue(gripper_open_position, value_type=int),
                'gripper_closed_position': ParameterValue(gripper_closed_position, value_type=int),
                'gripper_move_duration_s': ParameterValue(gripper_move_duration_s, value_type=float),
                'grab_center_tolerance': ParameterValue(grab_center_tolerance, value_type=float),
                'grab_area_ratio': ParameterValue(grab_area_ratio, value_type=float),
                'final_forward_linear_x': ParameterValue(final_forward_linear_x, value_type=float),
                'final_forward_duration_s': ParameterValue(final_forward_duration_s, value_type=float),
                'grab_duration_s': ParameterValue(grab_duration_s, value_type=float),
                'stop_after_grab': ParameterValue(stop_after_grab, value_type=bool),
            }],
        ),
    ])
