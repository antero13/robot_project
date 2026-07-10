from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    speed_scale = LaunchConfiguration('speed_scale')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='mission_manager/models/rl_avoid_search_best.pt',
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
            }],
        ),
    ])
