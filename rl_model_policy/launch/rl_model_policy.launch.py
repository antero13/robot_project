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
    target_timeout_s = LaunchConfiguration('target_timeout_s')
    target_visibility_topic = LaunchConfiguration('target_visibility_topic')
    target_center_y_topic = LaunchConfiguration('target_center_y_topic')
    target_confirmation_window = LaunchConfiguration('target_confirmation_window')
    target_confirmation_min_detections = LaunchConfiguration(
        'target_confirmation_min_detections'
    )
    target_activation_center_y_min = LaunchConfiguration(
        'target_activation_center_y_min'
    )
    target_bearing_prediction_enabled = LaunchConfiguration(
        'target_bearing_prediction_enabled'
    )
    odometry_topic = LaunchConfiguration('odometry_topic')
    pose_timeout_s = LaunchConfiguration('pose_timeout_s')
    pose_observation_enabled = LaunchConfiguration('pose_observation_enabled')
    arena_half_extent_m = LaunchConfiguration('arena_half_extent_m')
    pose_bounds_tolerance_m = LaunchConfiguration('pose_bounds_tolerance_m')
    camera_horizontal_fov_deg = LaunchConfiguration('camera_horizontal_fov_deg')
    coverage_enabled = LaunchConfiguration('coverage_enabled')
    coverage_min_x = LaunchConfiguration('coverage_min_x')
    coverage_max_x = LaunchConfiguration('coverage_max_x')
    coverage_main_road_y = LaunchConfiguration('coverage_main_road_y')
    coverage_scan_end_y = LaunchConfiguration('coverage_scan_end_y')
    coverage_lane_spacing = LaunchConfiguration('coverage_lane_spacing')
    coverage_scan_speed = LaunchConfiguration('coverage_scan_speed')
    coverage_transit_speed = LaunchConfiguration('coverage_transit_speed')
    coverage_return_speed = LaunchConfiguration('coverage_return_speed')
    coverage_waypoint_tolerance = LaunchConfiguration('coverage_waypoint_tolerance')
    coverage_turn_in_place_threshold = LaunchConfiguration(
        'coverage_turn_in_place_threshold'
    )
    coverage_max_angular_speed = LaunchConfiguration(
        'coverage_max_angular_speed'
    )
    coverage_avoid_angular_speed = LaunchConfiguration(
        'coverage_avoid_angular_speed'
    )
    coverage_avoid_linear_scale = LaunchConfiguration(
        'coverage_avoid_linear_scale'
    )
    coverage_rejoin_speed = LaunchConfiguration('coverage_rejoin_speed')
    coverage_reacquire_duration_s = LaunchConfiguration('coverage_reacquire_duration_s')
    coverage_reacquire_reverse_after_s = LaunchConfiguration(
        'coverage_reacquire_reverse_after_s'
    )
    coverage_reacquire_angular_z = LaunchConfiguration(
        'coverage_reacquire_angular_z'
    )
    gripper_enabled = LaunchConfiguration('gripper_enabled')
    gripper_type = LaunchConfiguration('gripper_type')
    gripper_servo_id = LaunchConfiguration('gripper_servo_id')
    gripper_open_position = LaunchConfiguration('gripper_open_position')
    gripper_closed_position = LaunchConfiguration('gripper_closed_position')
    gripper_move_duration_s = LaunchConfiguration('gripper_move_duration_s')
    grab_center_tolerance = LaunchConfiguration('grab_center_tolerance')
    grab_area_ratio = LaunchConfiguration('grab_area_ratio')
    grab_detection_timeout_s = LaunchConfiguration('grab_detection_timeout_s')
    final_forward_linear_x = LaunchConfiguration('final_forward_linear_x')
    final_forward_duration_s = LaunchConfiguration('final_forward_duration_s')
    grab_duration_s = LaunchConfiguration('grab_duration_s')
    stop_after_grab = LaunchConfiguration('stop_after_grab')
    full_mission_enabled = LaunchConfiguration('full_mission_enabled')
    mission_duration_s = LaunchConfiguration('mission_duration_s')
    force_return_remaining_s = LaunchConfiguration('force_return_remaining_s')
    storage_capacity = LaunchConfiguration('storage_capacity')
    target_object_count = LaunchConfiguration('target_object_count')
    storage_main_road_y = LaunchConfiguration('storage_main_road_y')
    storage_staging_x = LaunchConfiguration('storage_staging_x')
    storage_staging_y = LaunchConfiguration('storage_staging_y')
    storage_exit_y = LaunchConfiguration('storage_exit_y')
    storage_center_x = LaunchConfiguration('storage_center_x')
    storage_center_y = LaunchConfiguration('storage_center_y')
    storage_entry_yaw_deg = LaunchConfiguration('storage_entry_yaw_deg')
    storage_return_speed = LaunchConfiguration('storage_return_speed')
    storage_entry_speed = LaunchConfiguration('storage_entry_speed')
    storage_exit_reverse_speed = LaunchConfiguration('storage_exit_reverse_speed')
    storage_entry_tolerance = LaunchConfiguration('storage_entry_tolerance')

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
            default_value='0.75',
            description='Scale applied to learned linear/angular velocity commands.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, publish state but do not publish cmd_vel.',
        ),
        DeclareLaunchArgument(
            'target_timeout_s',
            default_value='1.0',
            description='Keep tracking the last target through short YOLO detection gaps.',
        ),
        DeclareLaunchArgument(
            'target_bearing_prediction_enabled',
            default_value='true',
            description='Project target image x from odometry during short detection gaps.',
        ),
        DeclareLaunchArgument('odometry_topic', default_value='/odom'),
        DeclareLaunchArgument('pose_timeout_s', default_value='0.5'),
        DeclareLaunchArgument(
            'pose_observation_enabled',
            default_value='false',
            description='Use pose/IMU policy inputs and yaw-based target prediction.',
        ),
        DeclareLaunchArgument('arena_half_extent_m', default_value='2.0'),
        DeclareLaunchArgument('pose_bounds_tolerance_m', default_value='0.25'),
        DeclareLaunchArgument('camera_horizontal_fov_deg', default_value='80.0'),
        DeclareLaunchArgument(
            'coverage_enabled',
            default_value='true',
            description='Use odometry-based lane coverage while no target is visible.',
        ),
        DeclareLaunchArgument('coverage_min_x', default_value='-0.75'),
        DeclareLaunchArgument('coverage_max_x', default_value='1.25'),
        DeclareLaunchArgument('coverage_main_road_y', default_value='-1.3343'),
        DeclareLaunchArgument('coverage_scan_end_y', default_value='1.0'),
        DeclareLaunchArgument('coverage_lane_spacing', default_value='1.0'),
        DeclareLaunchArgument('coverage_scan_speed', default_value='0.24'),
        DeclareLaunchArgument('coverage_transit_speed', default_value='0.30'),
        DeclareLaunchArgument('coverage_return_speed', default_value='0.24'),
        DeclareLaunchArgument('coverage_waypoint_tolerance', default_value='0.10'),
        DeclareLaunchArgument(
            'coverage_turn_in_place_threshold',
            default_value='0.65',
        ),
        DeclareLaunchArgument(
            'target_visibility_topic',
            default_value='/target_visible',
        ),
        DeclareLaunchArgument(
            'target_center_y_topic',
            default_value='/target_center_y',
        ),
        DeclareLaunchArgument(
            'target_activation_center_y_min',
            default_value='0.30',
        ),
        DeclareLaunchArgument(
            'target_confirmation_window',
            default_value='5',
        ),
        DeclareLaunchArgument(
            'target_confirmation_min_detections',
            default_value='3',
        ),
        DeclareLaunchArgument('coverage_max_angular_speed', default_value='1.00'),
        DeclareLaunchArgument('coverage_avoid_angular_speed', default_value='0.45'),
        DeclareLaunchArgument('coverage_avoid_linear_scale', default_value='0.70'),
        DeclareLaunchArgument('coverage_rejoin_speed', default_value='0.20'),
        DeclareLaunchArgument('coverage_reacquire_duration_s', default_value='1.5'),
        DeclareLaunchArgument(
            'coverage_reacquire_reverse_after_s',
            default_value='0.75',
        ),
        DeclareLaunchArgument('coverage_reacquire_angular_z', default_value='0.35'),
        DeclareLaunchArgument('full_mission_enabled', default_value='true'),
        DeclareLaunchArgument('mission_duration_s', default_value='180.0'),
        DeclareLaunchArgument('force_return_remaining_s', default_value='30.0'),
        DeclareLaunchArgument('storage_capacity', default_value='4'),
        DeclareLaunchArgument('target_object_count', default_value='7'),
        DeclareLaunchArgument('storage_main_road_y', default_value='-1.3343'),
        DeclareLaunchArgument('storage_staging_x', default_value='-1.75'),
        DeclareLaunchArgument('storage_staging_y', default_value='-1.25'),
        DeclareLaunchArgument('storage_exit_y', default_value='-1.0'),
        DeclareLaunchArgument('storage_center_x', default_value='-1.75'),
        DeclareLaunchArgument('storage_center_y', default_value='-1.75'),
        DeclareLaunchArgument('storage_entry_yaw_deg', default_value='-90.0'),
        DeclareLaunchArgument('storage_return_speed', default_value='0.25'),
        DeclareLaunchArgument('storage_entry_speed', default_value='0.12'),
        DeclareLaunchArgument('storage_exit_reverse_speed', default_value='0.16'),
        DeclareLaunchArgument('storage_entry_tolerance', default_value='0.04'),
        DeclareLaunchArgument('gripper_enabled', default_value='true'),
        DeclareLaunchArgument('gripper_type', default_value='bus'),
        DeclareLaunchArgument('gripper_servo_id', default_value='1'),
        DeclareLaunchArgument('gripper_open_position', default_value='1000'),
        DeclareLaunchArgument('gripper_closed_position', default_value='300'),
        DeclareLaunchArgument('gripper_move_duration_s', default_value='0.5'),
        DeclareLaunchArgument('grab_center_tolerance', default_value='0.18'),
        DeclareLaunchArgument('grab_area_ratio', default_value='0.70'),
        DeclareLaunchArgument('grab_detection_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('final_forward_linear_x', default_value='0.20'),
        DeclareLaunchArgument('final_forward_duration_s', default_value='1.2'),
        DeclareLaunchArgument('grab_duration_s', default_value='1.0'),
        DeclareLaunchArgument('stop_after_grab', default_value='false'),
        Node(
            package='rl_model_policy',
            executable='rl_model_policy',
            name='rl_model_policy',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'cmd_vel_topic': '/cmd_vel',
                'target_object_topic': '/target_object',
                'target_visibility_topic': target_visibility_topic,
                'target_center_y_topic': target_center_y_topic,
                'avoid_object_topic': '/avoid_object',
                'avoid_objects_topic': '/avoid_objects',
                'control_topic': '/rl_model_policy_control',
                'state_topic': '/rl_model_policy_state',
                'odometry_topic': odometry_topic,

                'active_on_start': False,
                'dry_run': dry_run,
                'timer_rate_hz': 20.0,
                'target_timeout_s': ParameterValue(target_timeout_s, value_type=float),
                'target_confirmation_window': ParameterValue(
                    target_confirmation_window,
                    value_type=int,
                ),
                'target_confirmation_min_detections': ParameterValue(
                    target_confirmation_min_detections,
                    value_type=int,
                ),
                'target_activation_center_y_min': ParameterValue(
                    target_activation_center_y_min,
                    value_type=float,
                ),
                'target_bearing_prediction_enabled': ParameterValue(
                    target_bearing_prediction_enabled,
                    value_type=bool,
                ),
                'avoid_timeout_s': 0.25,
                'episode_length_s': 18.0,
                'pose_timeout_s': ParameterValue(pose_timeout_s, value_type=float),
                'pose_observation_enabled': ParameterValue(
                    pose_observation_enabled,
                    value_type=bool,
                ),
                'arena_half_extent_m': ParameterValue(arena_half_extent_m, value_type=float),
                'pose_bounds_tolerance_m': ParameterValue(
                    pose_bounds_tolerance_m,
                    value_type=float,
                ),
                'camera_horizontal_fov_deg': ParameterValue(
                    camera_horizontal_fov_deg,
                    value_type=float,
                ),
                'coverage_enabled': ParameterValue(coverage_enabled, value_type=bool),
                'coverage_min_x': ParameterValue(coverage_min_x, value_type=float),
                'coverage_max_x': ParameterValue(coverage_max_x, value_type=float),
                'coverage_main_road_y': ParameterValue(
                    coverage_main_road_y,
                    value_type=float,
                ),
                'coverage_scan_end_y': ParameterValue(
                    coverage_scan_end_y,
                    value_type=float,
                ),
                'coverage_lane_spacing': ParameterValue(
                    coverage_lane_spacing,
                    value_type=float,
                ),
                'coverage_scan_speed': ParameterValue(
                    coverage_scan_speed,
                    value_type=float,
                ),
                'coverage_transit_speed': ParameterValue(
                    coverage_transit_speed,
                    value_type=float,
                ),
                'coverage_return_speed': ParameterValue(
                    coverage_return_speed,
                    value_type=float,
                ),
                'coverage_waypoint_tolerance': ParameterValue(
                    coverage_waypoint_tolerance,
                    value_type=float,
                ),
                'coverage_turn_in_place_threshold': ParameterValue(
                    coverage_turn_in_place_threshold,
                    value_type=float,
                ),
                'coverage_max_angular_speed': ParameterValue(
                    coverage_max_angular_speed,
                    value_type=float,
                ),
                'coverage_avoid_angular_speed': ParameterValue(
                    coverage_avoid_angular_speed,
                    value_type=float,
                ),
                'coverage_avoid_linear_scale': ParameterValue(
                    coverage_avoid_linear_scale,
                    value_type=float,
                ),
                'coverage_rejoin_speed': ParameterValue(
                    coverage_rejoin_speed,
                    value_type=float,
                ),
                'coverage_reacquire_duration_s': ParameterValue(
                    coverage_reacquire_duration_s,
                    value_type=float,
                ),
                'coverage_reacquire_reverse_after_s': ParameterValue(
                    coverage_reacquire_reverse_after_s,
                    value_type=float,
                ),
                'coverage_reacquire_angular_z': ParameterValue(
                    coverage_reacquire_angular_z,
                    value_type=float,
                ),

                'avoid_area_ratio': 0.42,
                'avoid_center_band': 0.75,
                'avoid_center_corridor': 0.15,
                'avoid_vfh_center_weight': 0.5,
                'avoid_only_if_closer_than_target': False,
                'avoid_closer_ratio': 0.85,

                # These match the training environment before speed_scale.
                'max_forward_speed': 0.20,
                'max_reverse_speed': 0.05,
                'max_angular_speed': 0.80,
                'speed_scale': speed_scale,
                'max_linear_action_delta': 0.25,
                'max_angular_action_delta': 0.08,
                'action_filter_alpha': 0.55,
                'publish_stop_when_inactive': True,
                'full_mission_enabled': ParameterValue(
                    full_mission_enabled,
                    value_type=bool,
                ),
                'mission_duration_s': ParameterValue(
                    mission_duration_s,
                    value_type=float,
                ),
                'force_return_remaining_s': ParameterValue(
                    force_return_remaining_s,
                    value_type=float,
                ),
                'storage_capacity': ParameterValue(storage_capacity, value_type=int),
                'target_object_count': ParameterValue(
                    target_object_count,
                    value_type=int,
                ),
                'storage_main_road_y': ParameterValue(
                    storage_main_road_y,
                    value_type=float,
                ),
                'storage_staging_x': ParameterValue(
                    storage_staging_x,
                    value_type=float,
                ),
                'storage_staging_y': ParameterValue(
                    storage_staging_y,
                    value_type=float,
                ),
                'storage_exit_y': ParameterValue(
                    storage_exit_y,
                    value_type=float,
                ),
                'storage_center_x': ParameterValue(
                    storage_center_x,
                    value_type=float,
                ),
                'storage_center_y': ParameterValue(
                    storage_center_y,
                    value_type=float,
                ),
                'storage_entry_yaw_deg': ParameterValue(
                    storage_entry_yaw_deg,
                    value_type=float,
                ),
                'storage_return_speed': ParameterValue(
                    storage_return_speed,
                    value_type=float,
                ),
                'storage_entry_speed': ParameterValue(
                    storage_entry_speed,
                    value_type=float,
                ),
                'storage_exit_reverse_speed': ParameterValue(
                    storage_exit_reverse_speed,
                    value_type=float,
                ),
                'storage_entry_tolerance': ParameterValue(
                    storage_entry_tolerance,
                    value_type=float,
                ),

                'gripper_enabled': ParameterValue(gripper_enabled, value_type=bool),
                'gripper_type': gripper_type,
                'gripper_servo_id': ParameterValue(gripper_servo_id, value_type=int),
                'gripper_open_position': ParameterValue(gripper_open_position, value_type=int),
                'gripper_closed_position': ParameterValue(gripper_closed_position, value_type=int),
                'gripper_move_duration_s': ParameterValue(gripper_move_duration_s, value_type=float),
                'grab_center_tolerance': ParameterValue(grab_center_tolerance, value_type=float),
                'grab_area_ratio': ParameterValue(grab_area_ratio, value_type=float),
                'grab_detection_timeout_s': ParameterValue(
                    grab_detection_timeout_s,
                    value_type=float,
                ),
                'final_forward_linear_x': ParameterValue(final_forward_linear_x, value_type=float),
                'final_forward_duration_s': ParameterValue(final_forward_duration_s, value_type=float),
                'grab_duration_s': ParameterValue(grab_duration_s, value_type=float),
                'stop_after_grab': ParameterValue(stop_after_grab, value_type=bool),
            }],
        ),
    ])
