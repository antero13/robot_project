from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def arg(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    camera_index = int(arg(context, 'camera_index'))
    camera_width = int(arg(context, 'camera_width'))
    camera_height = int(arg(context, 'camera_height'))
    camera_fps = float(arg(context, 'camera_fps'))
    time_per_frame_denominator = max(1, int(round(camera_fps)))
    confidence = float(arg(context, 'confidence'))
    tracker_enabled = as_bool(arg(context, 'tracker_enabled'))
    tracker_persist = as_bool(arg(context, 'tracker_persist'))
    stable_tracking_enabled = as_bool(arg(context, 'stable_tracking_enabled'))
    publish_raw = as_bool(arg(context, 'publish_raw'))
    publish_annotated = as_bool(arg(context, 'publish_annotated'))
    gripper_enabled = as_bool(arg(context, 'gripper_enabled'))
    gripper_servo_id = int(arg(context, 'gripper_servo_id'))
    gripper_open_position = int(arg(context, 'gripper_open_position'))
    gripper_closed_position = int(arg(context, 'gripper_closed_position'))
    final_forward_linear_x = float(arg(context, 'final_forward_linear_x'))
    final_forward_duration_s = float(arg(context, 'final_forward_duration_s'))

    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            namespace='camera',
            output='screen',
            parameters=[{
                'video_device': arg(context, 'video_device'),
                'image_size': [camera_width, camera_height],
                'time_per_frame': [1, time_per_frame_denominator],
                'pixel_format': arg(context, 'pixel_format'),
                'output_encoding': arg(context, 'output_encoding'),
                'power_line_frequency': int(arg(context, 'power_line_frequency')),
                'auto_exposure': int(arg(context, 'auto_exposure')),
                'exposure_time_absolute': int(arg(context, 'exposure_time_absolute')),
                'gain': int(arg(context, 'gain')),
            }],
        ),
        Node(
            package='ros2_yolo_detector',
            executable='yolo_camera_node',
            name='yolo_camera_node',
            output='screen',
            parameters=[{
                'model_path': arg(context, 'model_path'),
                'input_mode': 'topic',
                'image_topic': arg(context, 'image_topic'),
                'camera_index': camera_index,
                'camera_width': camera_width,
                'camera_height': camera_height,
                'camera_fps': camera_fps,
                'confidence': confidence,
                'tracker_enabled': tracker_enabled,
                'tracker_config': arg(context, 'tracker_config'),
                'tracker_persist': tracker_persist,
                'stable_tracking_enabled': stable_tracking_enabled,
                'stable_track_timeout_s': float(arg(context, 'stable_track_timeout_s')),
                'stable_track_iou_threshold': float(arg(context, 'stable_track_iou_threshold')),
                'stable_track_center_ratio': float(arg(context, 'stable_track_center_ratio')),
                'detections_topic': '/yolo/detections',
                'annotated_topic': '/yolo/annotated_image',
                'raw_topic': arg(context, 'raw_topic'),
                'publish_raw': publish_raw,
                'publish_annotated': publish_annotated,
            }],
        ),
        Node(
            package='ros2_yolo_detector',
            executable='detections_to_target_node',
            name='detections_to_target_node',
            output='screen',
            parameters=[{
                'detections_topic': '/yolo/detections',
                'target_topic': '/target_object',
                'target_label_topic': '/target_label',
                'avoid_topic': '/avoid_object',
                'avoid_label_topic': '/avoid_label',
                'avoid_objects_topic': '/avoid_objects',
                'target_lock_enabled': True,
                'target_lock_timeout_s': 0.7,
                'target_lock_iou_threshold': 0.20,
                'target_lock_x_margin': 0.30,
                'target_lock_y_margin': 0.20,
                'target_switch_y_margin': 0.12,
                'target_switch_score_margin': 0.25,
                'target_center_weight': 0.25,
                'avoid_target_iou_threshold': 0.35,
                'target_classes': arg(context, 'target_classes'),
                'avoid_classes': arg(context, 'avoid_classes'),
                'min_confidence': confidence,
                'image_width': float(camera_width),
                'image_height': float(camera_height),
            }],
        ),
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
                'avoid_object_topic': '/avoid_object',
                'avoid_objects_topic': '/avoid_objects',
                'timer_rate_hz': 20.0,
                'leave_start_linear_x': 0.08,
                'leave_start_duration_s': 1.5,
                'search_linear_x': 0.06,
                'search_forward_angular_z': 0.10,
                'search_forward_duration_s': 1.4,
                'search_angular_z': 0.35,
                'search_turn_duration_s': 0.9,
                'search_duration_s': 150.0,
                'approach_max_linear_x': 0.10,
                'approach_min_linear_x': 0.03,
                'approach_angular_gain': 0.8,
                'approach_max_angular_z': 0.45,
                'center_tolerance': 0.12,
                'grab_area_ratio': 0.50,
                'target_timeout_s': 0.5,
                'final_forward_linear_x': final_forward_linear_x,
                'final_forward_duration_s': final_forward_duration_s,
                'avoid_enabled': True,
                'avoid_timeout_s': 0.5,
                'avoid_area_ratio': 0.38,
                'avoid_center_band': 0.75,
                'avoid_center_corridor': 0.30,
                'avoid_path_margin': 0.30,
                'avoid_roi_enabled': True,
                'avoid_roi_left_near_x': -0.27,
                'avoid_roi_left_near_y': 0.80,
                'avoid_roi_left_far_x': -0.75,
                'avoid_roi_left_far_y': 0.42,
                'avoid_roi_right_near_x': 0.11,
                'avoid_roi_right_near_y': 0.80,
                'avoid_roi_right_far_x': 0.62,
                'avoid_roi_right_far_y': 0.42,
                'avoid_emergency_ratio': 0.68,
                'avoid_only_if_closer_than_target': True,
                'avoid_closer_ratio': 0.85,
                'avoid_turn_duration_s': 0.55,
                'avoid_turn_angular_z': 0.65,
                'avoid_forward_duration_s': 0.85,
                'avoid_forward_linear_x': 0.05,
                'avoid_forward_angular_z': 0.25,
                'avoid_turn_direction_sign': 1.0,
                'avoid_vfh_center_weight': 2.0,
                'avoid_vfh_target_weight': 0.60,
                'avoid_vfh_switch_penalty': 0.25,
                'avoid_direction_hold_s': 0.8,
                'avoid_ignore_near_target_enabled': True,
                'avoid_ignore_target_min_y': 0.35,
                'avoid_ignore_target_center_band': 0.25,
                'avoid_ignore_target_x_margin': 0.25,
                'avoid_ignore_target_y_margin': 0.20,
                'reacquire_duration_s': 3.0,
                'reacquire_angular_z': 0.30,
                'back_out_linear_x': -0.08,
                'grab_duration_s': 1.0,
                'back_out_duration_s': 1.5,
                'gripper_enabled': gripper_enabled,
                'gripper_type': arg(context, 'gripper_type'),
                'gripper_servo_id': gripper_servo_id,
                'gripper_open_position': gripper_open_position,
                'gripper_closed_position': gripper_closed_position,
                'gripper_move_duration_s': 0.5,
            }],
        ),
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ros2_yolo_detector'), 'models', 'best.pt']
            ),
        ),
        DeclareLaunchArgument('input_mode', default_value='topic'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0',
        ),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='bgr8'),
        DeclareLaunchArgument('power_line_frequency', default_value='2'),
        DeclareLaunchArgument('auto_exposure', default_value='1'),
        DeclareLaunchArgument('exposure_time_absolute', default_value='200'),
        DeclareLaunchArgument('gain', default_value='20'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        DeclareLaunchArgument('tracker_enabled', default_value='true'),
        DeclareLaunchArgument('tracker_config', default_value='bytetrack.yaml'),
        DeclareLaunchArgument('tracker_persist', default_value='true'),
        DeclareLaunchArgument('stable_tracking_enabled', default_value='true'),
        DeclareLaunchArgument('stable_track_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('stable_track_iou_threshold', default_value='0.15'),
        DeclareLaunchArgument('stable_track_center_ratio', default_value='0.75'),
        DeclareLaunchArgument('publish_raw', default_value='true'),
        DeclareLaunchArgument('publish_annotated', default_value='false'),
        DeclareLaunchArgument('raw_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('target_classes', default_value=''),
        DeclareLaunchArgument('avoid_classes', default_value=''),
        DeclareLaunchArgument('gripper_enabled', default_value='true'),
        DeclareLaunchArgument('gripper_type', default_value='bus'),
        DeclareLaunchArgument('gripper_servo_id', default_value='1'),
        DeclareLaunchArgument('gripper_open_position', default_value='500'),
        DeclareLaunchArgument('gripper_closed_position', default_value='750'),
        DeclareLaunchArgument('final_forward_linear_x', default_value='0.06'),
        DeclareLaunchArgument('final_forward_duration_s', default_value='1.6'),
        OpaqueFunction(function=launch_setup),
    ])
