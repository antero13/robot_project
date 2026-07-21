from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


AUTO_START_DELAY_S = 8.0


def generate_launch_description():
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    secondary_yolo_model_path = LaunchConfiguration("secondary_yolo_model_path")
    video_device = LaunchConfiguration("video_device")
    target_classes = LaunchConfiguration("target_classes")
    avoid_classes = LaunchConfiguration("avoid_classes")
    confidence = LaunchConfiguration("confidence")
    yolo_iou = LaunchConfiguration("yolo_iou")
    agnostic_nms = LaunchConfiguration("agnostic_nms")
    publish_annotated = LaunchConfiguration("publish_annotated")
    correction_enabled = LaunchConfiguration("correction_enabled")
    yolo_imgsz = LaunchConfiguration("yolo_imgsz")
    secondary_yolo_confidence = LaunchConfiguration("secondary_yolo_confidence")
    secondary_yolo_imgsz = LaunchConfiguration("secondary_yolo_imgsz")
    yolo_min_bbox_area_ratio = LaunchConfiguration("yolo_min_bbox_area_ratio")
    target_lock_enabled = LaunchConfiguration("target_lock_enabled")
    target_lock_timeout_s = LaunchConfiguration("target_lock_timeout_s")
    target_lock_iou_threshold = LaunchConfiguration("target_lock_iou_threshold")
    target_lock_center_distance = LaunchConfiguration("target_lock_center_distance")
    correction_backend = LaunchConfiguration("correction_backend")
    correction_device = LaunchConfiguration("correction_device")
    yolo_performance_log_interval_s = LaunchConfiguration(
        "yolo_performance_log_interval_s"
    )
    timer_rate_hz = LaunchConfiguration("timer_rate_hz")
    approach_center_tolerance = LaunchConfiguration("approach_center_tolerance")
    approach_max_linear_x = LaunchConfiguration("approach_max_linear_x")
    approach_min_linear_x = LaunchConfiguration("approach_min_linear_x")
    approach_angular_gain = LaunchConfiguration("approach_angular_gain")
    approach_max_angular_z = LaunchConfiguration("approach_max_angular_z")
    target_timeout_s = LaunchConfiguration("target_timeout_s")
    target_tracking_timeout_s = LaunchConfiguration("target_tracking_timeout_s")
    target_confirmation_window = LaunchConfiguration("target_confirmation_window")
    target_confirmation_min_detections = LaunchConfiguration(
        "target_confirmation_min_detections"
    )
    target_activation_center_y_min = LaunchConfiguration(
        "target_activation_center_y_min"
    )
    target_tracking_center_y_min = LaunchConfiguration("target_tracking_center_y_min")
    target_bearing_prediction_enabled = LaunchConfiguration(
        "target_bearing_prediction_enabled"
    )
    near_target_loss_enabled = LaunchConfiguration("near_target_loss_enabled")
    near_target_loss_margin = LaunchConfiguration("near_target_loss_margin")
    near_target_loss_timeout_s = LaunchConfiguration("near_target_loss_timeout_s")
    near_target_loss_min_missing_s = LaunchConfiguration(
        "near_target_loss_min_missing_s"
    )
    odometry_topic = LaunchConfiguration("odometry_topic")
    pose_timeout_s = LaunchConfiguration("pose_timeout_s")
    pose_observation_enabled = LaunchConfiguration("pose_observation_enabled")
    launch_pose_tracker = LaunchConfiguration("launch_pose_tracker")
    pose_x_correction_topic = LaunchConfiguration("pose_x_correction_topic")
    pose_y_correction_topic = LaunchConfiguration("pose_y_correction_topic")
    pose_yaw_correction_topic = LaunchConfiguration("pose_yaw_correction_topic")
    launch_wall_distance_sensor = LaunchConfiguration("launch_wall_distance_sensor")
    wall_driver_backend = LaunchConfiguration("wall_driver_backend")
    wall_left_i2c_bus = LaunchConfiguration("wall_left_i2c_bus")
    wall_right_i2c_bus = LaunchConfiguration("wall_right_i2c_bus")
    wall_left_address = LaunchConfiguration("wall_left_address")
    wall_right_address = LaunchConfiguration("wall_right_address")
    wall_ranging_mode = LaunchConfiguration("wall_ranging_mode")
    wall_distance_angle_topic = LaunchConfiguration("wall_distance_angle_topic")
    tof_wall_angle_sign = LaunchConfiguration("tof_wall_angle_sign")
    tof_validation_samples = LaunchConfiguration("tof_validation_samples")
    tof_max_valid_wall_angle_rad = LaunchConfiguration("tof_max_valid_wall_angle_rad")
    tof_max_angle_spread_rad = LaunchConfiguration("tof_max_angle_spread_rad")
    tof_max_distance_spread_m = LaunchConfiguration("tof_max_distance_spread_m")
    tof_alignment_watchdog_s = LaunchConfiguration("tof_alignment_watchdog_s")
    arena_half_extent_m = LaunchConfiguration("arena_half_extent_m")
    pose_bounds_tolerance_m = LaunchConfiguration("pose_bounds_tolerance_m")
    camera_horizontal_fov_deg = LaunchConfiguration("camera_horizontal_fov_deg")
    leave_start_enabled = LaunchConfiguration("leave_start_enabled")
    leave_start_distance_m = LaunchConfiguration("leave_start_distance_m")
    leave_start_speed = LaunchConfiguration("leave_start_speed")
    leave_start_target_yaw_deg = LaunchConfiguration("leave_start_target_yaw_deg")
    leave_start_heading_gain = LaunchConfiguration("leave_start_heading_gain")
    leave_start_max_angular_speed = LaunchConfiguration("leave_start_max_angular_speed")
    leave_start_heading_tolerance = LaunchConfiguration(
        "leave_start_heading_tolerance"
    )
    launch_object_mapper = LaunchConfiguration("launch_object_mapper")
    launch_status_gui = LaunchConfiguration("launch_status_gui")
    object_calibration_path = LaunchConfiguration("object_calibration_path")
    object_association_radius_m = LaunchConfiguration("object_association_radius_m")
    object_position_smoothing_alpha = LaunchConfiguration(
        "object_position_smoothing_alpha"
    )
    object_retention_s = LaunchConfiguration("object_retention_s")
    coverage_enabled = LaunchConfiguration("coverage_enabled")
    coverage_min_x = LaunchConfiguration("coverage_min_x")
    coverage_max_x = LaunchConfiguration("coverage_max_x")
    coverage_main_road_y = LaunchConfiguration("coverage_main_road_y")
    coverage_scan_end_y = LaunchConfiguration("coverage_scan_end_y")
    coverage_lane_spacing = LaunchConfiguration("coverage_lane_spacing")
    coverage_scan_speed = LaunchConfiguration("coverage_scan_speed")
    coverage_transit_speed = LaunchConfiguration("coverage_transit_speed")
    coverage_return_speed = LaunchConfiguration("coverage_return_speed")
    coverage_waypoint_tolerance = LaunchConfiguration("coverage_waypoint_tolerance")
    coverage_turn_in_place_threshold = LaunchConfiguration(
        "coverage_turn_in_place_threshold"
    )
    coverage_max_angular_speed = LaunchConfiguration("coverage_max_angular_speed")
    coverage_avoid_heading_tolerance = LaunchConfiguration(
        "coverage_avoid_heading_tolerance"
    )
    coverage_avoid_angular_speed = LaunchConfiguration("coverage_avoid_angular_speed")
    coverage_avoid_linear_scale = LaunchConfiguration("coverage_avoid_linear_scale")
    coverage_rejoin_speed = LaunchConfiguration("coverage_rejoin_speed")
    coverage_rejoin_coordinate_limit = LaunchConfiguration(
        "coverage_rejoin_coordinate_limit"
    )
    coverage_reacquire_duration_s = LaunchConfiguration("coverage_reacquire_duration_s")
    coverage_reacquire_reverse_after_s = LaunchConfiguration(
        "coverage_reacquire_reverse_after_s"
    )
    coverage_reacquire_angular_z = LaunchConfiguration("coverage_reacquire_angular_z")
    lane_tof_correction_enabled = LaunchConfiguration("lane_tof_correction_enabled")
    lane_tof_left_wall_x_m = LaunchConfiguration("lane_tof_left_wall_x_m")
    lane_tof_right_wall_x_m = LaunchConfiguration("lane_tof_right_wall_x_m")
    lane_tof_sensor_forward_offset_m = LaunchConfiguration(
        "lane_tof_sensor_forward_offset_m"
    )
    lane_tof_measurement_timeout_s = LaunchConfiguration(
        "lane_tof_measurement_timeout_s"
    )
    lane_tof_x_tolerance_m = LaunchConfiguration("lane_tof_x_tolerance_m")
    lane_tof_min_speed = LaunchConfiguration("lane_tof_min_speed")
    lane_tof_slowdown_distance_m = LaunchConfiguration("lane_tof_slowdown_distance_m")
    lane_tof_wall_angle_tolerance_rad = LaunchConfiguration(
        "lane_tof_wall_angle_tolerance_rad"
    )
    main_road_tof_correction_enabled = LaunchConfiguration(
        "main_road_tof_correction_enabled"
    )
    main_road_tof_south_wall_y_m = LaunchConfiguration(
        "main_road_tof_south_wall_y_m"
    )
    main_road_tof_sensor_forward_offset_m = LaunchConfiguration(
        "main_road_tof_sensor_forward_offset_m"
    )
    main_road_tof_measurement_timeout_s = LaunchConfiguration(
        "main_road_tof_measurement_timeout_s"
    )
    main_road_tof_y_tolerance_m = LaunchConfiguration(
        "main_road_tof_y_tolerance_m"
    )
    main_road_tof_min_speed = LaunchConfiguration("main_road_tof_min_speed")
    main_road_tof_slowdown_distance_m = LaunchConfiguration(
        "main_road_tof_slowdown_distance_m"
    )
    main_road_tof_angle_trigger_rad = LaunchConfiguration(
        "main_road_tof_angle_trigger_rad"
    )
    main_road_tof_angle_release_rad = LaunchConfiguration(
        "main_road_tof_angle_release_rad"
    )
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw_deg = LaunchConfiguration("initial_yaw_deg")
    pose_linear_scale = LaunchConfiguration("pose_linear_scale")
    imu_yaw_sign = LaunchConfiguration("imu_yaw_sign")
    gyro_calibration_duration_s = LaunchConfiguration("gyro_calibration_duration_s")
    dry_run = LaunchConfiguration("dry_run")
    auto_start = LaunchConfiguration("auto_start")
    gripper_enabled = LaunchConfiguration("gripper_enabled")
    gripper_type = LaunchConfiguration("gripper_type")
    gripper_servo_id = LaunchConfiguration("gripper_servo_id")
    gripper_open_position = LaunchConfiguration("gripper_open_position")
    gripper_closed_position = LaunchConfiguration("gripper_closed_position")
    gripper_move_duration_s = LaunchConfiguration("gripper_move_duration_s")
    grab_center_tolerance = LaunchConfiguration("grab_center_tolerance")
    grab_area_ratio = LaunchConfiguration("grab_area_ratio")
    grab_detection_timeout_s = LaunchConfiguration("grab_detection_timeout_s")
    final_forward_linear_x = LaunchConfiguration("final_forward_linear_x")
    final_forward_duration_s = LaunchConfiguration("final_forward_duration_s")
    grab_duration_s = LaunchConfiguration("grab_duration_s")
    stop_after_grab = LaunchConfiguration("stop_after_grab")
    full_mission_enabled = LaunchConfiguration("full_mission_enabled")
    mission_duration_s = LaunchConfiguration("mission_duration_s")
    force_return_remaining_s = LaunchConfiguration("force_return_remaining_s")
    storage_capacity = LaunchConfiguration("storage_capacity")
    target_object_count = LaunchConfiguration("target_object_count")
    storage_main_road_y = LaunchConfiguration("storage_main_road_y")
    storage_staging_x = LaunchConfiguration("storage_staging_x")
    storage_staging_y = LaunchConfiguration("storage_staging_y")
    storage_exit_x = LaunchConfiguration("storage_exit_x")
    storage_center_x = LaunchConfiguration("storage_center_x")
    storage_center_y = LaunchConfiguration("storage_center_y")
    storage_entry_yaw_deg = LaunchConfiguration("storage_entry_yaw_deg")
    storage_return_speed = LaunchConfiguration("storage_return_speed")
    storage_entry_speed = LaunchConfiguration("storage_entry_speed")
    storage_x_entry_speed = LaunchConfiguration("storage_x_entry_speed")
    storage_exit_reverse_speed = LaunchConfiguration("storage_exit_reverse_speed")
    storage_entry_dash_duration_s = LaunchConfiguration(
        "storage_entry_dash_duration_s"
    )
    storage_exit_dash_duration_s = LaunchConfiguration(
        "storage_exit_dash_duration_s"
    )
    storage_contact_settle_duration_s = LaunchConfiguration(
        "storage_contact_settle_duration_s"
    )
    storage_dash_heading_deg = LaunchConfiguration("storage_dash_heading_deg")
    storage_dash_heading_tolerance = LaunchConfiguration(
        "storage_dash_heading_tolerance"
    )
    storage_dash_max_angular_speed = LaunchConfiguration(
        "storage_dash_max_angular_speed"
    )
    storage_entry_tolerance = LaunchConfiguration("storage_entry_tolerance")
    storage_tof_correction_enabled = LaunchConfiguration(
        "storage_tof_correction_enabled"
    )
    storage_tof_left_wall_x_m = LaunchConfiguration("storage_tof_left_wall_x_m")
    storage_tof_bottom_wall_y_m = LaunchConfiguration("storage_tof_bottom_wall_y_m")
    storage_tof_sensor_forward_offset_m = LaunchConfiguration(
        "storage_tof_sensor_forward_offset_m"
    )
    storage_tof_measurement_timeout_s = LaunchConfiguration(
        "storage_tof_measurement_timeout_s"
    )
    storage_exit_tof_fallback_timeout_s = LaunchConfiguration(
        "storage_exit_tof_fallback_timeout_s"
    )
    storage_tof_xy_tolerance_m = LaunchConfiguration("storage_tof_xy_tolerance_m")
    storage_tof_min_speed = LaunchConfiguration("storage_tof_min_speed")
    storage_tof_slowdown_distance_m = LaunchConfiguration(
        "storage_tof_slowdown_distance_m"
    )
    storage_tof_wall_angle_tolerance_rad = LaunchConfiguration(
        "storage_tof_wall_angle_tolerance_rad"
    )
    storage_exit_tof_angle_trigger_rad = LaunchConfiguration(
        "storage_exit_tof_angle_trigger_rad"
    )
    storage_exit_tof_angle_release_rad = LaunchConfiguration(
        "storage_exit_tof_angle_release_rad"
    )

    controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("ros_robot_controller"),
            "launch",
            "ros_robot_controller.launch.xml",
            ]
        )
    )

    yolo_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("ros2_yolo_detector"),
            "launch",
            "v4l2_yolo_camera.launch.py",
            ]
        ),
        launch_arguments={
            "model_path": yolo_model_path,
            "secondary_model_path": secondary_yolo_model_path,
            "video_device": video_device,
            "target_classes": target_classes,
            "avoid_classes": avoid_classes,
            "confidence": confidence,
            "iou": yolo_iou,
            "agnostic_nms": agnostic_nms,
            "publish_annotated": publish_annotated,
            "correction_enabled": correction_enabled,
            "imgsz": yolo_imgsz,
            "secondary_confidence": secondary_yolo_confidence,
            "secondary_imgsz": secondary_yolo_imgsz,
            "min_bbox_area_ratio": yolo_min_bbox_area_ratio,
            "target_lock_enabled": target_lock_enabled,
            "target_lock_timeout_s": target_lock_timeout_s,
            "target_lock_iou_threshold": target_lock_iou_threshold,
            "target_lock_center_distance": target_lock_center_distance,
            "correction_backend": correction_backend,
            "correction_device": correction_device,
            "performance_log_interval_s": yolo_performance_log_interval_s,
        }.items(),
    )

    motor_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("cmd_vel_to_motor"),
            "launch",
            "cmd_vel_to_motor.launch.py",
            ]
        )
    )

    pose_tracker_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("robot_pose_tracker"),
            "launch",
            "robot_pose_tracker.launch.py",
            ]
        ),
        launch_arguments={
            "cmd_vel_topic": "/cmd_vel",
            "x_correction_topic": pose_x_correction_topic,
            "y_correction_topic": pose_y_correction_topic,
            "yaw_correction_topic": pose_yaw_correction_topic,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw_deg": initial_yaw_deg,
            "linear_scale": pose_linear_scale,
            "imu_yaw_sign": imu_yaw_sign,
            "gyro_calibration_duration_s": gyro_calibration_duration_s,
            "publish_tf": "true",
        }.items(),
        condition=IfCondition(launch_pose_tracker),
    )

    wall_distance_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("wall_distance_sensor"),
            "launch",
            "wall_distance_angle.launch.py",
            ]
        ),
        launch_arguments={
            "driver_backend": wall_driver_backend,
            "left_i2c_bus": wall_left_i2c_bus,
            "right_i2c_bus": wall_right_i2c_bus,
            "left_address": wall_left_address,
            "right_address": wall_right_address,
            "ranging_mode": wall_ranging_mode,
            "update_rate_hz": "20.0",
            "distance_angle_topic": wall_distance_angle_topic,
        }.items(),
        condition=IfCondition(launch_wall_distance_sensor),
    )

    mission_controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
            FindPackageShare("rl_model_policy"),
            "launch",
            "rl_model_policy.launch.py",
            ]
        ),
        launch_arguments={
            "timer_rate_hz": timer_rate_hz,
            "approach_center_tolerance": approach_center_tolerance,
            "approach_max_linear_x": approach_max_linear_x,
            "approach_min_linear_x": approach_min_linear_x,
            "approach_angular_gain": approach_angular_gain,
            "approach_max_angular_z": approach_max_angular_z,
            "target_timeout_s": target_timeout_s,
            "target_tracking_timeout_s": target_tracking_timeout_s,
            "target_confirmation_window": target_confirmation_window,
            "target_confirmation_min_detections": (target_confirmation_min_detections),
            "target_activation_center_y_min": target_activation_center_y_min,
            "target_tracking_center_y_min": target_tracking_center_y_min,
            "target_bearing_prediction_enabled": target_bearing_prediction_enabled,
            "near_target_loss_enabled": near_target_loss_enabled,
            "near_target_loss_margin": near_target_loss_margin,
            "near_target_loss_timeout_s": near_target_loss_timeout_s,
            "near_target_loss_min_missing_s": near_target_loss_min_missing_s,
            "dry_run": dry_run,
            "odometry_topic": odometry_topic,
            "pose_timeout_s": pose_timeout_s,
            "pose_observation_enabled": pose_observation_enabled,
            "arena_half_extent_m": arena_half_extent_m,
            "pose_bounds_tolerance_m": pose_bounds_tolerance_m,
            "camera_horizontal_fov_deg": camera_horizontal_fov_deg,
            "leave_start_enabled": leave_start_enabled,
            "leave_start_distance_m": leave_start_distance_m,
            "leave_start_speed": leave_start_speed,
            "leave_start_target_yaw_deg": leave_start_target_yaw_deg,
            "leave_start_heading_gain": leave_start_heading_gain,
            "leave_start_max_angular_speed": leave_start_max_angular_speed,
            "leave_start_heading_tolerance": leave_start_heading_tolerance,
            "coverage_enabled": coverage_enabled,
            "coverage_min_x": coverage_min_x,
            "coverage_max_x": coverage_max_x,
            "coverage_main_road_y": coverage_main_road_y,
            "coverage_scan_end_y": coverage_scan_end_y,
            "coverage_lane_spacing": coverage_lane_spacing,
            "coverage_scan_speed": coverage_scan_speed,
            "coverage_transit_speed": coverage_transit_speed,
            "coverage_return_speed": coverage_return_speed,
            "coverage_waypoint_tolerance": coverage_waypoint_tolerance,
            "coverage_turn_in_place_threshold": (coverage_turn_in_place_threshold),
            "coverage_max_angular_speed": coverage_max_angular_speed,
            "coverage_avoid_heading_tolerance": (
                coverage_avoid_heading_tolerance
            ),
            "coverage_avoid_angular_speed": coverage_avoid_angular_speed,
            "coverage_avoid_linear_scale": coverage_avoid_linear_scale,
            "coverage_rejoin_speed": coverage_rejoin_speed,
            "coverage_rejoin_coordinate_limit": coverage_rejoin_coordinate_limit,
            "coverage_reacquire_duration_s": coverage_reacquire_duration_s,
            "coverage_reacquire_reverse_after_s": (coverage_reacquire_reverse_after_s),
            "coverage_reacquire_angular_z": coverage_reacquire_angular_z,
            "lane_tof_correction_enabled": lane_tof_correction_enabled,
            "wall_distance_angle_topic": wall_distance_angle_topic,
            "tof_wall_angle_sign": tof_wall_angle_sign,
            "tof_validation_samples": tof_validation_samples,
            "tof_max_valid_wall_angle_rad": tof_max_valid_wall_angle_rad,
            "tof_max_angle_spread_rad": tof_max_angle_spread_rad,
            "tof_max_distance_spread_m": tof_max_distance_spread_m,
            "tof_alignment_watchdog_s": tof_alignment_watchdog_s,
            "pose_x_correction_topic": pose_x_correction_topic,
            "pose_y_correction_topic": pose_y_correction_topic,
            "pose_yaw_correction_topic": pose_yaw_correction_topic,
            "lane_tof_left_wall_x_m": lane_tof_left_wall_x_m,
            "lane_tof_right_wall_x_m": lane_tof_right_wall_x_m,
            "lane_tof_sensor_forward_offset_m": (lane_tof_sensor_forward_offset_m),
            "lane_tof_measurement_timeout_s": lane_tof_measurement_timeout_s,
            "lane_tof_x_tolerance_m": lane_tof_x_tolerance_m,
            "lane_tof_min_speed": lane_tof_min_speed,
            "lane_tof_slowdown_distance_m": lane_tof_slowdown_distance_m,
            "lane_tof_wall_angle_tolerance_rad": (
                lane_tof_wall_angle_tolerance_rad
            ),
            "main_road_tof_correction_enabled": (
                main_road_tof_correction_enabled
            ),
            "main_road_tof_south_wall_y_m": main_road_tof_south_wall_y_m,
            "main_road_tof_sensor_forward_offset_m": (
                main_road_tof_sensor_forward_offset_m
            ),
            "main_road_tof_measurement_timeout_s": (
                main_road_tof_measurement_timeout_s
            ),
            "main_road_tof_y_tolerance_m": main_road_tof_y_tolerance_m,
            "main_road_tof_min_speed": main_road_tof_min_speed,
            "main_road_tof_slowdown_distance_m": (
                main_road_tof_slowdown_distance_m
            ),
            "main_road_tof_angle_trigger_rad": (
                main_road_tof_angle_trigger_rad
            ),
            "main_road_tof_angle_release_rad": (
                main_road_tof_angle_release_rad
            ),
            "gripper_enabled": gripper_enabled,
            "gripper_type": gripper_type,
            "gripper_servo_id": gripper_servo_id,
            "gripper_open_position": gripper_open_position,
            "gripper_closed_position": gripper_closed_position,
            "gripper_move_duration_s": gripper_move_duration_s,
            "grab_center_tolerance": grab_center_tolerance,
            "grab_area_ratio": grab_area_ratio,
            "grab_detection_timeout_s": grab_detection_timeout_s,
            "final_forward_linear_x": final_forward_linear_x,
            "final_forward_duration_s": final_forward_duration_s,
            "grab_duration_s": grab_duration_s,
            "stop_after_grab": stop_after_grab,
            "full_mission_enabled": full_mission_enabled,
            "mission_duration_s": mission_duration_s,
            "force_return_remaining_s": force_return_remaining_s,
            "storage_capacity": storage_capacity,
            "target_object_count": target_object_count,
            "storage_main_road_y": storage_main_road_y,
            "storage_staging_x": storage_staging_x,
            "storage_staging_y": storage_staging_y,
            "storage_exit_x": storage_exit_x,
            "storage_center_x": storage_center_x,
            "storage_center_y": storage_center_y,
            "storage_entry_yaw_deg": storage_entry_yaw_deg,
            "storage_return_speed": storage_return_speed,
            "storage_entry_speed": storage_entry_speed,
            "storage_x_entry_speed": storage_x_entry_speed,
            "storage_exit_reverse_speed": storage_exit_reverse_speed,
            "storage_entry_dash_duration_s": storage_entry_dash_duration_s,
            "storage_exit_dash_duration_s": storage_exit_dash_duration_s,
            "storage_contact_settle_duration_s": (
                storage_contact_settle_duration_s
            ),
            "storage_dash_heading_deg": storage_dash_heading_deg,
            "storage_dash_heading_tolerance": storage_dash_heading_tolerance,
            "storage_dash_max_angular_speed": storage_dash_max_angular_speed,
            "storage_entry_tolerance": storage_entry_tolerance,
            "storage_tof_correction_enabled": storage_tof_correction_enabled,
            "storage_tof_left_wall_x_m": storage_tof_left_wall_x_m,
            "storage_tof_bottom_wall_y_m": storage_tof_bottom_wall_y_m,
            "storage_tof_sensor_forward_offset_m": (
                storage_tof_sensor_forward_offset_m
            ),
            "storage_tof_measurement_timeout_s": (storage_tof_measurement_timeout_s),
            "storage_exit_tof_fallback_timeout_s": (
                storage_exit_tof_fallback_timeout_s
            ),
            "storage_tof_xy_tolerance_m": storage_tof_xy_tolerance_m,
            "storage_tof_min_speed": storage_tof_min_speed,
            "storage_tof_slowdown_distance_m": (storage_tof_slowdown_distance_m),
            "storage_tof_wall_angle_tolerance_rad": (
                storage_tof_wall_angle_tolerance_rad
            ),
            "storage_exit_tof_angle_trigger_rad": (
                storage_exit_tof_angle_trigger_rad
            ),
            "storage_exit_tof_angle_release_rad": (
                storage_exit_tof_angle_release_rad
            ),
        }.items(),
    )

    object_mapper_node = Node(
        package="rl_model_policy",
        executable="rl_object_world_mapper",
        name="rl_object_world_mapper",
        output="screen",
        parameters=[
            {
            "detections_topic": "/yolo/detections",
            "odometry_topic": odometry_topic,
            "output_topic": "/rl_estimated_objects",
            "policy_state_topic": "/rl_model_policy_state",
            "target_classes": ParameterValue(target_classes, value_type=str),
            "avoid_classes": ParameterValue(avoid_classes, value_type=str),
            "calibration_path": object_calibration_path,
            "pose_timeout_s": ParameterValue(pose_timeout_s, value_type=float),
            "retention_s": ParameterValue(object_retention_s, value_type=float),
            "association_radius_m": ParameterValue(
                object_association_radius_m,
                value_type=float,
            ),
            "position_smoothing_alpha": ParameterValue(
                object_position_smoothing_alpha,
                value_type=float,
            ),
            "arena_half_extent_m": ParameterValue(
                arena_half_extent_m,
                value_type=float,
            ),
            }
        ],
        condition=IfCondition(launch_object_mapper),
        respawn=True,
        respawn_delay=2.0,
    )

    delayed_start = TimerAction(
        period=AUTO_START_DELAY_S,
        condition=IfCondition(auto_start),
        actions=[
            LogInfo(
                msg="Auto-starting deterministic drive after the 8 second startup delay."
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/rl_model_policy_control",
                    "std_msgs/msg/String",
                    "{data: start}",
                ],
                output="screen",
            ),
        ],
    )

    status_gui = Node(
        package="robot_status_gui",
        executable="robot_status_gui",
        name="robot_status_gui",
        output="screen",
        parameters=[
            {
            "detections_topic": "/yolo/detections",
            "target_classes": ParameterValue(target_classes, value_type=str),
            "avoid_classes": ParameterValue(avoid_classes, value_type=str),
            "calibration_path": object_calibration_path,
            "arena_half_extent_m": ParameterValue(
                arena_half_extent_m,
                value_type=float,
            ),
            "object_retention_s": ParameterValue(
                object_retention_s,
                value_type=float,
            ),
            "object_association_radius_m": ParameterValue(
                object_association_radius_m,
                value_type=float,
            ),
            "object_position_smoothing_alpha": ParameterValue(
                object_position_smoothing_alpha,
                value_type=float,
            ),
            }
        ],
        condition=IfCondition(launch_status_gui),
    )

    return LaunchDescription(
        [
        DeclareLaunchArgument(
            "yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                FindPackageShare("ros2_yolo_detector"),
                "models",
                "best.pt",
                    ]
                ),
            description="YOLO .pt or TensorRT .engine model path.",
        ),
        DeclareLaunchArgument(
            "secondary_yolo_model_path",
            default_value="",
            description=(
                "Fruit crop classifier model. Empty uses best_secondary.pt "
                "beside the primary YOLO model."
            ),
        ),
        DeclareLaunchArgument(
            "video_device",
            default_value=(
                "/dev/v4l/by-path/"
                "platform-3610000.usb-usb-0:2.1:1.0-video-index0"
            ),
            description="Stable V4L2 path for the driving camera.",
        ),
        DeclareLaunchArgument("target_classes", default_value="0"),
        DeclareLaunchArgument("avoid_classes", default_value=""),
        DeclareLaunchArgument("confidence", default_value="0.25"),
        DeclareLaunchArgument(
            "yolo_iou",
            default_value="0.45",
            description="IoU threshold used to suppress duplicate YOLO boxes.",
        ),
        DeclareLaunchArgument(
            "agnostic_nms",
            default_value="true",
            description="Suppress overlapping boxes even when their classes differ.",
        ),
        DeclareLaunchArgument("publish_annotated", default_value="false"),
        DeclareLaunchArgument("correction_enabled", default_value="true"),
        DeclareLaunchArgument("yolo_imgsz", default_value="800"),
        DeclareLaunchArgument("secondary_yolo_confidence", default_value="0.25"),
        DeclareLaunchArgument("secondary_yolo_imgsz", default_value="800"),
        DeclareLaunchArgument("yolo_min_bbox_area_ratio", default_value="0.02"),
        DeclareLaunchArgument("target_lock_enabled", default_value="true"),
        DeclareLaunchArgument("target_lock_timeout_s", default_value="0.80"),
        DeclareLaunchArgument("target_lock_iou_threshold", default_value="0.10"),
        DeclareLaunchArgument("target_lock_center_distance", default_value="0.20"),
        DeclareLaunchArgument("correction_backend", default_value="auto"),
        DeclareLaunchArgument("correction_device", default_value="cuda:0"),
            DeclareLaunchArgument(
                "yolo_performance_log_interval_s", default_value="5.0"
            ),
        DeclareLaunchArgument(
            "timer_rate_hz",
            default_value="10.0",
            description="Deterministic controller and cmd_vel publication rate.",
        ),
        DeclareLaunchArgument("approach_center_tolerance", default_value="0.12"),
        DeclareLaunchArgument("approach_max_linear_x", default_value="0.10"),
        DeclareLaunchArgument("approach_min_linear_x", default_value="0.03"),
        DeclareLaunchArgument("approach_angular_gain", default_value="0.8"),
        DeclareLaunchArgument("approach_max_angular_z", default_value="0.45"),
        DeclareLaunchArgument(
            "target_timeout_s",
            default_value="1.0",
            description="Keep tracking the last target through short YOLO detection gaps.",
        ),
        DeclareLaunchArgument(
            "target_tracking_timeout_s",
            default_value="1.5",
            description=(
                "Keep the last target briefly during class flicker after tracking starts."
            ),
        ),
        DeclareLaunchArgument(
            "target_confirmation_window",
            default_value="5",
            description="Number of recent YOLO frames used to confirm a target.",
        ),
        DeclareLaunchArgument(
            "target_confirmation_min_detections",
            default_value="3",
            description="Required target detections within the confirmation window.",
        ),
        DeclareLaunchArgument(
            "target_activation_center_y_min",
            default_value="0.30",
            description=(
                "Minimum normalized bbox-center y required before a new target "
                "can interrupt coverage and enter deterministic target tracking."
            ),
        ),
        DeclareLaunchArgument(
            "target_tracking_center_y_min",
            default_value="0.22",
            description=(
                "Lower bbox-center y threshold retained while target tracking "
                "is already active."
            ),
        ),
        DeclareLaunchArgument(
            "target_bearing_prediction_enabled",
            default_value="true",
            description="Project target image x from odometry during short detection gaps.",
        ),
        DeclareLaunchArgument("near_target_loss_enabled", default_value="true"),
        DeclareLaunchArgument("near_target_loss_margin", default_value="0.10"),
        DeclareLaunchArgument("near_target_loss_timeout_s", default_value="0.60"),
        DeclareLaunchArgument("near_target_loss_min_missing_s", default_value="0.15"),
        DeclareLaunchArgument("odometry_topic", default_value="/odom"),
        DeclareLaunchArgument("pose_timeout_s", default_value="0.5"),
        DeclareLaunchArgument(
            "pose_observation_enabled",
            default_value="false",
            description="Include pose/IMU fields in controller state telemetry.",
        ),
        DeclareLaunchArgument(
            "launch_pose_tracker",
            default_value="true",
            description="Start command/IMU pose tracking used by coverage search.",
        ),
        DeclareLaunchArgument(
            "pose_x_correction_topic",
            default_value="/robot_pose/correct_x",
        ),
            DeclareLaunchArgument(
                "pose_y_correction_topic",
                default_value="/robot_pose/correct_y",
            ),
        DeclareLaunchArgument(
            "pose_yaw_correction_topic",
            default_value="/robot_pose/correct_yaw",
        ),
        DeclareLaunchArgument(
            "launch_wall_distance_sensor",
            default_value="true",
            description="Start the two-VL53L1X wall distance node.",
        ),
        DeclareLaunchArgument("wall_driver_backend", default_value="vl53l1x"),
        DeclareLaunchArgument("wall_left_i2c_bus", default_value="7"),
        DeclareLaunchArgument("wall_right_i2c_bus", default_value="1"),
        DeclareLaunchArgument(
            "wall_left_address",
            default_value="41",
            description="Integer I2C address; 41 decimal is 0x29.",
        ),
        DeclareLaunchArgument(
            "wall_right_address",
            default_value="41",
            description="Integer I2C address; 41 decimal is 0x29.",
        ),
        DeclareLaunchArgument(
            "wall_ranging_mode",
            default_value="3",
            description="VL53L1X long-range mode (1=short, 2=medium, 3=long).",
        ),
        DeclareLaunchArgument(
            "wall_distance_angle_topic",
            default_value="/wall/distance_angle",
        ),
        DeclareLaunchArgument("arena_half_extent_m", default_value="2.0"),
        DeclareLaunchArgument("pose_bounds_tolerance_m", default_value="0.25"),
        DeclareLaunchArgument("camera_horizontal_fov_deg", default_value="80.0"),
        DeclareLaunchArgument(
            "launch_object_mapper",
            default_value="true",
            description="Estimate detected object positions on the arena map.",
        ),
        DeclareLaunchArgument(
            "launch_status_gui",
            default_value="false",
            description="Open the PyQt robot status GUI on the local display.",
        ),
        DeclareLaunchArgument(
            "object_calibration_path",
                default_value=PathJoinSubstitution(
                    [
                FindPackageShare("robot_status_gui"),
                "config",
                "distance_normalized_points.csv",
                    ]
                ),
            description="Measured bbox-center to camera-position calibration CSV.",
        ),
        DeclareLaunchArgument("object_association_radius_m", default_value="0.30"),
        DeclareLaunchArgument(
            "object_position_smoothing_alpha",
            default_value="0.35",
        ),
        DeclareLaunchArgument("object_retention_s", default_value="180.0"),
        DeclareLaunchArgument(
            "coverage_enabled",
            default_value="true",
            description="Use odometry-based lane coverage while no target is visible.",
        ),
            DeclareLaunchArgument("coverage_min_x", default_value="-1.25"),
        DeclareLaunchArgument("coverage_max_x", default_value="1.25"),
        DeclareLaunchArgument("coverage_main_road_y", default_value="-1.3343"),
        DeclareLaunchArgument("coverage_scan_end_y", default_value="1.0"),
        DeclareLaunchArgument("coverage_lane_spacing", default_value="1.0"),
        DeclareLaunchArgument("coverage_scan_speed", default_value="0.24"),
        DeclareLaunchArgument("coverage_transit_speed", default_value="0.40"),
        DeclareLaunchArgument("coverage_return_speed", default_value="0.24"),
        DeclareLaunchArgument("coverage_waypoint_tolerance", default_value="0.10"),
        DeclareLaunchArgument(
            "coverage_turn_in_place_threshold",
            default_value="0.65",
        ),
        DeclareLaunchArgument("coverage_max_angular_speed", default_value="1.00"),
        DeclareLaunchArgument(
            "coverage_avoid_heading_tolerance",
            default_value="0.14",
            description=(
                "Allow lane obstacle avoidance only after heading error is "
                "within this tolerance in radians."
            ),
        ),
        DeclareLaunchArgument("coverage_avoid_angular_speed", default_value="0.45"),
        DeclareLaunchArgument("coverage_avoid_linear_scale", default_value="0.70"),
        DeclareLaunchArgument("coverage_rejoin_speed", default_value="0.20"),
        DeclareLaunchArgument(
            "coverage_rejoin_coordinate_limit",
            default_value="1.80",
        ),
        DeclareLaunchArgument("coverage_reacquire_duration_s", default_value="1.5"),
        DeclareLaunchArgument(
            "coverage_reacquire_reverse_after_s",
            default_value="0.75",
        ),
        DeclareLaunchArgument("coverage_reacquire_angular_z", default_value="0.35"),
        DeclareLaunchArgument(
            "lane_tof_correction_enabled",
            default_value="true",
            description=(
                "Fine-align x with VL53L1X after reaching the lane waypoint."
            ),
        ),
        DeclareLaunchArgument("lane_tof_left_wall_x_m", default_value="-2.0"),
            DeclareLaunchArgument("lane_tof_right_wall_x_m", default_value="2.0"),
        DeclareLaunchArgument(
            "lane_tof_sensor_forward_offset_m",
                default_value="0.09",
        ),
        DeclareLaunchArgument(
            "lane_tof_measurement_timeout_s",
            default_value="0.25",
        ),
        DeclareLaunchArgument("lane_tof_x_tolerance_m", default_value="0.03"),
        DeclareLaunchArgument("lane_tof_min_speed", default_value="0.08"),
        DeclareLaunchArgument(
            "lane_tof_slowdown_distance_m",
            default_value="0.20",
        ),
        DeclareLaunchArgument(
            "lane_tof_wall_angle_tolerance_rad",
            default_value="0.05",
            description="Maximum east/west wall angle before x correction.",
        ),
        DeclareLaunchArgument("tof_wall_angle_sign", default_value="1.0"),
        DeclareLaunchArgument("tof_validation_samples", default_value="3"),
        DeclareLaunchArgument("tof_max_valid_wall_angle_rad", default_value="0.436332313"),
        DeclareLaunchArgument("tof_max_angle_spread_rad", default_value="0.13962634"),
        DeclareLaunchArgument("tof_max_distance_spread_m", default_value="0.12"),
        DeclareLaunchArgument("tof_alignment_watchdog_s", default_value="4.0"),
        DeclareLaunchArgument(
            "main_road_tof_correction_enabled",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "main_road_tof_south_wall_y_m",
            default_value="-2.0",
        ),
        DeclareLaunchArgument(
            "main_road_tof_sensor_forward_offset_m",
            default_value="0.09",
        ),
        DeclareLaunchArgument(
            "main_road_tof_measurement_timeout_s",
            default_value="0.25",
        ),
        DeclareLaunchArgument(
            "main_road_tof_y_tolerance_m",
            default_value="0.03",
        ),
        DeclareLaunchArgument(
            "main_road_tof_min_speed",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "main_road_tof_slowdown_distance_m",
            default_value="0.20",
        ),
        DeclareLaunchArgument(
            "main_road_tof_angle_trigger_rad",
            default_value="0.1745329252",
            description="Start south-wall angle correction at 10 degrees.",
        ),
        DeclareLaunchArgument(
            "main_road_tof_angle_release_rad",
            default_value="0.0872664626",
            description="Finish south-wall angle correction at 5 degrees.",
        ),
        DeclareLaunchArgument("leave_start_enabled", default_value="true"),
        DeclareLaunchArgument("leave_start_distance_m", default_value="0.55"),
        DeclareLaunchArgument("leave_start_speed", default_value="0.25"),
        DeclareLaunchArgument("leave_start_target_yaw_deg", default_value="90.0"),
        DeclareLaunchArgument("leave_start_heading_gain", default_value="1.5"),
        DeclareLaunchArgument(
            "leave_start_max_angular_speed",
            default_value="0.60",
        ),
        DeclareLaunchArgument("leave_start_heading_tolerance", default_value="0.12"),
        DeclareLaunchArgument(
            "initial_x",
            default_value="1.8",
            description="Start x in the arena-center coordinate frame used by training.",
        ),
        DeclareLaunchArgument(
            "initial_y",
            default_value="-1.8",
            description="Start y in the arena-center coordinate frame used by training.",
        ),
        DeclareLaunchArgument("initial_yaw_deg", default_value="180.0"),
        DeclareLaunchArgument("pose_linear_scale", default_value="1.0"),
        DeclareLaunchArgument("imu_yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("gyro_calibration_duration_s", default_value="2.0"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("full_mission_enabled", default_value="true"),
        DeclareLaunchArgument("mission_duration_s", default_value="180.0"),
        DeclareLaunchArgument("force_return_remaining_s", default_value="30.0"),
        DeclareLaunchArgument("storage_capacity", default_value="4"),
        DeclareLaunchArgument("target_object_count", default_value="7"),
        DeclareLaunchArgument("storage_main_road_y", default_value="-1.3343"),
            DeclareLaunchArgument("storage_staging_x", default_value="-1.25"),
            DeclareLaunchArgument("storage_staging_y", default_value="-1.75"),
            DeclareLaunchArgument("storage_exit_x", default_value="-1.25"),
        DeclareLaunchArgument("storage_center_x", default_value="-1.75"),
        DeclareLaunchArgument("storage_center_y", default_value="-1.75"),
        DeclareLaunchArgument("storage_entry_yaw_deg", default_value="-90.0"),
        DeclareLaunchArgument("storage_return_speed", default_value="0.25"),
            DeclareLaunchArgument("storage_entry_speed", default_value="0.30"),
            DeclareLaunchArgument("storage_x_entry_speed", default_value="0.40"),
        DeclareLaunchArgument(
            "storage_exit_reverse_speed",
            default_value="0.40",
        ),
        DeclareLaunchArgument("storage_entry_dash_duration_s", default_value="2.50"),
        DeclareLaunchArgument("storage_exit_dash_duration_s", default_value="1.50"),
        DeclareLaunchArgument(
            "storage_contact_settle_duration_s",
            default_value="0.20",
        ),
        DeclareLaunchArgument("storage_dash_heading_deg", default_value="-139.26"),
        DeclareLaunchArgument(
            "storage_dash_heading_tolerance",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "storage_dash_max_angular_speed",
            default_value="0.30",
        ),
        DeclareLaunchArgument("storage_entry_tolerance", default_value="0.04"),
            DeclareLaunchArgument(
                "storage_tof_correction_enabled",
                default_value="true",
                description=(
                    "Correct storage entrance x with ToF, plus final y for "
                    "returns from lanes 1 and 2."
                ),
            ),
            DeclareLaunchArgument("storage_tof_left_wall_x_m", default_value="-2.0"),
            DeclareLaunchArgument("storage_tof_bottom_wall_y_m", default_value="-2.0"),
            DeclareLaunchArgument(
                "storage_tof_sensor_forward_offset_m",
                default_value="0.09",
            ),
            DeclareLaunchArgument(
                "storage_tof_measurement_timeout_s",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "storage_exit_tof_fallback_timeout_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument("storage_tof_xy_tolerance_m", default_value="0.03"),
            DeclareLaunchArgument("storage_tof_min_speed", default_value="0.05"),
            DeclareLaunchArgument(
                "storage_tof_slowdown_distance_m",
                default_value="0.20",
            ),
        DeclareLaunchArgument(
            "storage_tof_wall_angle_tolerance_rad",
            default_value="0.05",
            description="Maximum ToF wall angle before storage pose correction.",
        ),
        DeclareLaunchArgument(
            "storage_exit_tof_angle_trigger_rad",
            default_value="0.1745329252",
            description="Start storage-exit west-wall alignment at 10 degrees.",
        ),
        DeclareLaunchArgument(
            "storage_exit_tof_angle_release_rad",
            default_value="0.0872664626",
            description="Finish storage-exit west-wall alignment at 5 degrees.",
        ),
        DeclareLaunchArgument("gripper_enabled", default_value="true"),
        DeclareLaunchArgument("gripper_type", default_value="bus"),
        DeclareLaunchArgument("gripper_servo_id", default_value="1"),
        DeclareLaunchArgument("gripper_open_position", default_value="1000"),
        DeclareLaunchArgument("gripper_closed_position", default_value="300"),
        DeclareLaunchArgument("gripper_move_duration_s", default_value="0.5"),
        DeclareLaunchArgument("grab_center_tolerance", default_value="0.18"),
        DeclareLaunchArgument("grab_area_ratio", default_value="0.70"),
        DeclareLaunchArgument("grab_detection_timeout_s", default_value="0.25"),
        DeclareLaunchArgument("final_forward_linear_x", default_value="0.20"),
        DeclareLaunchArgument("final_forward_duration_s", default_value="1.2"),
        DeclareLaunchArgument("grab_duration_s", default_value="1.0"),
        DeclareLaunchArgument("stop_after_grab", default_value="false"),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description="Start driving automatically after an 8 second delay.",
        ),
        LogInfo(
            condition=IfCondition(auto_start),
            msg="AUTO START enabled: the robot will move after 8 seconds.",
        ),
        controller_launch,
        yolo_launch,
        motor_launch,
        wall_distance_launch,
        pose_tracker_launch,
        mission_controller_launch,
        object_mapper_node,
        status_gui,
        delayed_start,
        ]
    )
