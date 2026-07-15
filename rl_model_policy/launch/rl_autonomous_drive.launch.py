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
    rl_model_path = LaunchConfiguration("rl_model_path")
    video_device = LaunchConfiguration("video_device")
    target_classes = LaunchConfiguration("target_classes")
    avoid_classes = LaunchConfiguration("avoid_classes")
    confidence = LaunchConfiguration("confidence")
    publish_annotated = LaunchConfiguration("publish_annotated")
    speed_scale = LaunchConfiguration("speed_scale")
    target_timeout_s = LaunchConfiguration("target_timeout_s")
    target_confirmation_window = LaunchConfiguration("target_confirmation_window")
    target_confirmation_min_detections = LaunchConfiguration(
        "target_confirmation_min_detections"
    )
    target_bearing_prediction_enabled = LaunchConfiguration(
        "target_bearing_prediction_enabled"
    )
    odometry_topic = LaunchConfiguration("odometry_topic")
    pose_timeout_s = LaunchConfiguration("pose_timeout_s")
    pose_observation_enabled = LaunchConfiguration("pose_observation_enabled")
    launch_pose_tracker = LaunchConfiguration("launch_pose_tracker")
    arena_half_extent_m = LaunchConfiguration("arena_half_extent_m")
    pose_bounds_tolerance_m = LaunchConfiguration("pose_bounds_tolerance_m")
    camera_horizontal_fov_deg = LaunchConfiguration("camera_horizontal_fov_deg")
    launch_object_mapper = LaunchConfiguration("launch_object_mapper")
    launch_status_gui = LaunchConfiguration("launch_status_gui")
    object_calibration_path = LaunchConfiguration("object_calibration_path")
    object_association_radius_m = LaunchConfiguration(
        "object_association_radius_m"
    )
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
    coverage_max_angular_speed = LaunchConfiguration(
        "coverage_max_angular_speed"
    )
    coverage_avoid_angular_speed = LaunchConfiguration(
        "coverage_avoid_angular_speed"
    )
    coverage_avoid_linear_scale = LaunchConfiguration(
        "coverage_avoid_linear_scale"
    )
    coverage_reacquire_duration_s = LaunchConfiguration("coverage_reacquire_duration_s")
    coverage_reacquire_reverse_after_s = LaunchConfiguration(
        "coverage_reacquire_reverse_after_s"
    )
    coverage_reacquire_angular_z = LaunchConfiguration(
        "coverage_reacquire_angular_z"
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
    storage_center_x = LaunchConfiguration("storage_center_x")
    storage_center_y = LaunchConfiguration("storage_center_y")
    storage_entry_yaw_deg = LaunchConfiguration("storage_entry_yaw_deg")
    storage_return_speed = LaunchConfiguration("storage_return_speed")
    storage_entry_speed = LaunchConfiguration("storage_entry_speed")
    storage_exit_reverse_speed = LaunchConfiguration("storage_exit_reverse_speed")
    storage_entry_tolerance = LaunchConfiguration("storage_entry_tolerance")

    controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_robot_controller"),
            "launch",
            "ros_robot_controller.launch.xml",
        ])
    )

    yolo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros2_yolo_detector"),
            "launch",
            "v4l2_yolo_camera.launch.py",
        ]),
        launch_arguments={
            "model_path": yolo_model_path,
            "video_device": video_device,
            "target_classes": target_classes,
            "avoid_classes": avoid_classes,
            "confidence": confidence,
            "publish_annotated": publish_annotated,
        }.items(),
    )

    motor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("cmd_vel_to_motor"),
            "launch",
            "cmd_vel_to_motor.launch.py",
        ])
    )

    pose_tracker_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot_pose_tracker"),
            "launch",
            "robot_pose_tracker.launch.py",
        ]),
        launch_arguments={
            "cmd_vel_topic": "/cmd_vel",
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

    policy_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rl_model_policy"),
            "launch",
            "rl_model_policy.launch.py",
        ]),
        launch_arguments={
            "model_path": rl_model_path,
            "speed_scale": speed_scale,
            "target_timeout_s": target_timeout_s,
            "target_confirmation_window": target_confirmation_window,
            "target_confirmation_min_detections": (
                target_confirmation_min_detections
            ),
            "target_bearing_prediction_enabled": target_bearing_prediction_enabled,
            "dry_run": dry_run,
            "odometry_topic": odometry_topic,
            "pose_timeout_s": pose_timeout_s,
            "pose_observation_enabled": pose_observation_enabled,
            "arena_half_extent_m": arena_half_extent_m,
            "pose_bounds_tolerance_m": pose_bounds_tolerance_m,
            "camera_horizontal_fov_deg": camera_horizontal_fov_deg,
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
            "coverage_turn_in_place_threshold": (
                coverage_turn_in_place_threshold
            ),
            "coverage_max_angular_speed": coverage_max_angular_speed,
            "coverage_avoid_angular_speed": coverage_avoid_angular_speed,
            "coverage_avoid_linear_scale": coverage_avoid_linear_scale,
            "coverage_reacquire_duration_s": coverage_reacquire_duration_s,
            "coverage_reacquire_reverse_after_s": (
                coverage_reacquire_reverse_after_s
            ),
            "coverage_reacquire_angular_z": coverage_reacquire_angular_z,
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
            "storage_center_x": storage_center_x,
            "storage_center_y": storage_center_y,
            "storage_entry_yaw_deg": storage_entry_yaw_deg,
            "storage_return_speed": storage_return_speed,
            "storage_entry_speed": storage_entry_speed,
            "storage_exit_reverse_speed": storage_exit_reverse_speed,
            "storage_entry_tolerance": storage_entry_tolerance,
        }.items(),
    )

    object_mapper_node = Node(
        package="rl_model_policy",
        executable="rl_object_world_mapper",
        name="rl_object_world_mapper",
        output="screen",
        parameters=[{
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
        }],
        condition=IfCondition(launch_object_mapper),
        respawn=True,
        respawn_delay=2.0,
    )

    delayed_start = TimerAction(
        period=AUTO_START_DELAY_S,
        condition=IfCondition(auto_start),
        actions=[
            LogInfo(msg="Auto-starting RL drive after the 8 second startup delay."),
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
        parameters=[{
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
        }],
        condition=IfCondition(launch_status_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "yolo_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros2_yolo_detector"),
                "models",
                "best.pt",
            ]),
            description="YOLO .pt or TensorRT .engine model path.",
        ),
        DeclareLaunchArgument(
            "rl_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("mission_manager"),
                "models",
                "rl_avoid_search_best.pt",
            ]),
            description="Trained RL policy checkpoint path.",
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
        DeclareLaunchArgument("publish_annotated", default_value="false"),
        DeclareLaunchArgument("speed_scale", default_value="0.50"),
        DeclareLaunchArgument(
            "target_timeout_s",
            default_value="1.0",
            description="Keep tracking the last target through short YOLO detection gaps.",
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
            "target_bearing_prediction_enabled",
            default_value="true",
            description="Project target image x from odometry during short detection gaps.",
        ),
        DeclareLaunchArgument("odometry_topic", default_value="/odom"),
        DeclareLaunchArgument("pose_timeout_s", default_value="0.5"),
        DeclareLaunchArgument(
            "pose_observation_enabled",
            default_value="false",
            description="Use pose/IMU policy inputs and yaw-based target prediction.",
        ),
        DeclareLaunchArgument(
            "launch_pose_tracker",
            default_value="true",
            description="Start command/IMU pose tracking used by coverage search.",
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
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_status_gui"),
                "config",
                "distance_normalized_points.csv",
            ]),
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
        DeclareLaunchArgument("coverage_min_x", default_value="-0.75"),
        DeclareLaunchArgument("coverage_max_x", default_value="1.25"),
        DeclareLaunchArgument("coverage_main_road_y", default_value="-1.3343"),
        DeclareLaunchArgument("coverage_scan_end_y", default_value="1.0"),
        DeclareLaunchArgument("coverage_lane_spacing", default_value="1.0"),
        DeclareLaunchArgument("coverage_scan_speed", default_value="0.24"),
        DeclareLaunchArgument("coverage_transit_speed", default_value="0.30"),
        DeclareLaunchArgument("coverage_return_speed", default_value="0.24"),
        DeclareLaunchArgument("coverage_waypoint_tolerance", default_value="0.10"),
        DeclareLaunchArgument(
            "coverage_turn_in_place_threshold",
            default_value="0.65",
        ),
        DeclareLaunchArgument("coverage_max_angular_speed", default_value="1.00"),
        DeclareLaunchArgument("coverage_avoid_angular_speed", default_value="0.45"),
        DeclareLaunchArgument("coverage_avoid_linear_scale", default_value="0.70"),
        DeclareLaunchArgument("coverage_reacquire_duration_s", default_value="1.5"),
        DeclareLaunchArgument(
            "coverage_reacquire_reverse_after_s",
            default_value="0.75",
        ),
        DeclareLaunchArgument("coverage_reacquire_angular_z", default_value="0.35"),
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
        DeclareLaunchArgument("initial_yaw_deg", default_value="90.0"),
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
        DeclareLaunchArgument("storage_staging_x", default_value="-1.75"),
        DeclareLaunchArgument("storage_staging_y", default_value="-1.25"),
        DeclareLaunchArgument("storage_center_x", default_value="-1.75"),
        DeclareLaunchArgument("storage_center_y", default_value="-1.75"),
        DeclareLaunchArgument("storage_entry_yaw_deg", default_value="-90.0"),
        DeclareLaunchArgument("storage_return_speed", default_value="0.25"),
        DeclareLaunchArgument("storage_entry_speed", default_value="0.12"),
        DeclareLaunchArgument(
            "storage_exit_reverse_speed",
            default_value="0.16",
        ),
        DeclareLaunchArgument("storage_entry_tolerance", default_value="0.04"),
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
        DeclareLaunchArgument("final_forward_duration_s", default_value="1.0"),
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
        pose_tracker_launch,
        policy_launch,
        object_mapper_node,
        status_gui,
        delayed_start,
    ])
