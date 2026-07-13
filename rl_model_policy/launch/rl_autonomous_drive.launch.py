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
    odometry_topic = LaunchConfiguration("odometry_topic")
    pose_timeout_s = LaunchConfiguration("pose_timeout_s")
    arena_half_extent_m = LaunchConfiguration("arena_half_extent_m")
    pose_bounds_tolerance_m = LaunchConfiguration("pose_bounds_tolerance_m")
    camera_horizontal_fov_deg = LaunchConfiguration("camera_horizontal_fov_deg")
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
    final_forward_linear_x = LaunchConfiguration("final_forward_linear_x")
    final_forward_duration_s = LaunchConfiguration("final_forward_duration_s")
    grab_duration_s = LaunchConfiguration("grab_duration_s")
    stop_after_grab = LaunchConfiguration("stop_after_grab")

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
            "dry_run": dry_run,
            "odometry_topic": odometry_topic,
            "pose_timeout_s": pose_timeout_s,
            "arena_half_extent_m": arena_half_extent_m,
            "pose_bounds_tolerance_m": pose_bounds_tolerance_m,
            "camera_horizontal_fov_deg": camera_horizontal_fov_deg,
            "gripper_enabled": gripper_enabled,
            "gripper_type": gripper_type,
            "gripper_servo_id": gripper_servo_id,
            "gripper_open_position": gripper_open_position,
            "gripper_closed_position": gripper_closed_position,
            "gripper_move_duration_s": gripper_move_duration_s,
            "grab_center_tolerance": grab_center_tolerance,
            "grab_area_ratio": grab_area_ratio,
            "final_forward_linear_x": final_forward_linear_x,
            "final_forward_duration_s": final_forward_duration_s,
            "grab_duration_s": grab_duration_s,
            "stop_after_grab": stop_after_grab,
        }.items(),
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
        DeclareLaunchArgument("speed_scale", default_value="0.25"),
        DeclareLaunchArgument("odometry_topic", default_value="/odom"),
        DeclareLaunchArgument("pose_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("arena_half_extent_m", default_value="2.0"),
        DeclareLaunchArgument("pose_bounds_tolerance_m", default_value="0.25"),
        DeclareLaunchArgument("camera_horizontal_fov_deg", default_value="90.0"),
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
        DeclareLaunchArgument("gripper_enabled", default_value="true"),
        DeclareLaunchArgument("gripper_type", default_value="bus"),
        DeclareLaunchArgument("gripper_servo_id", default_value="1"),
        DeclareLaunchArgument("gripper_open_position", default_value="1000"),
        DeclareLaunchArgument("gripper_closed_position", default_value="250"),
        DeclareLaunchArgument("gripper_move_duration_s", default_value="0.5"),
        DeclareLaunchArgument("grab_center_tolerance", default_value="0.12"),
        DeclareLaunchArgument("grab_area_ratio", default_value="0.50"),
        DeclareLaunchArgument("final_forward_linear_x", default_value="0.06"),
        DeclareLaunchArgument("final_forward_duration_s", default_value="1.6"),
        DeclareLaunchArgument("grab_duration_s", default_value="1.0"),
        DeclareLaunchArgument("stop_after_grab", default_value="true"),
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
        delayed_start,
    ])
