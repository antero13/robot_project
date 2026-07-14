from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_yolo_detector'),
                'launch',
                'v4l2_yolo_camera.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_camera')),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'video_device': LaunchConfiguration('video_device'),
            'image_topic': '/image_raw',
            'image_width': '640',
            'image_height': '480',
            'time_per_frame_numerator': '1',
            'time_per_frame_denominator': '30',
            'pixel_format': 'YUYV',
            'output_encoding': 'bgr8',
            'power_line_frequency': '2',
            'auto_exposure': '1',
            'exposure_time_absolute': '200',
            'gain': '20',
            'confidence': '0.25',
            'iou': '0.45',
            'device': LaunchConfiguration('yolo_device'),
            'imgsz': '800',
            'correction_enabled': 'true',
            'correction_gamma': '0.65',
            'correction_clahe_clip_limit': '1.2',
            'correction_clahe_tile_grid': '8',
            'correction_chroma_gain': '1.3',
            'publish_annotated': LaunchConfiguration('publish_annotated'),
            'detections_topic': '/yolo/detections',
            'publish_target': 'false',
        }.items(),
    )

    wall_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wall_distance_sensor'),
                'launch',
                'wall_distance_angle.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_wall_sensor')),
        launch_arguments={
            'driver_backend': LaunchConfiguration('wall_driver_backend'),
            'left_i2c_bus': LaunchConfiguration('left_i2c_bus'),
            'right_i2c_bus': LaunchConfiguration('right_i2c_bus'),
            'sensor_separation_m': '0.29',
            'safe_distance_m': '0.15',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_manager_2'),
                'config',
                'mission_manager_2.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('ros2_yolo_detector'),
                'models',
                'best.pt',
            ]),
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0',
        ),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('launch_wall_sensor', default_value='true'),
        DeclareLaunchArgument('wall_driver_backend', default_value='vl53l1x'),
        DeclareLaunchArgument('left_i2c_bus', default_value='7'),
        DeclareLaunchArgument('right_i2c_bus', default_value='1'),
        DeclareLaunchArgument('publish_annotated', default_value='false'),
        DeclareLaunchArgument('yolo_device', default_value=''),
        DeclareLaunchArgument('linear_scale', default_value='1.0'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        camera_launch,
        wall_launch,
        Node(
            package='robot_pose_tracker',
            executable='robot_pose_tracker',
            name='robot_pose_tracker',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'imu_topic': '/ros_robot_controller/imu_raw',
                'initial_x': 3.25,
                'initial_y': 0.6656854249,
                'initial_yaw_deg': 90.0,
                'linear_scale': ParameterValue(
                    LaunchConfiguration('linear_scale'),
                    value_type=float,
                ),
                'imu_yaw_sign': ParameterValue(
                    LaunchConfiguration('imu_yaw_sign'),
                    value_type=float,
                ),
                'gyro_calibration_duration_s': 2.0,
                'publish_tf': True,
            }],
        ),
        Node(
            package='mission_manager_2',
            executable='mission_manager_2',
            name='mission_manager_2',
            output='screen',
            parameters=[
                LaunchConfiguration('mission_config'),
                {
                    'auto_start': ParameterValue(
                        LaunchConfiguration('auto_start'),
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='cmd_vel_to_motor',
            executable='cmd_vel_to_motor',
            name='cmd_vel_to_motor',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'motor_topic': '/ros_robot_controller/set_motor',
                'wheel_radius_m': 0.05,
                'wheel_separation_m': 0.32,
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
