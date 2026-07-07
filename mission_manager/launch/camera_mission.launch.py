from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ros2_yolo_detector'), 'models', 'best.pt']
            ),
        ),
        DeclareLaunchArgument('target_class', default_value=''),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('show_window', default_value='false'),
        Node(
            package='yolo_target_detector',
            executable='yolo_target_detector',
            name='yolo_target_detector',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'target_class': LaunchConfiguration('target_class'),
                'camera_index': LaunchConfiguration('camera_index'),
                'device': LaunchConfiguration('device'),
                'show_window': LaunchConfiguration('show_window'),
                'target_topic': '/target_object',
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
                'pwm_servo_topic': '/ros_robot_controller/pwm_servo/set_state',
                'timer_rate_hz': 20.0,
                'search_angular_z': 0.35,
                'approach_max_linear_x': 0.10,
                'approach_min_linear_x': 0.03,
                'approach_angular_gain': 0.8,
                'approach_max_angular_z': 0.45,
                'center_tolerance': 0.12,
                'grab_area_ratio': 0.10,
                'target_timeout_s': 0.5,
                'storage_linear_x': 0.10,
                'back_out_linear_x': -0.08,
                'search_duration_s': 30.0,
                'grab_duration_s': 1.0,
                'move_to_storage_duration_s': 4.0,
                'release_duration_s': 1.0,
                'back_out_duration_s': 1.5,
                'gripper_enabled': False,
            }],
        ),
    ])
