import json
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mission_manager_2.mission_logic import (
    Pose2D,
    angular_error,
    clamp,
    parse_class_list,
    pose_distance,
    quaternion_to_yaw,
    select_target,
    target_is_large_enough,
    wall_matches_expected,
)


class MissionState(str, Enum):
    IDLE = 'IDLE'
    LEAVE_START_ARC = 'LEAVE_START_ARC'
    NAV_TURN = 'NAV_TURN'
    NAV_DRIVE = 'NAV_DRIVE'
    TURN_TO_HEADING = 'TURN_TO_HEADING'
    PREPARE_LANE = 'PREPARE_LANE'
    ALIGN_LANE_WITH_WALL = 'ALIGN_LANE_WITH_WALL'
    SCAN_LANE = 'SCAN_LANE'
    ALIGN_TARGET = 'ALIGN_TARGET'
    APPROACH_TARGET = 'APPROACH_TARGET'
    OPEN_GRIPPER = 'OPEN_GRIPPER'
    FINAL_GRAB_FORWARD = 'FINAL_GRAB_FORWARD'
    CLOSE_GRIPPER = 'CLOSE_GRIPPER'
    RETURN_TO_CHECKPOINT = 'RETURN_TO_CHECKPOINT'
    AFTER_TARGET_RETURN = 'AFTER_TARGET_RETURN'
    RETURN_MAIN_ROAD = 'RETURN_MAIN_ROAD'
    STORAGE_APPROACH_WALL = 'STORAGE_APPROACH_WALL'
    STORAGE_OPEN = 'STORAGE_OPEN'
    STORAGE_BACK_OUT = 'STORAGE_BACK_OUT'
    STORAGE_CLOSE = 'STORAGE_CLOSE'
    DONE = 'DONE'
    STOPPED = 'STOPPED'
    ERROR = 'ERROR'


ACTIVE_STATES = set(MissionState) - {
    MissionState.IDLE,
    MissionState.DONE,
    MissionState.STOPPED,
    MissionState.ERROR,
}


class MissionManager2(Node):
    def __init__(self):
        super().__init__('mission_manager_2')
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self.state = MissionState.IDLE
        self.state_started_at = self.get_clock().now()
        self.state_reason = 'waiting for start command'
        self.last_state_published = None
        self.last_status_published_at = None

        self.pose = None
        self.pose_received_at = None
        self.wall_distance = None
        self.wall_angle = None
        self.wall_min_distance = None
        self.wall_received_at = None
        self.left_range = None
        self.left_range_received_at = None
        self.right_range = None
        self.right_range_received_at = None

        self.latest_target = None
        self.target_received_at = None
        self.detection_frame_received_at = None
        self.locked_target_class = None
        self.target_checkpoint = None
        self.approach_started_pose = None
        self.motion_started_pose = None
        self.motion_heading = None

        self.nav_goal = None
        self.nav_next_state = None
        self.turn_goal_yaw = None
        self.turn_next_state = None
        self.lane_heading_yaw = math.pi * 0.5
        self.wall_aligned_ticks = 0
        self.lane_index = 0
        self.carried_count = 0
        self.total_pick_attempts = 0
        self.storage_pending = False
        self.pickup_succeeded = False
        self.grab_counted = False
        self.action_command_sent = False
        self.target_cooldown_until_s = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.pwm_servo_pub = self.create_publisher(
            SetPWMServoState,
            self.pwm_servo_topic,
            10,
        )
        self.bus_servo_pub = self.create_publisher(
            SetBusServoState,
            self.bus_servo_topic,
            10,
        )

        self.control_sub = self.create_subscription(
            String,
            self.control_topic,
            self.control_callback,
            10,
        )
        self.detections_sub = self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.wall_sub = self.create_subscription(
            Vector3Stamped,
            self.wall_topic,
            self.wall_callback,
            qos_profile_sensor_data,
        )
        self.left_range_sub = self.create_subscription(
            Range,
            self.left_range_topic,
            self.left_range_callback,
            qos_profile_sensor_data,
        )
        self.right_range_sub = self.create_subscription(
            Range,
            self.right_range_topic,
            self.right_range_callback,
            qos_profile_sensor_data,
        )
        self.pose_reset_client = self.create_client(Trigger, self.pose_reset_service)

        self.timer = self.create_timer(1.0 / self.timer_rate_hz, self.tick)
        self.publish_state(force=True)
        self.get_logger().info(
            'mission_manager_2 ready; start with: '
            f'ros2 topic pub --once {self.control_topic} std_msgs/msg/String "{{data: start}}"'
        )
        self.get_logger().info(
            f'Fixed search lanes={self.lane_x_positions}, main_road_y={self.main_road_y:.3f} m, '
            f'capacity={self.pickup_capacity}'
        )

        if self.auto_start:
            self.reset_mission_progress()
            self.transition(MissionState.LEAVE_START_ARC, 'automatic start')

    def _declare_parameters(self):
        topics = {
            'cmd_vel_topic': '/cmd_vel',
            'control_topic': '/mission2/control',
            'state_topic': '/mission2/state',
            'status_topic': '/mission2/status',
            'detections_topic': '/yolo/detections',
            'odom_topic': '/odom',
            'wall_topic': '/wall/distance_angle',
            'left_range_topic': '/wall/left_range',
            'right_range_topic': '/wall/right_range',
            'pose_reset_service': '/robot_pose/reset',
            'pwm_servo_topic': '/ros_robot_controller/pwm_servo/set_state',
            'bus_servo_topic': '/ros_robot_controller/bus_servo/set_state',
        }
        for name, default in topics.items():
            self.declare_parameter(name, default)

        parameters = {
            'auto_start': False,
            'timer_rate_hz': 20.0,
            'pose_timeout_s': 0.6,
            'wall_timeout_s': 0.5,
            'status_rate_hz': 5.0,
            'arena_width_m': 4.0,
            'arena_height_m': 4.0,
            'lane_x_positions': [3.25, 2.25, 1.25, 0.25],
            'main_road_y_m': 0.6656854249,
            'upper_wall_stop_distance_m': 1.0,
            'upper_wall_stop_pose_y_m': 3.0,
            'storage_wall_distance_m': 0.3828427125,
            'storage_pose_x_fallback_m': 0.45,
            'front_sensor_offset_m': 0.15,
            'wall_consistency_tolerance_m': 0.30,
            'wall_alignment_timeout_s': 2.0,
            'wall_angle_tolerance_deg': 2.0,
            'wall_alignment_stable_ticks': 4,
            'wall_alignment_gain': 1.2,
            'wall_alignment_max_angular_z': 0.25,
            'leave_start_linear_x': 0.08,
            'leave_start_angular_z': 0.22,
            'leave_start_duration_s': 3.0,
            'navigation_linear_x': 0.12,
            'navigation_min_linear_x': 0.04,
            'search_linear_x': 0.10,
            'return_linear_x': -0.14,
            'storage_linear_x': 0.08,
            'heading_gain': 1.4,
            'navigation_max_angular_z': 0.30,
            'scan_heading_max_angular_z': 0.18,
            'turn_max_angular_z': 0.40,
            'turn_min_angular_z': 0.10,
            'yaw_tolerance_deg': 3.0,
            'position_tolerance_m': 0.05,
            'navigation_timeout_s': 30.0,
            'lane_timeout_s': 75.0,
            'return_timeout_s': 40.0,
            'distance_motion_timeout_s': 10.0,
            'target_classes': '',
            'target_min_confidence': 0.30,
            'target_trigger_area_ratio': 0.008,
            'target_trigger_height_ratio': 0.10,
            'target_center_tolerance': 0.10,
            'target_timeout_s': 0.6,
            'target_action_timeout_s': 15.0,
            'target_align_gain': 0.90,
            'target_align_max_angular_z': 0.40,
            'target_approach_max_linear_x': 0.08,
            'target_approach_min_linear_x': 0.03,
            'target_max_approach_distance_m': 0.90,
            'pickup_bottom_y_ratio': 0.70,
            'final_grab_forward_distance_m': 0.10,
            'final_grab_forward_linear_x': 0.06,
            'target_cooldown_s': 1.0,
            'pickup_capacity': 4,
            'storage_turn_ccw_deg': 45.0,
            'storage_reverse_distance_m': 0.40,
            'gripper_enabled': True,
            'gripper_type': 'bus',
            'gripper_servo_id': 1,
            'gripper_open_position': 500,
            'gripper_closed_position': 750,
            'gripper_move_duration_s': 0.5,
            'gripper_open_dwell_s': 0.7,
            'gripper_close_dwell_s': 0.8,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        topic_names = (
            'cmd_vel_topic',
            'control_topic',
            'state_topic',
            'status_topic',
            'detections_topic',
            'odom_topic',
            'wall_topic',
            'left_range_topic',
            'right_range_topic',
            'pose_reset_service',
            'pwm_servo_topic',
            'bus_servo_topic',
        )
        for name in topic_names:
            setattr(self, name, str(self.get_parameter(name).value))

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.timer_rate_hz = self.get_float('timer_rate_hz')
        self.status_period_s = 1.0 / self.get_float('status_rate_hz')
        self.pose_timeout_s = self.get_float('pose_timeout_s')
        self.wall_timeout_s = self.get_float('wall_timeout_s')
        self.arena_width = self.get_float('arena_width_m')
        self.arena_height = self.get_float('arena_height_m')
        self.lane_x_positions = [
            float(value) for value in self.get_parameter('lane_x_positions').value
        ]
        self.main_road_y = self.get_float('main_road_y_m')
        self.target_classes = parse_class_list(self.get_parameter('target_classes').value)
        self.pickup_capacity = int(self.get_parameter('pickup_capacity').value)

    def _validate_parameters(self):
        if self.timer_rate_hz <= 0.0:
            raise ValueError('timer_rate_hz must be positive')
        if not self.lane_x_positions:
            raise ValueError('lane_x_positions cannot be empty')
        if any(not 0.0 < x < self.arena_width for x in self.lane_x_positions):
            raise ValueError('every search lane must be inside the arena')
        if not 0.0 < self.main_road_y < self.arena_height:
            raise ValueError('main_road_y_m must be inside the arena')
        if self.pickup_capacity <= 0:
            raise ValueError('pickup_capacity must be positive')
        if not 0.0 < self.get_float('pickup_bottom_y_ratio') <= 1.0:
            raise ValueError('pickup_bottom_y_ratio must be in (0, 1]')
        if self.get_float('return_linear_x') >= 0.0:
            raise ValueError('return_linear_x must be negative')
        if self.get_float('search_linear_x') <= 0.0:
            raise ValueError('search_linear_x must be positive')
        if self.get_float('final_grab_forward_distance_m') <= 0.0:
            raise ValueError('final_grab_forward_distance_m must be positive')
        if self.get_float('storage_reverse_distance_m') <= 0.0:
            raise ValueError('storage_reverse_distance_m must be positive')

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        self.pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        self.pose_received_at = self.get_clock().now()

    def wall_callback(self, msg):
        self.wall_distance = float(msg.vector.x)
        self.wall_angle = float(msg.vector.y)
        self.wall_min_distance = float(msg.vector.z)
        self.wall_received_at = self.get_clock().now()

    def left_range_callback(self, msg):
        self.left_range = float(msg.range)
        self.left_range_received_at = self.get_clock().now()

    def right_range_callback(self, msg):
        self.right_range = float(msg.range)
        self.right_range_received_at = self.get_clock().now()

    def detections_callback(self, msg):
        now = self.get_clock().now()
        self.detection_frame_received_at = now
        target = select_target(
            msg.data,
            self.target_classes,
            self.get_float('target_min_confidence'),
            locked_class=self.locked_target_class,
        )
        if target is not None:
            self.latest_target = target
            self.target_received_at = now

    def control_callback(self, msg):
        command = msg.data.strip().lower()
        if command == 'start':
            if self.state in ACTIVE_STATES:
                self.get_logger().warning('Start ignored because the mission is already active.')
                return
            self.reset_mission_progress()
            self.transition(MissionState.LEAVE_START_ARC, 'start command')
        elif command == 'stop':
            self.stop_robot()
            self.transition(MissionState.STOPPED, 'stop command')
        elif command == 'reset':
            self.stop_robot()
            self.reset_mission_progress()
            self.request_pose_reset()
            self.transition(MissionState.IDLE, 'mission and pose reset requested')
        elif command == 'open':
            self.stop_robot()
            self.command_gripper(open_gripper=True)
        elif command == 'close':
            self.stop_robot()
            self.command_gripper(open_gripper=False)
        else:
            self.get_logger().warning(
                f'Unknown mission command {command!r}; use start, stop, reset, open, or close.'
            )

    def reset_mission_progress(self):
        self.lane_index = 0
        self.carried_count = 0
        self.total_pick_attempts = 0
        self.storage_pending = False
        self.pickup_succeeded = False
        self.grab_counted = False
        self.locked_target_class = None
        self.target_checkpoint = None
        self.approach_started_pose = None
        self.motion_started_pose = None
        self.nav_goal = None
        self.nav_next_state = None
        self.turn_goal_yaw = None
        self.turn_next_state = None
        self.lane_heading_yaw = math.pi * 0.5
        self.wall_aligned_ticks = 0
        self.target_cooldown_until_s = 0.0

    def request_pose_reset(self):
        if not self.pose_reset_client.service_is_ready():
            self.get_logger().warning(
                f'Pose reset service {self.pose_reset_service} is not ready.'
            )
            return
        self.pose_reset_client.call_async(Trigger.Request())

    def tick(self):
        if self.state not in ACTIVE_STATES:
            self.stop_robot()
            self.publish_status()
            return

        if not self.pose_is_recent():
            self.stop_robot()
            self.state_reason = 'waiting for recent odometry'
            self.publish_status()
            return

        handlers = {
            MissionState.LEAVE_START_ARC: self.run_leave_start_arc,
            MissionState.NAV_TURN: self.run_nav_turn,
            MissionState.NAV_DRIVE: self.run_nav_drive,
            MissionState.TURN_TO_HEADING: self.run_turn_to_heading,
            MissionState.PREPARE_LANE: self.run_prepare_lane,
            MissionState.ALIGN_LANE_WITH_WALL: self.run_align_lane_with_wall,
            MissionState.SCAN_LANE: self.run_scan_lane,
            MissionState.ALIGN_TARGET: self.run_align_target,
            MissionState.APPROACH_TARGET: self.run_approach_target,
            MissionState.OPEN_GRIPPER: self.run_open_gripper,
            MissionState.FINAL_GRAB_FORWARD: self.run_final_grab_forward,
            MissionState.CLOSE_GRIPPER: self.run_close_gripper,
            MissionState.RETURN_TO_CHECKPOINT: self.run_return_to_checkpoint,
            MissionState.AFTER_TARGET_RETURN: self.run_after_target_return,
            MissionState.RETURN_MAIN_ROAD: self.run_return_main_road,
            MissionState.STORAGE_APPROACH_WALL: self.run_storage_approach_wall,
            MissionState.STORAGE_OPEN: self.run_storage_open,
            MissionState.STORAGE_BACK_OUT: self.run_storage_back_out,
            MissionState.STORAGE_CLOSE: self.run_storage_close,
        }
        handler = handlers.get(self.state)
        if handler is None:
            self.fail(f'no handler for state {self.state.value}')
            return
        handler()
        self.publish_status()

    def run_leave_start_arc(self):
        if self.state_age_s() < self.get_float('leave_start_duration_s'):
            self.publish_cmd_vel(
                self.get_float('leave_start_linear_x'),
                self.get_float('leave_start_angular_z'),
            )
            return
        self.begin_navigation(
            self.lane_x_positions[0],
            self.main_road_y,
            MissionState.PREPARE_LANE,
            'moving to first search-lane entrance',
        )

    def begin_navigation(self, x, y, next_state, reason):
        self.nav_goal = (float(x), float(y))
        self.nav_next_state = next_state
        self.transition(MissionState.NAV_TURN, reason)

    def run_nav_turn(self):
        if self.nav_goal is None or self.nav_next_state is None:
            self.fail('navigation goal is missing')
            return
        distance = math.hypot(self.nav_goal[0] - self.pose.x, self.nav_goal[1] - self.pose.y)
        if distance <= self.get_float('position_tolerance_m'):
            self.transition(self.nav_next_state, 'navigation position reached')
            return
        desired_yaw = math.atan2(
            self.nav_goal[1] - self.pose.y,
            self.nav_goal[0] - self.pose.x,
        )
        error = angular_error(desired_yaw, self.pose.yaw)
        if abs(error) <= self.yaw_tolerance_rad():
            self.transition(MissionState.NAV_DRIVE, 'navigation heading reached')
            return
        if self.state_age_s() > self.get_float('navigation_timeout_s'):
            self.fail('navigation turn timed out')
            return
        self.publish_cmd_vel(angular_z=self.turn_command(error))

    def run_nav_drive(self):
        if self.nav_goal is None or self.nav_next_state is None:
            self.fail('navigation goal is missing')
            return
        dx = self.nav_goal[0] - self.pose.x
        dy = self.nav_goal[1] - self.pose.y
        distance = math.hypot(dx, dy)
        if distance <= self.get_float('position_tolerance_m'):
            self.stop_robot()
            self.transition(self.nav_next_state, 'navigation position reached')
            return
        desired_yaw = math.atan2(dy, dx)
        error = angular_error(desired_yaw, self.pose.yaw)
        if abs(error) > math.radians(30.0):
            self.transition(MissionState.NAV_TURN, 'large navigation heading error')
            return
        if self.state_age_s() > self.get_float('navigation_timeout_s'):
            self.fail('navigation drive timed out')
            return

        maximum = self.get_float('navigation_linear_x')
        minimum = self.get_float('navigation_min_linear_x')
        linear = clamp(0.8 * distance, minimum, maximum)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('navigation_max_angular_z'),
            self.get_float('navigation_max_angular_z'),
        )
        self.publish_cmd_vel(linear, angular)

    def begin_turn(self, yaw, next_state, reason):
        self.turn_goal_yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.turn_next_state = next_state
        self.transition(MissionState.TURN_TO_HEADING, reason)

    def run_turn_to_heading(self):
        if self.turn_goal_yaw is None or self.turn_next_state is None:
            self.fail('turn goal is missing')
            return
        error = angular_error(self.turn_goal_yaw, self.pose.yaw)
        if abs(error) <= self.yaw_tolerance_rad():
            self.stop_robot()
            self.transition(self.turn_next_state, 'heading reached')
            return
        if self.state_age_s() > self.get_float('navigation_timeout_s'):
            self.fail('turn-to-heading timed out')
            return
        self.publish_cmd_vel(angular_z=self.turn_command(error))

    def run_prepare_lane(self):
        self.begin_turn(
            math.pi * 0.5,
            MissionState.ALIGN_LANE_WITH_WALL,
            f'orienting north for lane {self.lane_index + 1}',
        )

    def run_align_lane_with_wall(self):
        if self.top_wall_measurement_is_plausible():
            angle = self.wall_angle
            if abs(angle) <= math.radians(self.get_float('wall_angle_tolerance_deg')):
                self.wall_aligned_ticks += 1
                self.stop_robot()
                if self.wall_aligned_ticks >= int(
                    self.get_parameter('wall_alignment_stable_ticks').value
                ):
                    self.lane_heading_yaw = self.pose.yaw
                    self.transition(
                        MissionState.SCAN_LANE,
                        'lane heading corrected from front wall',
                    )
                return
            self.wall_aligned_ticks = 0
            angular = clamp(
                self.get_float('wall_alignment_gain') * angle,
                -self.get_float('wall_alignment_max_angular_z'),
                self.get_float('wall_alignment_max_angular_z'),
            )
            self.publish_cmd_vel(angular_z=angular)
            return

        if self.state_age_s() >= self.get_float('wall_alignment_timeout_s'):
            self.wall_aligned_ticks = 0
            self.lane_heading_yaw = self.pose.yaw
            self.transition(
                MissionState.SCAN_LANE,
                'wall unavailable or occluded; using IMU heading',
            )
            return
        self.stop_robot()

    def run_scan_lane(self):
        if self.state_age_s() > self.get_float('lane_timeout_s'):
            self.fail('search lane timed out before reaching the main road or upper limit')
            return

        target = self.recent_target()
        if (
            target is not None
            and self.now_seconds() >= self.target_cooldown_until_s
            and target_is_large_enough(
                target,
                self.get_float('target_trigger_area_ratio'),
                self.get_float('target_trigger_height_ratio'),
            )
        ):
            self.target_checkpoint = Pose2D(self.pose.x, self.pose.y, self.pose.yaw)
            self.approach_started_pose = None
            self.locked_target_class = target.class_name
            self.pickup_succeeded = False
            self.grab_counted = False
            self.transition(
                MissionState.ALIGN_TARGET,
                f'large-enough target detected: {target.class_name}',
            )
            return

        if self.upper_wall_limit_reached():
            self.begin_return_to_main_road(storage_pending=False)
            return

        error = angular_error(self.lane_heading_yaw, self.pose.yaw)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('scan_heading_max_angular_z'),
            self.get_float('scan_heading_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('search_linear_x'), angular)

    def run_align_target(self):
        if self.state_age_s() > self.get_float('target_action_timeout_s'):
            self.abort_target('target alignment timed out')
            return
        target = self.recent_target()
        if target is None:
            self.abort_target('target lost during alignment')
            return
        if abs(target.x_error) <= self.get_float('target_center_tolerance'):
            self.stop_robot()
            self.approach_started_pose = Pose2D(self.pose.x, self.pose.y, self.pose.yaw)
            if target.bottom_y_ratio >= self.get_float('pickup_bottom_y_ratio'):
                self.transition(MissionState.OPEN_GRIPPER, 'target centered and close')
            else:
                self.transition(MissionState.APPROACH_TARGET, 'target centered')
            return
        angular = clamp(
            -self.get_float('target_align_gain') * target.x_error,
            -self.get_float('target_align_max_angular_z'),
            self.get_float('target_align_max_angular_z'),
        )
        self.publish_cmd_vel(angular_z=angular)

    def run_approach_target(self):
        if self.state_age_s() > self.get_float('target_action_timeout_s'):
            self.abort_target('target approach timed out')
            return
        target = self.recent_target()
        if target is None:
            self.abort_target('target lost during approach')
            return
        if self.approach_started_pose is None:
            self.approach_started_pose = Pose2D(self.pose.x, self.pose.y, self.pose.yaw)
        if pose_distance(self.pose, self.approach_started_pose) > self.get_float(
            'target_max_approach_distance_m'
        ):
            self.abort_target('maximum target approach distance exceeded')
            return

        center_tolerance = self.get_float('target_center_tolerance')
        if target.bottom_y_ratio >= self.get_float('pickup_bottom_y_ratio'):
            if abs(target.x_error) <= center_tolerance * 1.5:
                self.stop_robot()
                self.transition(MissionState.OPEN_GRIPPER, 'pickup image threshold reached')
                return
            angular = clamp(
                -self.get_float('target_align_gain') * target.x_error,
                -self.get_float('target_align_max_angular_z'),
                self.get_float('target_align_max_angular_z'),
            )
            self.publish_cmd_vel(angular_z=angular)
            return

        max_linear = self.get_float('target_approach_max_linear_x')
        min_linear = self.get_float('target_approach_min_linear_x')
        linear = clamp(max_linear * (1.0 - target.bottom_y_ratio), min_linear, max_linear)
        angular = clamp(
            -self.get_float('target_align_gain') * target.x_error,
            -self.get_float('target_align_max_angular_z'),
            self.get_float('target_align_max_angular_z'),
        )
        self.publish_cmd_vel(linear, angular)

    def run_open_gripper(self):
        self.stop_robot()
        if not self.action_command_sent:
            self.command_gripper(open_gripper=True)
            self.action_command_sent = True
        if self.state_age_s() >= self.get_float('gripper_open_dwell_s'):
            self.motion_started_pose = Pose2D(self.pose.x, self.pose.y, self.pose.yaw)
            self.motion_heading = self.pose.yaw
            self.transition(MissionState.FINAL_GRAB_FORWARD, 'gripper open')

    def run_final_grab_forward(self):
        if self.motion_started_pose is None or self.motion_heading is None:
            self.fail('final pickup motion origin is missing')
            return
        if self.state_age_s() > self.get_float('distance_motion_timeout_s'):
            self.fail('final pickup motion timed out')
            return
        travelled = pose_distance(self.pose, self.motion_started_pose)
        if travelled >= self.get_float('final_grab_forward_distance_m'):
            self.stop_robot()
            self.transition(MissionState.CLOSE_GRIPPER, 'final pickup distance reached')
            return
        error = angular_error(self.motion_heading, self.pose.yaw)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('scan_heading_max_angular_z'),
            self.get_float('scan_heading_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('final_grab_forward_linear_x'), angular)

    def run_close_gripper(self):
        self.stop_robot()
        if not self.action_command_sent:
            self.command_gripper(open_gripper=False)
            self.action_command_sent = True
        if self.state_age_s() < self.get_float('gripper_close_dwell_s'):
            return
        if not self.grab_counted:
            self.carried_count += 1
            self.total_pick_attempts += 1
            self.grab_counted = True
            self.pickup_succeeded = True
            self.get_logger().info(
                f'Pickup attempt counted: carried={self.carried_count}, '
                f'total={self.total_pick_attempts}'
            )
        self.transition(
            MissionState.RETURN_TO_CHECKPOINT,
            'reversing to pre-target search pose',
        )

    def abort_target(self, reason):
        self.stop_robot()
        self.pickup_succeeded = False
        if self.target_checkpoint is None:
            self.fail(f'{reason}; target checkpoint is missing')
            return
        self.transition(MissionState.RETURN_TO_CHECKPOINT, reason)

    def run_return_to_checkpoint(self):
        if self.target_checkpoint is None:
            self.fail('target checkpoint is missing')
            return
        if self.state_age_s() > self.get_float('return_timeout_s'):
            self.fail('return to target checkpoint timed out')
            return
        distance = pose_distance(self.pose, self.target_checkpoint)
        if distance <= self.get_float('position_tolerance_m'):
            self.stop_robot()
            self.begin_turn(
                self.target_checkpoint.yaw,
                MissionState.AFTER_TARGET_RETURN,
                'restoring pre-target search yaw',
            )
            return

        # Face away from the checkpoint and reverse toward it.
        desired_yaw = math.atan2(
            self.pose.y - self.target_checkpoint.y,
            self.pose.x - self.target_checkpoint.x,
        )
        error = angular_error(desired_yaw, self.pose.yaw)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('navigation_max_angular_z'),
            self.get_float('navigation_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('return_linear_x'), angular)

    def run_after_target_return(self):
        succeeded = self.pickup_succeeded
        self.locked_target_class = None
        self.target_checkpoint = None
        self.approach_started_pose = None
        self.motion_started_pose = None
        self.target_cooldown_until_s = self.now_seconds() + self.get_float('target_cooldown_s')
        if succeeded and self.carried_count >= self.pickup_capacity:
            self.begin_return_to_main_road(storage_pending=True)
        else:
            self.transition(MissionState.SCAN_LANE, 'search path restored')

    def begin_return_to_main_road(self, storage_pending):
        self.storage_pending = bool(storage_pending)
        self.transition(
            MissionState.RETURN_MAIN_ROAD,
            'returning straight to the bottom main road',
        )

    def run_return_main_road(self):
        if self.state_age_s() > self.get_float('return_timeout_s'):
            self.fail('main-road return timed out')
            return
        if self.pose.y <= self.main_road_y + self.get_float('position_tolerance_m'):
            self.stop_robot()
            if self.storage_pending:
                self.begin_turn(
                    math.pi,
                    MissionState.STORAGE_APPROACH_WALL,
                    'turning west toward storage',
                )
                return
            if self.lane_index + 1 < len(self.lane_x_positions):
                self.lane_index += 1
                self.begin_navigation(
                    self.lane_x_positions[self.lane_index],
                    self.main_road_y,
                    MissionState.PREPARE_LANE,
                    f'shifting to search lane {self.lane_index + 1}',
                )
                return
            if self.carried_count > 0:
                self.storage_pending = True
                self.begin_turn(
                    math.pi,
                    MissionState.STORAGE_APPROACH_WALL,
                    'all lanes searched; taking remaining objects to storage',
                )
            else:
                self.transition(MissionState.DONE, 'all search lanes completed')
            return

        error = angular_error(self.lane_heading_yaw, self.pose.yaw)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('scan_heading_max_angular_z'),
            self.get_float('scan_heading_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('return_linear_x'), angular)

    def run_storage_approach_wall(self):
        if self.state_age_s() > self.get_float('navigation_timeout_s'):
            self.fail('storage-wall approach timed out')
            return
        reached_by_sensor = (
            self.left_wall_measurement_is_plausible()
            and self.wall_distance <= self.get_float('storage_wall_distance_m')
        )
        reached_by_pose = self.pose.x <= self.get_float('storage_pose_x_fallback_m')
        if reached_by_sensor or reached_by_pose:
            self.stop_robot()
            storage_yaw = self.pose.yaw + math.radians(
                self.get_float('storage_turn_ccw_deg')
            )
            self.begin_turn(
                storage_yaw,
                MissionState.STORAGE_OPEN,
                'storage distance reached; turning counterclockwise',
            )
            return

        heading_error = angular_error(math.pi, self.pose.yaw)
        angular = self.get_float('heading_gain') * heading_error
        if self.left_wall_measurement_is_plausible():
            angular += self.get_float('wall_alignment_gain') * self.wall_angle
        angular = clamp(
            angular,
            -self.get_float('navigation_max_angular_z'),
            self.get_float('navigation_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('storage_linear_x'), angular)

    def run_storage_open(self):
        self.stop_robot()
        if not self.action_command_sent:
            self.command_gripper(open_gripper=True)
            self.action_command_sent = True
        if self.state_age_s() >= self.get_float('gripper_open_dwell_s'):
            self.motion_started_pose = Pose2D(self.pose.x, self.pose.y, self.pose.yaw)
            self.motion_heading = self.pose.yaw
            self.transition(
                MissionState.STORAGE_BACK_OUT,
                'gripper open; reversing away from deposited objects',
            )

    def run_storage_back_out(self):
        if self.motion_started_pose is None or self.motion_heading is None:
            self.fail('storage reverse origin is missing')
            return
        if self.state_age_s() > self.get_float('distance_motion_timeout_s'):
            self.fail('storage reverse motion timed out')
            return
        if pose_distance(self.pose, self.motion_started_pose) >= self.get_float(
            'storage_reverse_distance_m'
        ):
            self.stop_robot()
            self.transition(MissionState.STORAGE_CLOSE, 'storage reverse distance reached')
            return
        error = angular_error(self.motion_heading, self.pose.yaw)
        angular = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('scan_heading_max_angular_z'),
            self.get_float('scan_heading_max_angular_z'),
        )
        self.publish_cmd_vel(self.get_float('return_linear_x'), angular)

    def run_storage_close(self):
        self.stop_robot()
        if not self.action_command_sent:
            self.command_gripper(open_gripper=False)
            self.action_command_sent = True
        if self.state_age_s() >= self.get_float('gripper_close_dwell_s'):
            deposited = self.carried_count
            self.carried_count = 0
            self.transition(
                MissionState.DONE,
                f'storage deposit completed for {deposited} pickup attempts',
            )

    def upper_wall_limit_reached(self):
        if self.pose.y >= self.get_float('upper_wall_stop_pose_y_m'):
            return True
        return (
            self.top_wall_measurement_is_plausible()
            and self.wall_distance <= self.get_float('upper_wall_stop_distance_m')
        )

    def top_wall_measurement_is_plausible(self):
        if not self.wall_is_recent() or self.wall_distance is None:
            return False
        expected = self.arena_height - self.pose.y - self.get_float('front_sensor_offset_m')
        return wall_matches_expected(
            self.wall_distance,
            expected,
            self.get_float('wall_consistency_tolerance_m'),
        )

    def left_wall_measurement_is_plausible(self):
        if not self.wall_is_recent() or self.wall_distance is None:
            return False
        expected = self.pose.x - self.get_float('front_sensor_offset_m')
        return wall_matches_expected(
            self.wall_distance,
            expected,
            self.get_float('wall_consistency_tolerance_m'),
        )

    def recent_target(self):
        if self.latest_target is None or self.target_received_at is None:
            return None
        if self.seconds_since(self.target_received_at) > self.get_float('target_timeout_s'):
            return None
        if self.locked_target_class is not None:
            keys = {
                self.latest_target.class_name.lower(),
                self.latest_target.class_id.lower(),
            }
            if self.locked_target_class.lower() not in keys:
                return None
        return self.latest_target

    def pose_is_recent(self):
        return (
            self.pose is not None
            and self.pose_received_at is not None
            and self.seconds_since(self.pose_received_at) <= self.pose_timeout_s
        )

    def wall_is_recent(self):
        return (
            self.wall_received_at is not None
            and self.seconds_since(self.wall_received_at) <= self.wall_timeout_s
        )

    def transition(self, state, reason):
        self.stop_robot()
        self.state = state
        self.state_started_at = self.get_clock().now()
        self.state_reason = reason
        self.action_command_sent = False
        if state == MissionState.ALIGN_LANE_WITH_WALL:
            self.wall_aligned_ticks = 0
        self.get_logger().info(f'Mission state -> {state.value}: {reason}')
        self.publish_state(force=True)
        self.publish_status(force=True)

    def fail(self, reason):
        self.stop_robot()
        self.get_logger().error(reason)
        self.transition(MissionState.ERROR, reason)

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        if hasattr(self, 'cmd_vel_pub'):
            self.publish_cmd_vel()

    def command_gripper(self, open_gripper):
        if not bool(self.get_parameter('gripper_enabled').value):
            return
        position = int(
            self.get_parameter('gripper_open_position').value
            if open_gripper
            else self.get_parameter('gripper_closed_position').value
        )
        servo_id = int(self.get_parameter('gripper_servo_id').value)
        duration = self.get_float('gripper_move_duration_s')
        gripper_type = str(self.get_parameter('gripper_type').value).strip().lower()
        if gripper_type == 'bus':
            state = BusServoState()
            state.present_id = [1, servo_id]
            state.position = [1, position]
            msg = SetBusServoState()
            msg.duration = duration
            msg.state = [state]
            self.bus_servo_pub.publish(msg)
        elif gripper_type == 'pwm':
            state = PWMServoState()
            state.id = [servo_id]
            state.position = [position]
            msg = SetPWMServoState()
            msg.duration = duration
            msg.state = [state]
            self.pwm_servo_pub.publish(msg)
        else:
            self.fail(f'unsupported gripper_type: {gripper_type}')

    def turn_command(self, error):
        command = clamp(
            self.get_float('heading_gain') * error,
            -self.get_float('turn_max_angular_z'),
            self.get_float('turn_max_angular_z'),
        )
        minimum = self.get_float('turn_min_angular_z')
        if 0.0 < abs(command) < minimum:
            command = math.copysign(minimum, command)
        return command

    def publish_state(self, force=False):
        if not force and self.last_state_published == self.state:
            return
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        self.last_state_published = self.state

    def publish_status(self, force=False):
        now = self.get_clock().now()
        if (
            not force
            and self.last_status_published_at is not None
            and self.seconds_between(now, self.last_status_published_at) < self.status_period_s
        ):
            return
        payload = {
            'state': self.state.value,
            'reason': self.state_reason,
            'lane_index': self.lane_index,
            'lane_number': self.lane_index + 1,
            'lane_count': len(self.lane_x_positions),
            'carried_count': self.carried_count,
            'pickup_attempts': self.total_pick_attempts,
            'pose': None if self.pose is None else {
                'x': self.pose.x,
                'y': self.pose.y,
                'yaw_deg': math.degrees(self.pose.yaw),
            },
            'wall': None if self.wall_distance is None else {
                'distance_m': self.wall_distance,
                'angle_deg': math.degrees(self.wall_angle or 0.0),
                'recent': self.wall_is_recent(),
            },
            'target': None if self.latest_target is None else {
                'class_name': self.latest_target.class_name,
                'confidence': self.latest_target.confidence,
                'x_error': self.latest_target.x_error,
                'bottom_y_ratio': self.latest_target.bottom_y_ratio,
                'area_ratio': self.latest_target.area_ratio,
            },
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.last_status_published_at = now

    def state_age_s(self):
        return self.seconds_since(self.state_started_at)

    def seconds_since(self, older):
        return self.seconds_between(self.get_clock().now(), older)

    @staticmethod
    def seconds_between(newer, older):
        return (newer - older).nanoseconds / 1_000_000_000.0

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def yaw_tolerance_rad(self):
        return math.radians(self.get_float('yaw_tolerance_deg'))

    def get_float(self, name):
        return float(self.get_parameter(name).value)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
