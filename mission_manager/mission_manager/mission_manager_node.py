import json
from enum import Enum
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from std_msgs.msg import String


class MissionState(str, Enum):
    IDLE = 'IDLE'
    LEAVE_START = 'LEAVE_START'
    SEARCH = 'SEARCH'
    ALIGN_TARGET = 'ALIGN_TARGET'
    APPROACH_TARGET = 'APPROACH_TARGET'
    OPEN_GRIPPER = 'OPEN_GRIPPER'
    FINAL_FORWARD = 'FINAL_FORWARD'
    AVOID_TURN = 'AVOID_TURN'
    AVOID_FORWARD = 'AVOID_FORWARD'
    REACQUIRE_TARGET = 'REACQUIRE_TARGET'
    GRAB_OBJECT = 'GRAB_OBJECT'
    BACK_OUT = 'BACK_OUT'
    DONE = 'DONE'
    STOPPED = 'STOPPED'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mission_control_topic', '/mission_control')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('target_object_topic', '/target_object')
        self.declare_parameter('avoid_object_topic', '/avoid_object')
        self.declare_parameter('avoid_objects_topic', '/avoid_objects')
        self.declare_parameter('pwm_servo_topic', '/ros_robot_controller/pwm_servo/set_state')
        self.declare_parameter('bus_servo_topic', '/ros_robot_controller/bus_servo/set_state')

        self.declare_parameter('timer_rate_hz', 20.0)
        self.declare_parameter('leave_start_linear_x', 0.08)
        self.declare_parameter('leave_start_angular_z', 0.0)
        self.declare_parameter('leave_start_duration_s', 1.5)
        self.declare_parameter('search_linear_x', 0.06)
        self.declare_parameter('search_forward_angular_z', 0.10)
        self.declare_parameter('search_forward_duration_s', 1.4)
        self.declare_parameter('search_angular_z', 0.35)
        self.declare_parameter('search_turn_duration_s', 0.9)
        self.declare_parameter('search_turn_direction', 1.0)
        self.declare_parameter('search_alternate_turn_direction', False)
        self.declare_parameter('search_duration_s', 150.0)

        self.declare_parameter('approach_max_linear_x', 0.10)
        self.declare_parameter('approach_min_linear_x', 0.03)
        self.declare_parameter('approach_angular_gain', 0.8)
        self.declare_parameter('approach_max_angular_z', 0.45)
        self.declare_parameter('center_tolerance', 0.12)
        self.declare_parameter('grab_area_ratio', 0.50)
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('final_forward_linear_x', 0.06)
        self.declare_parameter('final_forward_duration_s', 1.6)
        self.declare_parameter('back_out_linear_x', -0.08)
        self.declare_parameter('grab_duration_s', 1.0)
        self.declare_parameter('back_out_duration_s', 1.5)

        self.declare_parameter('avoid_enabled', True)
        self.declare_parameter('avoid_timeout_s', 0.5)
        self.declare_parameter('avoid_area_ratio', 0.38)
        self.declare_parameter('avoid_center_band', 0.75)
        self.declare_parameter('avoid_center_corridor', 0.30)
        self.declare_parameter('avoid_path_margin', 0.30)
        self.declare_parameter('avoid_roi_enabled', True)
        self.declare_parameter('avoid_roi_left_near_x', -0.27)
        self.declare_parameter('avoid_roi_left_near_y', 0.80)
        self.declare_parameter('avoid_roi_left_far_x', -0.75)
        self.declare_parameter('avoid_roi_left_far_y', 0.42)
        self.declare_parameter('avoid_roi_right_near_x', 0.11)
        self.declare_parameter('avoid_roi_right_near_y', 0.80)
        self.declare_parameter('avoid_roi_right_far_x', 0.62)
        self.declare_parameter('avoid_roi_right_far_y', 0.42)
        self.declare_parameter('avoid_emergency_ratio', 0.68)
        self.declare_parameter('avoid_only_if_closer_than_target', True)
        self.declare_parameter('avoid_closer_ratio', 0.85)
        self.declare_parameter('avoid_turn_duration_s', 0.55)
        self.declare_parameter('avoid_turn_angular_z', 0.65)
        self.declare_parameter('avoid_forward_duration_s', 0.85)
        self.declare_parameter('avoid_forward_linear_x', 0.05)
        self.declare_parameter('avoid_forward_angular_z', 0.25)
        self.declare_parameter('avoid_turn_direction_sign', 1.0)
        self.declare_parameter('avoid_vfh_center_weight', 2.0)
        self.declare_parameter('avoid_vfh_target_weight', 0.60)
        self.declare_parameter('avoid_vfh_switch_penalty', 0.25)
        self.declare_parameter('avoid_direction_hold_s', 0.8)
        self.declare_parameter('avoid_ignore_near_target_enabled', True)
        self.declare_parameter('avoid_ignore_target_min_y', 0.35)
        self.declare_parameter('avoid_ignore_target_center_band', 0.25)
        self.declare_parameter('avoid_ignore_target_x_margin', 0.25)
        self.declare_parameter('avoid_ignore_target_y_margin', 0.20)
        self.declare_parameter('reacquire_duration_s', 3.0)
        self.declare_parameter('reacquire_angular_z', 0.30)

        self.declare_parameter('gripper_enabled', True)
        self.declare_parameter('gripper_type', 'pwm')
        self.declare_parameter('gripper_servo_id', 1)
        self.declare_parameter('gripper_open_position', 1500)
        self.declare_parameter('gripper_closed_position', 1000)
        self.declare_parameter('gripper_move_duration_s', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mission_control_topic = self.get_parameter('mission_control_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.target_object_topic = self.get_parameter('target_object_topic').value
        self.avoid_object_topic = self.get_parameter('avoid_object_topic').value
        self.avoid_objects_topic = self.get_parameter('avoid_objects_topic').value
        self.pwm_servo_topic = self.get_parameter('pwm_servo_topic').value
        self.bus_servo_topic = self.get_parameter('bus_servo_topic').value

        self.state = MissionState.IDLE
        self.state_started_at = self.get_clock().now()
        self.last_state_published = None
        self.latest_target = None
        self.latest_target_time = None
        self.latest_avoid = None
        self.latest_avoid_time = None
        self.latest_avoid_objects = []
        self.latest_avoid_objects_time = None
        self.last_target_direction = self.search_direction()
        self.avoid_turn_direction = self.search_direction()
        self.last_avoid_direction_time = None
        self.open_command_sent = False
        self.grab_command_sent = False

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.mission_state_topic, 10)
        self.pwm_servo_pub = self.create_publisher(SetPWMServoState, self.pwm_servo_topic, 10)
        self.bus_servo_pub = self.create_publisher(SetBusServoState, self.bus_servo_topic, 10)
        self.control_sub = self.create_subscription(
            String,
            self.mission_control_topic,
            self.control_callback,
            10,
        )
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_object_topic,
            self.target_callback,
            10,
        )
        self.avoid_sub = self.create_subscription(
            PointStamped,
            self.avoid_object_topic,
            self.avoid_callback,
            10,
        )
        self.avoid_objects_sub = self.create_subscription(
            String,
            self.avoid_objects_topic,
            self.avoid_objects_callback,
            10,
        )

        timer_rate_hz = float(self.get_parameter('timer_rate_hz').value)
        self.timer = self.create_timer(1.0 / timer_rate_hz, self.tick)

        self.get_logger().info(
            f'Mission manager ready. Send commands on {self.mission_control_topic}: '
            'start, demo, search, stop, reset, open, close'
        )
        self.get_logger().info(
            f'Subscribing target={self.target_object_topic}, avoid={self.avoid_object_topic}, '
            f'avoid_objects={self.avoid_objects_topic}; '
            f'publishing cmd_vel={self.cmd_vel_topic}'
        )

    def target_callback(self, msg):
        self.latest_target = msg.point
        self.latest_target_time = self.get_clock().now()

        x_error = float(msg.point.x)
        if abs(x_error) > self.get_float('center_tolerance') * 0.5:
            self.last_target_direction = 1.0 if x_error > 0.0 else -1.0

    def avoid_callback(self, msg):
        self.latest_avoid = self.make_avoid_point(msg.point.x, msg.point.y, msg.point.z, msg.point.y)
        self.latest_avoid_time = self.get_clock().now()

    def avoid_objects_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid avoid_objects JSON: {exc}')
            return

        if isinstance(payload, list):
            raw_objects = payload
        elif isinstance(payload, dict):
            raw_objects = payload.get('objects', [])
        else:
            self.get_logger().warning('avoid_objects JSON must be an object or list.')
            return

        if not isinstance(raw_objects, list):
            self.get_logger().warning("avoid_objects JSON has no list field named 'objects'.")
            return

        points = []
        for raw in raw_objects:
            if not isinstance(raw, dict):
                continue
            try:
                x = float(raw.get('x', raw.get('point_x', 0.0)))
                y = float(raw.get('y', raw.get('point_y', 0.0)))
                center_y = float(
                    raw.get('center_y', raw.get('bbox_center_y', y))
                )
                bottom_y = float(raw.get('bottom_y', y))
                confidence = float(raw.get('confidence', raw.get('z', 1.0)))
            except (TypeError, ValueError):
                continue
            points.append(self.make_avoid_point(x, y, confidence, center_y, bottom_y))

        self.latest_avoid_objects = points
        self.latest_avoid_objects_time = self.get_clock().now()

    def make_avoid_point(self, x, y, confidence, center_y, bottom_y=None):
        return SimpleNamespace(
            x=self.clamp(float(x), -1.0, 1.0),
            y=self.clamp(float(y), 0.0, 1.0),
            z=max(0.0, float(confidence)),
            center_y=self.clamp(float(center_y), 0.0, 1.0),
            bottom_y=self.clamp(
                float(y if bottom_y is None else bottom_y),
                0.0,
                1.0,
            ),
        )

    def control_callback(self, msg):
        command = msg.data.strip().lower()

        if command in ('start', 'demo'):
            self.start_mission()
        elif command == 'search':
            self.change_state(MissionState.SEARCH)
        elif command == 'stop':
            self.stop_robot()
            self.change_state(MissionState.STOPPED)
        elif command == 'reset':
            self.stop_robot()
            self.change_state(MissionState.IDLE)
        elif command == 'open':
            self.command_gripper(open_gripper=True)
        elif command == 'close':
            self.command_gripper(open_gripper=False)
        else:
            self.get_logger().warn(f'Unknown mission command: {msg.data}')

    def start_mission(self):
        self.open_command_sent = False
        self.grab_command_sent = False
        self.command_gripper(open_gripper=False)
        self.change_state(MissionState.LEAVE_START)

    def tick(self):
        self.publish_state()

        if self.state == MissionState.IDLE:
            self.publish_cmd_vel()
        elif self.state == MissionState.LEAVE_START:
            self.run_leave_start()
        elif self.state == MissionState.SEARCH:
            self.run_search()
        elif self.state == MissionState.ALIGN_TARGET:
            self.run_align_target()
        elif self.state == MissionState.APPROACH_TARGET:
            self.run_approach_target()
        elif self.state == MissionState.OPEN_GRIPPER:
            self.run_open_gripper()
        elif self.state == MissionState.FINAL_FORWARD:
            self.run_final_forward()
        elif self.state == MissionState.AVOID_TURN:
            self.run_avoid_turn()
        elif self.state == MissionState.AVOID_FORWARD:
            self.run_avoid_forward()
        elif self.state == MissionState.REACQUIRE_TARGET:
            self.run_reacquire_target()
        elif self.state == MissionState.GRAB_OBJECT:
            self.run_grab()
        elif self.state == MissionState.BACK_OUT:
            self.publish_cmd_vel(linear_x=self.get_float('back_out_linear_x'))
            self.advance_after('back_out_duration_s', MissionState.DONE)
        elif self.state in (MissionState.DONE, MissionState.STOPPED):
            self.publish_cmd_vel()

    def run_leave_start(self):
        if self.begin_avoid_if_needed():
            return
        if self.has_recent_target():
            self.change_state(MissionState.ALIGN_TARGET)
            return

        self.publish_cmd_vel(
            linear_x=self.get_float('leave_start_linear_x'),
            angular_z=self.get_float('leave_start_angular_z'),
        )
        self.advance_after('leave_start_duration_s', MissionState.SEARCH)

    def run_search(self):
        if self.begin_avoid_if_needed():
            return
        if self.has_recent_target():
            self.change_state(MissionState.ALIGN_TARGET)
            return

        self.publish_search_command()
        self.advance_after('search_duration_s', MissionState.STOPPED)

    def publish_search_command(self):
        forward_duration = self.get_float('search_forward_duration_s')
        turn_duration = self.get_float('search_turn_duration_s')
        cycle_duration = max(0.1, forward_duration + turn_duration)
        state_age = self.state_age_s()
        cycle_index = int(state_age / cycle_duration)
        phase_time = state_age % cycle_duration

        direction = self.search_direction()
        if bool(self.get_parameter('search_alternate_turn_direction').value) and cycle_index % 2 == 1:
            direction *= -1.0

        if phase_time < forward_duration:
            self.publish_cmd_vel(
                linear_x=self.get_float('search_linear_x'),
                angular_z=direction * self.get_float('search_forward_angular_z'),
            )
        else:
            self.publish_cmd_vel(angular_z=direction * self.get_float('search_angular_z'))

    def run_align_target(self):
        if self.begin_avoid_if_needed():
            return
        if not self.has_recent_target():
            self.change_state(MissionState.REACQUIRE_TARGET)
            return

        x_error = float(self.latest_target.x)
        if abs(x_error) <= self.get_float('center_tolerance'):
            self.publish_cmd_vel()
            self.change_state(MissionState.APPROACH_TARGET)
            return

        self.publish_cmd_vel(angular_z=self.target_turn_command(x_error))

    def run_approach_target(self):
        if self.begin_avoid_if_needed():
            return
        if not self.has_recent_target():
            self.change_state(MissionState.REACQUIRE_TARGET)
            return

        x_error = float(self.latest_target.x)
        closeness = float(self.latest_target.y)

        if abs(x_error) > self.get_float('center_tolerance'):
            self.change_state(MissionState.ALIGN_TARGET)
            return

        if closeness >= self.get_float('grab_area_ratio'):
            self.publish_cmd_vel()
            self.change_state(MissionState.OPEN_GRIPPER)
            return

        closeness_scale = max(0.0, min(1.0, closeness / self.get_float('grab_area_ratio')))
        max_linear = self.get_float('approach_max_linear_x')
        min_linear = self.get_float('approach_min_linear_x')
        linear_x = max(min_linear, max_linear * (1.0 - closeness_scale))
        angular_z = self.target_turn_command(x_error)
        self.publish_cmd_vel(linear_x=linear_x, angular_z=angular_z)

    def run_open_gripper(self):
        self.publish_cmd_vel()
        if not self.open_command_sent:
            self.command_gripper(open_gripper=True)
            self.open_command_sent = True
        self.advance_after('gripper_move_duration_s', MissionState.FINAL_FORWARD)

    def run_final_forward(self):
        self.publish_cmd_vel(linear_x=self.get_float('final_forward_linear_x'))
        self.advance_after('final_forward_duration_s', MissionState.GRAB_OBJECT)

    def run_avoid_turn(self):
        self.publish_cmd_vel(angular_z=self.avoid_turn_direction * self.get_float('avoid_turn_angular_z'))
        self.advance_after('avoid_turn_duration_s', MissionState.AVOID_FORWARD)

    def run_avoid_forward(self):
        self.publish_cmd_vel(
            linear_x=self.get_float('avoid_forward_linear_x'),
            angular_z=self.avoid_turn_direction * self.get_float('avoid_forward_angular_z'),
        )
        self.advance_after('avoid_forward_duration_s', MissionState.REACQUIRE_TARGET)

    def run_reacquire_target(self):
        if self.begin_avoid_if_needed():
            return
        if self.has_recent_target():
            self.change_state(MissionState.ALIGN_TARGET)
            return

        self.publish_cmd_vel(angular_z=self.last_target_direction * self.get_float('reacquire_angular_z'))
        self.advance_after('reacquire_duration_s', MissionState.SEARCH)

    def begin_avoid_if_needed(self):
        if not self.should_avoid():
            return False

        self.avoid_turn_direction = self.compute_avoid_turn_direction()
        self.last_avoid_direction_time = self.get_clock().now()
        self.change_state(MissionState.AVOID_TURN)
        return True

    def should_avoid(self):
        if not bool(self.get_parameter('avoid_enabled').value):
            return False
        avoid_points = self.recent_avoid_points()
        if not avoid_points:
            return False

        for point in avoid_points:
            closeness = float(point.y)
            if not self.avoid_is_inside_roi(point):
                continue
            if self.avoid_matches_locked_target(point):
                continue
            if closeness >= self.get_float('avoid_emergency_ratio'):
                return True
            if closeness < self.get_float('avoid_area_ratio'):
                continue
            if not self.avoid_is_closer_than_target(closeness):
                continue
            return True
        return False

    def avoid_is_closer_than_target(self, avoid_closeness):
        if not bool(self.get_parameter('avoid_only_if_closer_than_target').value):
            return True
        if not self.has_recent_target():
            return True

        target_closeness = float(self.latest_target.y)
        return avoid_closeness >= target_closeness * self.get_float('avoid_closer_ratio')

    def compute_avoid_turn_direction(self):
        avoid_points = self.recent_avoid_points()
        if not avoid_points:
            return self.search_direction()

        bins = self.build_avoid_histogram(avoid_points)
        left_cost = (
            bins[0] * 1.00
            + bins[1] * 0.70
            + bins[2] * 0.45
            + bins[3] * 0.15
        )
        right_cost = (
            bins[4] * 1.00
            + bins[3] * 0.70
            + bins[2] * 0.45
            + bins[1] * 0.15
        )

        target_preference = self.target_direction_preference()
        if target_preference > 0.0:
            right_cost += self.get_float('avoid_vfh_target_weight')
        elif target_preference < 0.0:
            left_cost += self.get_float('avoid_vfh_target_weight')

        left_cost += self.direction_switch_cost(1.0)
        right_cost += self.direction_switch_cost(-1.0)

        direction = 1.0 if left_cost <= right_cost else -1.0
        direction *= self.get_float('avoid_turn_direction_sign')
        return 1.0 if direction >= 0.0 else -1.0

    def build_avoid_histogram(self, avoid_points):
        bins = [0.0, 0.0, 0.0, 0.0, 0.0]
        min_closeness = self.get_float('avoid_area_ratio')
        center_band = max(0.05, self.get_float('avoid_center_band'))
        center_weight = self.get_float('avoid_vfh_center_weight')

        for point in avoid_points:
            x = self.clamp(float(point.x), -1.0, 1.0)
            closeness = self.clamp(float(point.y), 0.0, 1.0)
            if closeness < min_closeness:
                continue
            if not self.avoid_is_inside_roi(point):
                continue
            if self.avoid_matches_locked_target(point):
                continue

            confidence = self.clamp(float(point.z), 0.0, 1.0)
            centered = max(0.0, 1.0 - abs(x) / center_band)
            danger = (closeness * closeness) * (0.5 + 0.5 * confidence) * (1.0 + center_weight * centered)
            bin_index = min(4, max(0, int((x + 1.0) * 2.5)))
            bins[bin_index] += danger

        return bins

    def avoid_is_inside_roi(self, point):
        if not bool(self.get_parameter('avoid_roi_enabled').value):
            return True

        x = self.clamp(float(point.x), -1.0, 1.0)
        y = self.clamp(float(getattr(point, 'center_y', point.y)), 0.0, 1.0)

        left_far_x = self.get_float('avoid_roi_left_far_x')
        left_far_y = self.get_float('avoid_roi_left_far_y')
        left_near_x = self.get_float('avoid_roi_left_near_x')
        left_near_y = self.get_float('avoid_roi_left_near_y')
        right_far_x = self.get_float('avoid_roi_right_far_x')
        right_far_y = self.get_float('avoid_roi_right_far_y')
        right_near_x = self.get_float('avoid_roi_right_near_x')
        right_near_y = self.get_float('avoid_roi_right_near_y')

        min_y = min(left_far_y, left_near_y, right_far_y, right_near_y)
        max_y = max(left_far_y, left_near_y, right_far_y, right_near_y)
        if y < min_y or y > max_y:
            return False

        left_x = self.interpolate_roi_x(y, left_far_x, left_far_y, left_near_x, left_near_y)
        right_x = self.interpolate_roi_x(y, right_far_x, right_far_y, right_near_x, right_near_y)
        if left_x > right_x:
            left_x, right_x = right_x, left_x
        return left_x <= x <= right_x

    @staticmethod
    def interpolate_roi_x(y, x1, y1, x2, y2):
        if abs(y2 - y1) < 1e-6:
            return (x1 + x2) * 0.5
        t = (y - y1) / (y2 - y1)
        return x1 + t * (x2 - x1)

    def avoid_matches_locked_target(self, point):
        if not bool(self.get_parameter('avoid_ignore_near_target_enabled').value):
            return False
        if not self.has_recent_target():
            return False

        target_x = float(self.latest_target.x)
        target_y = float(self.latest_target.y)
        if target_y < self.get_float('avoid_ignore_target_min_y'):
            return False
        if abs(target_x) > self.get_float('avoid_ignore_target_center_band'):
            return False

        avoid_x = float(point.x)
        avoid_y = float(point.y)
        if abs(avoid_x - target_x) > self.get_float('avoid_ignore_target_x_margin'):
            return False
        return abs(avoid_y - target_y) <= self.get_float('avoid_ignore_target_y_margin')

    def avoid_is_on_active_path(self, point, closeness):
        x = float(point.x)
        if abs(x) <= self.get_float('avoid_center_corridor'):
            return True
        if closeness >= self.get_float('avoid_emergency_ratio'):
            return True
        if not self.has_recent_target():
            return True

        target_x = float(self.latest_target.x)
        margin = self.get_float('avoid_path_margin')
        left_limit = min(0.0, target_x) - margin
        right_limit = max(0.0, target_x) + margin
        return left_limit <= x <= right_limit

    def target_direction_preference(self):
        if not self.has_recent_target():
            return 0.0

        x_error = float(self.latest_target.x)
        if abs(x_error) <= self.get_float('center_tolerance'):
            return 0.0
        return -1.0 if x_error > 0.0 else 1.0

    def direction_switch_cost(self, candidate_direction):
        if self.last_avoid_direction_time is None:
            return 0.0

        elapsed = self.get_clock().now() - self.last_avoid_direction_time
        if elapsed.nanoseconds / 1_000_000_000.0 > self.get_float('avoid_direction_hold_s'):
            return 0.0
        signed_candidate = candidate_direction * self.get_float('avoid_turn_direction_sign')
        signed_candidate = 1.0 if signed_candidate >= 0.0 else -1.0
        if signed_candidate == self.avoid_turn_direction:
            return 0.0
        return self.get_float('avoid_vfh_switch_penalty')

    def target_turn_command(self, x_error):
        angular_z = -self.get_float('approach_angular_gain') * x_error
        return self.clamp(
            angular_z,
            -self.get_float('approach_max_angular_z'),
            self.get_float('approach_max_angular_z'),
        )

    def run_grab(self):
        self.publish_cmd_vel()
        if not self.grab_command_sent:
            self.command_gripper(open_gripper=False)
            self.grab_command_sent = True
        self.advance_after('grab_duration_s', MissionState.DONE)

    def advance_after(self, duration_param, next_state):
        if self.state_age_s() >= self.get_float(duration_param):
            self.change_state(next_state)

    def has_recent_target(self):
        if self.latest_target is None or self.latest_target_time is None:
            return False
        elapsed = self.get_clock().now() - self.latest_target_time
        return elapsed.nanoseconds / 1_000_000_000.0 <= self.get_float('target_timeout_s')

    def has_recent_avoid(self):
        if self.latest_avoid is None or self.latest_avoid_time is None:
            return False
        elapsed = self.get_clock().now() - self.latest_avoid_time
        return elapsed.nanoseconds / 1_000_000_000.0 <= self.get_float('avoid_timeout_s')

    def has_recent_avoid_objects(self):
        if self.latest_avoid_objects_time is None:
            return False
        elapsed = self.get_clock().now() - self.latest_avoid_objects_time
        return elapsed.nanoseconds / 1_000_000_000.0 <= self.get_float('avoid_timeout_s')

    def recent_avoid_points(self):
        if self.has_recent_avoid_objects():
            return list(self.latest_avoid_objects)
        if self.has_recent_avoid():
            return [self.latest_avoid]
        return []

    def change_state(self, next_state):
        if self.state == next_state:
            return

        self.state = next_state
        self.state_started_at = self.get_clock().now()

        if next_state == MissionState.OPEN_GRIPPER:
            self.open_command_sent = False
        if next_state == MissionState.GRAB_OBJECT:
            self.grab_command_sent = False

        self.get_logger().info(f'Mission state -> {self.state.value}')
        self.publish_state(force=True)

    def publish_state(self, force=False):
        if not force and self.last_state_published == self.state:
            return

        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        self.last_state_published = self.state

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd_vel()

    def command_gripper(self, open_gripper):
        if not bool(self.get_parameter('gripper_enabled').value):
            action = 'open' if open_gripper else 'close'
            self.get_logger().info(f'Gripper {action} skipped because gripper_enabled is false')
            return

        gripper_type = str(self.get_parameter('gripper_type').value).strip().lower()
        position = int(
            self.get_parameter('gripper_open_position').value
            if open_gripper
            else self.get_parameter('gripper_closed_position').value
        )

        if gripper_type == 'bus':
            self.command_bus_gripper(position)
        elif gripper_type == 'pwm':
            self.command_pwm_gripper(position)
        else:
            self.get_logger().warn(f'Unknown gripper_type: {gripper_type}')

    def command_pwm_gripper(self, position):
        state = PWMServoState()
        state.id = [int(self.get_parameter('gripper_servo_id').value)]
        state.position = [int(position)]

        msg = SetPWMServoState()
        msg.duration = self.get_float('gripper_move_duration_s')
        msg.state = [state]
        self.pwm_servo_pub.publish(msg)

    def command_bus_gripper(self, position):
        state = BusServoState()
        state.present_id = [1, int(self.get_parameter('gripper_servo_id').value)]
        state.position = [1, int(position)]

        msg = SetBusServoState()
        msg.duration = self.get_float('gripper_move_duration_s')
        msg.state = [state]
        self.bus_servo_pub.publish(msg)

    def state_age_s(self):
        elapsed = self.get_clock().now() - self.state_started_at
        return elapsed.nanoseconds / 1_000_000_000.0

    def search_direction(self):
        return 1.0 if self.get_float('search_turn_direction') >= 0.0 else -1.0

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
