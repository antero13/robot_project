from enum import Enum

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from std_msgs.msg import String


class MissionState(str, Enum):
    IDLE = 'IDLE'
    LEAVE_START = 'LEAVE_START'
    SEARCH_OBJECT = 'SEARCH_OBJECT'
    APPROACH_OBJECT = 'APPROACH_OBJECT'
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
        self.declare_parameter('grab_area_ratio', 0.10)
        self.declare_parameter('target_timeout_s', 1.5)
        self.declare_parameter('back_out_linear_x', -0.08)
        self.declare_parameter('grab_duration_s', 1.0)
        self.declare_parameter('back_out_duration_s', 1.5)

        self.declare_parameter('gripper_enabled', False)
        self.declare_parameter('gripper_type', 'pwm')
        self.declare_parameter('gripper_servo_id', 1)
        self.declare_parameter('gripper_open_position', 1500)
        self.declare_parameter('gripper_closed_position', 1000)
        self.declare_parameter('gripper_move_duration_s', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mission_control_topic = self.get_parameter('mission_control_topic').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.target_object_topic = self.get_parameter('target_object_topic').value
        self.pwm_servo_topic = self.get_parameter('pwm_servo_topic').value
        self.bus_servo_topic = self.get_parameter('bus_servo_topic').value

        self.state = MissionState.IDLE
        self.state_started_at = self.get_clock().now()
        self.last_state_published = None
        self.latest_target = None
        self.latest_target_time = None
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

        timer_rate_hz = float(self.get_parameter('timer_rate_hz').value)
        self.timer = self.create_timer(1.0 / timer_rate_hz, self.tick)

        self.get_logger().info(
            f'Mission manager ready. Send commands on {self.mission_control_topic}: '
            'start, demo, search, stop, reset, open, close'
        )

    def target_callback(self, msg):
        self.latest_target = msg.point
        self.latest_target_time = self.get_clock().now()

    def control_callback(self, msg):
        command = msg.data.strip().lower()

        if command in ('start', 'demo'):
            self.start_mission()
        elif command == 'search':
            self.change_state(MissionState.SEARCH_OBJECT)
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
        self.grab_command_sent = False
        self.command_gripper(open_gripper=True)
        self.change_state(MissionState.LEAVE_START)

    def tick(self):
        self.publish_state()

        if self.state == MissionState.IDLE:
            self.publish_cmd_vel()
        elif self.state == MissionState.LEAVE_START:
            self.run_leave_start()
        elif self.state == MissionState.SEARCH_OBJECT:
            self.run_search()
        elif self.state == MissionState.APPROACH_OBJECT:
            self.run_approach()
        elif self.state == MissionState.GRAB_OBJECT:
            self.run_grab()
        elif self.state == MissionState.BACK_OUT:
            self.publish_cmd_vel(linear_x=self.get_float('back_out_linear_x'))
            self.advance_after('back_out_duration_s', MissionState.DONE)
        elif self.state in (MissionState.DONE, MissionState.STOPPED):
            self.publish_cmd_vel()

    def run_leave_start(self):
        if self.has_recent_target():
            self.change_state(MissionState.APPROACH_OBJECT)
            return

        self.publish_cmd_vel(
            linear_x=self.get_float('leave_start_linear_x'),
            angular_z=self.get_float('leave_start_angular_z'),
        )
        self.advance_after('leave_start_duration_s', MissionState.SEARCH_OBJECT)

    def run_search(self):
        if self.has_recent_target():
            self.change_state(MissionState.APPROACH_OBJECT)
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

        direction = 1.0 if self.get_float('search_turn_direction') >= 0.0 else -1.0
        if bool(self.get_parameter('search_alternate_turn_direction').value) and cycle_index % 2 == 1:
            direction *= -1.0

        if phase_time < forward_duration:
            self.publish_cmd_vel(
                linear_x=self.get_float('search_linear_x'),
                angular_z=direction * self.get_float('search_forward_angular_z'),
            )
        else:
            self.publish_cmd_vel(angular_z=direction * self.get_float('search_angular_z'))

    def run_approach(self):
        if not self.has_recent_target():
            self.change_state(MissionState.SEARCH_OBJECT)
            return

        x_error = float(self.latest_target.x)
        area_ratio = float(self.latest_target.y)

        if area_ratio >= self.get_float('grab_area_ratio') and abs(x_error) <= self.get_float('center_tolerance'):
            self.publish_cmd_vel()
            self.change_state(MissionState.GRAB_OBJECT)
            return

        angular_z = -self.get_float('approach_angular_gain') * x_error
        angular_z = self.clamp(
            angular_z,
            -self.get_float('approach_max_angular_z'),
            self.get_float('approach_max_angular_z'),
        )

        if abs(x_error) > self.get_float('center_tolerance'):
            linear_x = 0.0
        else:
            area_scale = max(0.0, min(1.0, area_ratio / self.get_float('grab_area_ratio')))
            max_linear = self.get_float('approach_max_linear_x')
            min_linear = self.get_float('approach_min_linear_x')
            linear_x = max(min_linear, max_linear * (1.0 - area_scale))

        self.publish_cmd_vel(linear_x=linear_x, angular_z=angular_z)

    def run_grab(self):
        self.publish_cmd_vel()
        if not self.grab_command_sent:
            self.command_gripper(open_gripper=False)
            self.grab_command_sent = True
        self.advance_after('grab_duration_s', MissionState.BACK_OUT)

    def advance_after(self, duration_param, next_state):
        if self.state_age_s() >= self.get_float(duration_param):
            self.change_state(next_state)

    def has_recent_target(self):
        if self.latest_target is None or self.latest_target_time is None:
            return False
        elapsed = self.get_clock().now() - self.latest_target_time
        return elapsed.nanoseconds / 1_000_000_000.0 <= self.get_float('target_timeout_s')

    def change_state(self, next_state):
        if self.state == next_state:
            return

        self.state = next_state
        self.state_started_at = self.get_clock().now()

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
