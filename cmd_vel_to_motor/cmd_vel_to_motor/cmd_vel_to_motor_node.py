import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorState, MotorsState


class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_topic', '/ros_robot_controller/set_motor')
        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheel_separation_m', 0.18)
        self.declare_parameter('left_motor_ids', [4, 3])
        self.declare_parameter('right_motor_ids', [2, 1])
        self.declare_parameter('left_motor_signs', [1.0, 1.0])
        self.declare_parameter('right_motor_signs', [-1.0, -1.0])
        self.declare_parameter('max_rps', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_s', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.motor_topic = self.get_parameter('motor_topic').value
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
        self.left_motor_ids = list(self.get_parameter('left_motor_ids').value)
        self.right_motor_ids = list(self.get_parameter('right_motor_ids').value)
        self.left_motor_signs = [float(v) for v in self.get_parameter('left_motor_signs').value]
        self.right_motor_signs = [float(v) for v in self.get_parameter('right_motor_signs').value]
        self.max_rps = float(self.get_parameter('max_rps').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)

        self._validate_parameters()

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.motor_pub = self.create_publisher(MotorsState, self.motor_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_motor_state)

        self.get_logger().info(
            f'Listening on {self.cmd_vel_topic}, publishing motor commands to {self.motor_topic}'
        )

    def _validate_parameters(self):
        if self.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be greater than 0')
        if self.wheel_separation_m <= 0.0:
            raise ValueError('wheel_separation_m must be greater than 0')
        if len(self.left_motor_ids) != len(self.left_motor_signs):
            raise ValueError('left_motor_ids and left_motor_signs must have the same length')
        if len(self.right_motor_ids) != len(self.right_motor_signs):
            raise ValueError('right_motor_ids and right_motor_signs must have the same length')
        if self.max_rps <= 0.0:
            raise ValueError('max_rps must be greater than 0')

    def cmd_vel_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_motor_state(self):
        if self._is_command_timed_out():
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self.last_cmd.linear.x
            angular_z = self.last_cmd.angular.z

        left_rps, right_rps = self._twist_to_rps(linear_x, angular_z)

        msg = MotorsState()
        msg.data = []

        for motor_id, sign in zip(self.left_motor_ids, self.left_motor_signs):
            msg.data.append(self._make_motor_state(motor_id, sign * left_rps))

        for motor_id, sign in zip(self.right_motor_ids, self.right_motor_signs):
            msg.data.append(self._make_motor_state(motor_id, sign * right_rps))

        self.motor_pub.publish(msg)

    def _is_command_timed_out(self):
        age = self.get_clock().now() - self.last_cmd_time
        return age.nanoseconds / 1_000_000_000.0 > self.command_timeout_s

    def _twist_to_rps(self, linear_x, angular_z):
        half_width = self.wheel_separation_m / 2.0
        left_mps = linear_x - angular_z * half_width
        right_mps = linear_x + angular_z * half_width

        wheel_circumference_m = 2.0 * math.pi * self.wheel_radius_m
        left_rps = left_mps / wheel_circumference_m
        right_rps = right_mps / wheel_circumference_m

        return self._clamp(left_rps), self._clamp(right_rps)

    def _clamp(self, value):
        return max(-self.max_rps, min(self.max_rps, value))

    @staticmethod
    def _make_motor_state(motor_id, rps):
        state = MotorState()
        state.id = int(motor_id)
        state.rps = float(rps)
        return state


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotor()

    try:
        rclpy.spin(node)
    finally:
        stop_msg = MotorsState()
        for motor_id in [*node.left_motor_ids, *node.right_motor_ids]:
            stop_msg.data.append(node._make_motor_state(motor_id, 0.0))
        node.motor_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
