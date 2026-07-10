import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class MotionMode:
    IDLE = 'IDLE'
    DRIVE = 'DRIVE'
    TURN = 'TURN'


class TimedMotionNode(Node):
    def __init__(self):
        super().__init__('timed_motion')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('drive_distance_topic', '/drive_distance')
        self.declare_parameter('turn_angle_topic', '/turn_angle')
        self.declare_parameter('linear_speed_mps', 0.10)
        self.declare_parameter('angular_speed_radps', 0.50)
        self.declare_parameter('distance_scale', 1.0)
        self.declare_parameter('angle_scale', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('stop_publish_count', 5)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.drive_sub = self.create_subscription(
            Float32,
            self.get_parameter('drive_distance_topic').value,
            self.drive_distance_callback,
            10,
        )
        self.turn_sub = self.create_subscription(
            Float32,
            self.get_parameter('turn_angle_topic').value,
            self.turn_angle_callback,
            10,
        )

        self.mode = MotionMode.IDLE
        self.command_started_at = self.get_clock().now()
        self.command_duration_s = 0.0
        self.command_linear_x = 0.0
        self.command_angular_z = 0.0
        self.stop_messages_remaining = 0

        publish_rate_hz = max(1.0, self.get_float('publish_rate_hz'))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.tick)

        self.get_logger().info(
            'Timed motion ready. Publish meters to /drive_distance or radians to /turn_angle.'
        )

    def drive_distance_callback(self, msg):
        distance_m = float(msg.data)
        speed = abs(self.get_float('linear_speed_mps'))
        if speed <= 0.0:
            self.get_logger().error('linear_speed_mps must be greater than 0')
            return

        duration_s = abs(distance_m) / speed * self.get_float('distance_scale')
        direction = 1.0 if distance_m >= 0.0 else -1.0
        self.start_command(MotionMode.DRIVE, duration_s, direction * speed, 0.0)
        self.get_logger().info(
            f'Drive request: distance={distance_m:.3f}m, speed={direction * speed:.3f}m/s, '
            f'duration={duration_s:.3f}s'
        )

    def turn_angle_callback(self, msg):
        angle_rad = float(msg.data)
        speed = abs(self.get_float('angular_speed_radps'))
        if speed <= 0.0:
            self.get_logger().error('angular_speed_radps must be greater than 0')
            return

        duration_s = abs(angle_rad) / speed * self.get_float('angle_scale')
        direction = 1.0 if angle_rad >= 0.0 else -1.0
        self.start_command(MotionMode.TURN, duration_s, 0.0, direction * speed)
        self.get_logger().info(
            f'Turn request: angle={angle_rad:.3f}rad ({math.degrees(angle_rad):.1f}deg), '
            f'speed={direction * speed:.3f}rad/s, duration={duration_s:.3f}s'
        )

    def start_command(self, mode, duration_s, linear_x, angular_z):
        self.mode = mode
        self.command_started_at = self.get_clock().now()
        self.command_duration_s = max(0.0, float(duration_s))
        self.command_linear_x = float(linear_x)
        self.command_angular_z = float(angular_z)
        self.stop_messages_remaining = 0

    def tick(self):
        if self.mode == MotionMode.IDLE:
            if self.stop_messages_remaining > 0:
                self.publish_cmd_vel()
                self.stop_messages_remaining -= 1
            return

        if self.command_age_s() >= self.command_duration_s:
            self.stop_motion()
            return

        self.publish_cmd_vel(
            linear_x=self.command_linear_x,
            angular_z=self.command_angular_z,
        )

    def stop_motion(self):
        finished_mode = self.mode
        self.mode = MotionMode.IDLE
        self.publish_cmd_vel()
        self.stop_messages_remaining = max(0, int(self.get_parameter('stop_publish_count').value) - 1)
        self.get_logger().info(f'{finished_mode} complete; stopping.')

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def command_age_s(self):
        elapsed = self.get_clock().now() - self.command_started_at
        return elapsed.nanoseconds / 1_000_000_000.0

    def get_float(self, name):
        return float(self.get_parameter(name).value)


def main(args=None):
    rclpy.init(args=args)
    node = TimedMotionNode()

    try:
        rclpy.spin(node)
    finally:
        node.publish_cmd_vel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
