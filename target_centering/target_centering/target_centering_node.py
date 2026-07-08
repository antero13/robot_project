from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node


class TargetCentering(Node):
    def __init__(self):
        super().__init__('target_centering')

        self.declare_parameter('target_topic', '/target_object')
        self.declare_parameter('avoid_topic', '/avoid_object')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('center_tolerance', 0.08)
        self.declare_parameter('angular_kp', 0.8)
        self.declare_parameter('angular_kd', 0.08)
        self.declare_parameter('max_angular_z', 0.45)
        self.declare_parameter('min_angular_z', 0.12)
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('min_consecutive_detections', 2)
        self.declare_parameter('lost_hold_s', 0.2)
        self.declare_parameter('avoid_enabled', True)
        self.declare_parameter('avoid_timeout_s', 0.5)
        self.declare_parameter('avoid_area_ratio', 0.60)
        self.declare_parameter('avoid_center_band', 0.65)
        self.declare_parameter('avoid_angular_z', 0.35)
        self.declare_parameter('avoid_only_if_closer_than_target', True)
        self.declare_parameter('avoid_closer_ratio', 1.05)
        self.declare_parameter('search_when_lost', False)
        self.declare_parameter('search_angular_z', 0.25)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.target_topic = self.get_parameter('target_topic').value
        self.avoid_topic = self.get_parameter('avoid_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.latest_target = None
        self.latest_target_time = None
        self.latest_avoid = None
        self.latest_avoid_time = None
        self.filtered_error = 0.0
        self.error_buffer = deque(maxlen=max(1, int(self.get_parameter('filter_window_size').value)))
        self.consecutive_detections = 0
        self.previous_error = None
        self.previous_error_time = None
        self.last_cmd = Twist()
        self.last_seen_time = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self.target_callback,
            10,
        )
        self.avoid_sub = self.create_subscription(
            PointStamped,
            self.avoid_topic,
            self.avoid_callback,
            10,
        )

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f'Target centering ready. Subscribing {self.target_topic} and {self.avoid_topic}, '
            f'publishing {self.cmd_vel_topic}'
        )

    def target_callback(self, msg):
        self.latest_target = msg.point
        self.latest_target_time = self.get_clock().now()
        self.last_seen_time = self.latest_target_time

        self.error_buffer.append(float(msg.point.x))
        self.filtered_error = sum(self.error_buffer) / len(self.error_buffer)
        self.consecutive_detections += 1

    def avoid_callback(self, msg):
        self.latest_avoid = msg.point
        self.latest_avoid_time = self.get_clock().now()

    def tick(self):
        cmd = Twist()

        if self.should_avoid():
            self.reset_error_history()
            avoid_cmd = self.make_avoid_command()
            self.publish_cmd(avoid_cmd)
            return

        if not self.has_recent_target():
            self.consecutive_detections = 0
            if self.should_hold_last_command():
                self.cmd_pub.publish(self.last_cmd)
                return

            self.reset_tracking()
            if bool(self.get_parameter('search_when_lost').value):
                cmd.angular.z = self.get_float('search_angular_z')
            self.publish_cmd(cmd)
            return

        if self.consecutive_detections < int(self.get_parameter('min_consecutive_detections').value):
            self.publish_cmd(cmd)
            return

        x_error = self.filtered_error
        if abs(x_error) <= self.get_float('center_tolerance'):
            self.reset_error_history()
            self.publish_cmd(cmd)
            return

        angular_z = -self.compute_pd_output(x_error)
        angular_z = self.clamp(
            angular_z,
            -self.get_float('max_angular_z'),
            self.get_float('max_angular_z'),
        )
        angular_z = self.apply_min_speed(angular_z)

        cmd.angular.z = angular_z
        self.publish_cmd(cmd)

    def compute_pd_output(self, error):
        now = self.get_clock().now()
        derivative = 0.0

        if self.previous_error is not None and self.previous_error_time is not None:
            dt = (now - self.previous_error_time).nanoseconds / 1_000_000_000.0
            if dt > 0.0:
                derivative = (error - self.previous_error) / dt

        self.previous_error = error
        self.previous_error_time = now

        return self.get_float('angular_kp') * error + self.get_float('angular_kd') * derivative

    def reset_error_history(self):
        self.previous_error = None
        self.previous_error_time = None

    def reset_tracking(self):
        self.latest_target = None
        self.latest_target_time = None
        self.last_seen_time = None
        self.error_buffer.clear()
        self.filtered_error = 0.0
        self.consecutive_detections = 0
        self.reset_error_history()

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

    def should_avoid(self):
        if not bool(self.get_parameter('avoid_enabled').value):
            return False
        if not self.has_recent_avoid():
            return False

        x_error = abs(float(self.latest_avoid.x))
        closeness = float(self.latest_avoid.y)
        if closeness < self.get_float('avoid_area_ratio'):
            return False
        if x_error > self.get_float('avoid_center_band'):
            return False
        if not self.avoid_is_closer_than_target(closeness):
            return False
        return True

    def avoid_is_closer_than_target(self, avoid_closeness):
        if not bool(self.get_parameter('avoid_only_if_closer_than_target').value):
            return True
        if not self.has_recent_target():
            return True

        target_closeness = float(self.latest_target.y)
        return avoid_closeness >= target_closeness * self.get_float('avoid_closer_ratio')

    def make_avoid_command(self):
        cmd = Twist()
        avoid_x = float(self.latest_avoid.x)
        turn_direction = -1.0 if avoid_x >= 0.0 else 1.0
        cmd.angular.z = turn_direction * self.get_float('avoid_angular_z')
        return cmd

    def should_hold_last_command(self):
        if self.last_seen_time is None:
            return False
        elapsed = self.get_clock().now() - self.last_seen_time
        elapsed_s = elapsed.nanoseconds / 1_000_000_000.0
        return elapsed_s <= self.get_float('target_timeout_s') + self.get_float('lost_hold_s')

    def publish_cmd(self, cmd):
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def apply_min_speed(self, angular_z):
        min_speed = self.get_float('min_angular_z')
        if abs(angular_z) < min_speed:
            return min_speed if angular_z > 0.0 else -min_speed
        return angular_z

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    node = TargetCentering()

    try:
        rclpy.spin(node)
    finally:
        stop = Twist()
        node.publish_cmd(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
