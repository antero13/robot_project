import json
import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

from wall_distance_sensor.wall_geometry import calculate_wall_measurement


class WallDistanceAggregatorNode(Node):
    def __init__(self):
        super().__init__("wall_distance_aggregator")

        self.declare_parameter("left_range_topic", "/wall/left_range")
        self.declare_parameter("right_range_topic", "/wall/right_range")
        self.declare_parameter("distance_angle_topic", "/wall/distance_angle")
        self.declare_parameter("measurement_json_topic", "/wall/measurement_json")
        self.declare_parameter("measurement_frame_id", "front_wall_sensors")
        self.declare_parameter("sensor_separation_m", 0.29)
        self.declare_parameter("safe_distance_m", 0.15)
        self.declare_parameter("input_timeout_s", 0.20)
        self.declare_parameter("max_pair_skew_s", 0.10)
        self.declare_parameter("update_rate_hz", 20.0)

        self.sensor_separation_m = self.get_float("sensor_separation_m")
        self.safe_distance_m = self.get_float("safe_distance_m")
        self.input_timeout_s = self.get_float("input_timeout_s")
        self.max_pair_skew_s = self.get_float("max_pair_skew_s")
        update_rate_hz = self.get_float("update_rate_hz")
        self.measurement_frame_id = str(
            self.get_parameter("measurement_frame_id").value
        )
        self.validate_parameters(update_rate_hz)

        self.latest_left = None
        self.latest_left_time = None
        self.latest_right = None
        self.latest_right_time = None

        self.distance_angle_pub = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("distance_angle_topic").value),
            10,
        )
        self.measurement_json_pub = self.create_publisher(
            String,
            str(self.get_parameter("measurement_json_topic").value),
            10,
        )
        self.left_sub = self.create_subscription(
            Range,
            str(self.get_parameter("left_range_topic").value),
            self.left_callback,
            10,
        )
        self.right_sub = self.create_subscription(
            Range,
            str(self.get_parameter("right_range_topic").value),
            self.right_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / update_rate_hz, self.timer_callback)
        self.get_logger().info(
            "Wall distance aggregator ready: "
            f"separation={self.sensor_separation_m:.3f} m, "
            f"timeout={self.input_timeout_s:.3f} s"
        )

    def validate_parameters(self, update_rate_hz):
        if self.sensor_separation_m <= 0.0:
            raise ValueError("sensor_separation_m must be positive")
        if self.safe_distance_m <= 0.0:
            raise ValueError("safe_distance_m must be positive")
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be positive")
        if self.max_pair_skew_s < 0.0:
            raise ValueError("max_pair_skew_s cannot be negative")
        if update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")

    def left_callback(self, msg):
        if not self.valid_range(msg):
            return
        self.latest_left = float(msg.range)
        self.latest_left_time = self.get_clock().now()

    def right_callback(self, msg):
        if not self.valid_range(msg):
            return
        self.latest_right = float(msg.range)
        self.latest_right_time = self.get_clock().now()

    @staticmethod
    def valid_range(msg):
        value = float(msg.range)
        if not math.isfinite(value):
            return False
        if float(msg.min_range) > 0.0 and value < float(msg.min_range):
            return False
        if float(msg.max_range) > 0.0 and value > float(msg.max_range):
            return False
        return value > 0.0

    def timer_callback(self):
        if self.latest_left_time is None or self.latest_right_time is None:
            return

        now = self.get_clock().now()
        left_age_s = self.seconds_between(now, self.latest_left_time)
        right_age_s = self.seconds_between(now, self.latest_right_time)
        pair_skew_s = abs(
            self.seconds_between(self.latest_left_time, self.latest_right_time)
        )
        if (
            left_age_s > self.input_timeout_s
            or right_age_s > self.input_timeout_s
            or pair_skew_s > self.max_pair_skew_s
        ):
            return

        try:
            measurement = calculate_wall_measurement(
                self.latest_left,
                self.latest_right,
                self.sensor_separation_m,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Cannot combine wall ranges: {exc}")
            return

        stamp = now.to_msg()
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.measurement_frame_id
        msg.vector.x = measurement.distance_m
        msg.vector.y = measurement.angle_rad
        msg.vector.z = measurement.min_distance_m
        self.distance_angle_pub.publish(msg)

        payload = {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "frame_id": self.measurement_frame_id,
            "left_distance_m": self.latest_left,
            "right_distance_m": self.latest_right,
            "wall_distance_m": measurement.distance_m,
            "wall_angle_rad": measurement.angle_rad,
            "wall_angle_deg": math.degrees(measurement.angle_rad),
            "min_distance_m": measurement.min_distance_m,
            "safe_distance_m": self.safe_distance_m,
            "too_close": measurement.distance_m <= self.safe_distance_m,
            "left_age_s": left_age_s,
            "right_age_s": right_age_s,
            "pair_skew_s": pair_skew_s,
        }
        json_msg = String()
        json_msg.data = json.dumps(payload, separators=(",", ":"))
        self.measurement_json_pub.publish(json_msg)

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def seconds_between(newer, older):
        return (newer - older).nanoseconds / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = WallDistanceAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
