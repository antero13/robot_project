import json
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from wall_distance_sensor.alignment_logic import alignment_angular_command


class WallAlignmentTestNode(Node):
    ACTIVE_STATES = {"WAITING_FOR_SENSOR", "ALIGNING"}

    def __init__(self):
        super().__init__("wall_alignment_test")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("wall_topic", "/wall/distance_angle"),
                ("cmd_vel_topic", "/cmd_vel"),
                ("control_topic", "/wall_align/control"),
                ("state_topic", "/wall_align/state"),
                ("status_topic", "/wall_align/status"),
                ("auto_start", False),
                ("timer_rate_hz", 20.0),
                ("status_rate_hz", 5.0),
                ("measurement_timeout_s", 0.5),
                ("alignment_timeout_s", 10.0),
                ("angle_tolerance_deg", 2.0),
                ("stable_ticks", 5),
                ("alignment_gain", 1.2),
                ("minimum_angular_z", 0.05),
                ("maximum_angular_z", 0.20),
                ("minimum_wall_distance_m", 0.35),
                ("maximum_wall_distance_m", 2.0),
            ],
        )
        self.validate_parameters()

        self.state = "IDLE"
        self.reason = "send 'align' to start"
        self.wall_distance = None
        self.wall_angle = None
        self.wall_min_distance = None
        self.wall_received_at = None
        self.alignment_started_at = None
        self.stable_ticks = 0
        self.last_angular_command = 0.0
        self.last_status_at = None

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.wall_sub = self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("wall_topic").value),
            self.wall_callback,
            qos_profile_sensor_data,
        )
        self.control_sub = self.create_subscription(
            String,
            str(self.get_parameter("control_topic").value),
            self.control_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.get_float("timer_rate_hz"),
            self.timer_callback,
        )
        self.publish_state()
        self.publish_status(force=True)
        if bool(self.get_parameter("auto_start").value):
            self.start_alignment("automatic alignment after launch")
        else:
            self.get_logger().info(
                "Wall alignment test ready; send 'align' on /wall_align/control."
            )

    def validate_parameters(self):
        if self.get_float("timer_rate_hz") <= 0.0:
            raise ValueError("timer_rate_hz must be positive")
        if self.get_float("status_rate_hz") <= 0.0:
            raise ValueError("status_rate_hz must be positive")
        if self.get_float("measurement_timeout_s") <= 0.0:
            raise ValueError("measurement_timeout_s must be positive")
        if self.get_float("alignment_timeout_s") <= 0.0:
            raise ValueError("alignment_timeout_s must be positive")
        if int(self.get_parameter("stable_ticks").value) <= 0:
            raise ValueError("stable_ticks must be positive")
        minimum_distance = self.get_float("minimum_wall_distance_m")
        maximum_distance = self.get_float("maximum_wall_distance_m")
        if minimum_distance <= 0.0 or minimum_distance >= maximum_distance:
            raise ValueError("wall-distance limits are invalid")
        alignment_angular_command(
            wall_angle_rad=math.radians(10.0),
            tolerance_rad=math.radians(self.get_float("angle_tolerance_deg")),
            gain=self.get_float("alignment_gain"),
            minimum_angular_z=self.get_float("minimum_angular_z"),
            maximum_angular_z=self.get_float("maximum_angular_z"),
        )

    def wall_callback(self, msg):
        self.wall_distance = float(msg.vector.x)
        self.wall_angle = float(msg.vector.y)
        self.wall_min_distance = float(msg.vector.z)
        self.wall_received_at = self.get_clock().now()

    def control_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "align":
            self.start_alignment("alignment requested by user")
        elif command == "stop":
            self.stop_robot()
            self.alignment_started_at = None
            self.transition("STOPPED", "stopped by user")
        elif command == "reset":
            self.stop_robot()
            self.alignment_started_at = None
            self.stable_ticks = 0
            self.transition("IDLE", "send 'align' to start")
        else:
            self.get_logger().warning(
                f"Unknown command {command!r}; use align, stop, or reset."
            )

    def start_alignment(self, reason):
        self.stop_robot()
        self.alignment_started_at = self.get_clock().now()
        self.stable_ticks = 0
        self.transition("WAITING_FOR_SENSOR", reason)

    def timer_callback(self):
        if self.state not in self.ACTIVE_STATES:
            self.stop_robot()
            self.publish_status()
            return
        if self.alignment_started_at is None:
            self.fail("alignment start time is missing")
            return
        if self.seconds_since(self.alignment_started_at) > self.get_float(
            "alignment_timeout_s"
        ):
            self.fail("alignment timed out; verify left/right sensor placement")
            return
        if not self.wall_measurement_is_recent():
            self.stop_robot()
            if self.state != "WAITING_FOR_SENSOR":
                self.transition("WAITING_FOR_SENSOR", "wall measurement lost")
            self.publish_status()
            return
        if not self.wall_measurement_is_finite():
            self.fail("wall measurement contains a non-finite value")
            return

        minimum_distance = self.get_float("minimum_wall_distance_m")
        maximum_distance = self.get_float("maximum_wall_distance_m")
        closest_distance = min(self.wall_distance, self.wall_min_distance)
        if closest_distance < minimum_distance:
            self.fail(
                f"wall is too close for an in-place turn: {closest_distance:.3f} m"
            )
            return
        if self.wall_distance > maximum_distance:
            self.fail(
                f"wall is too far for this test: {self.wall_distance:.3f} m"
            )
            return
        if self.state == "WAITING_FOR_SENSOR":
            self.transition("ALIGNING", "recent wall measurement received")

        angular = alignment_angular_command(
            wall_angle_rad=self.wall_angle,
            tolerance_rad=math.radians(self.get_float("angle_tolerance_deg")),
            gain=self.get_float("alignment_gain"),
            minimum_angular_z=self.get_float("minimum_angular_z"),
            maximum_angular_z=self.get_float("maximum_angular_z"),
        )
        if angular == 0.0:
            self.stop_robot()
            self.stable_ticks += 1
            if self.stable_ticks >= int(self.get_parameter("stable_ticks").value):
                self.alignment_started_at = None
                self.transition("ALIGNED", "wall angle is stable inside tolerance")
            self.publish_status()
            return

        self.stable_ticks = 0
        self.publish_cmd_vel(angular)
        self.publish_status()

    def wall_measurement_is_recent(self):
        return (
            self.wall_received_at is not None
            and self.seconds_since(self.wall_received_at)
            <= self.get_float("measurement_timeout_s")
        )

    def wall_measurement_is_finite(self):
        return (
            self.wall_distance is not None
            and self.wall_angle is not None
            and self.wall_min_distance is not None
            and math.isfinite(self.wall_distance)
            and math.isfinite(self.wall_angle)
            and math.isfinite(self.wall_min_distance)
        )

    def transition(self, state, reason):
        self.state = state
        self.reason = reason
        self.publish_state()
        self.publish_status(force=True)
        self.get_logger().info(f"Wall alignment state -> {state}: {reason}")

    def fail(self, reason):
        self.stop_robot()
        self.alignment_started_at = None
        self.transition("ERROR", reason)

    def publish_cmd_vel(self, angular_z):
        self.last_angular_command = float(angular_z)
        msg = Twist()
        msg.angular.z = self.last_angular_command
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        if hasattr(self, "cmd_vel_pub"):
            self.publish_cmd_vel(0.0)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        if hasattr(self, "state_pub"):
            self.state_pub.publish(msg)

    def publish_status(self, force=False):
        if not hasattr(self, "status_pub"):
            return
        now = self.get_clock().now()
        if (
            not force
            and self.last_status_at is not None
            and self.seconds_between(now, self.last_status_at)
            < 1.0 / self.get_float("status_rate_hz")
        ):
            return
        payload = {
            "state": self.state,
            "reason": self.reason,
            "wall_distance_m": self.wall_distance,
            "wall_angle_deg": (
                None if self.wall_angle is None else math.degrees(self.wall_angle)
            ),
            "minimum_sensor_distance_m": self.wall_min_distance,
            "measurement_recent": self.wall_measurement_is_recent(),
            "angular_command": self.last_angular_command,
            "stable_ticks": self.stable_ticks,
            "required_stable_ticks": int(self.get_parameter("stable_ticks").value),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        self.last_status_at = now

    def seconds_since(self, older):
        return self.seconds_between(self.get_clock().now(), older)

    @staticmethod
    def seconds_between(newer, older):
        return (newer - older).nanoseconds / 1_000_000_000.0

    def get_float(self, name):
        return float(self.get_parameter(name).value)


def main(args=None):
    rclpy.init(args=args)
    node = WallAlignmentTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
