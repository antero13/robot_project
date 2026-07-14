import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .control_logic import (
    MotionDecision,
    MotionSettings,
    ZoneGeometry,
    decide_motion,
    parse_class_list,
    select_largest_candidate,
)


class BboxZoneController(Node):
    def __init__(self) -> None:
        super().__init__("bbox_zone_controller")

        defaults = (
            ("detections_topic", "/yolo/detections"),
            ("cmd_vel_topic", "/cmd_vel"),
            ("control_topic", "/bbox_zone_controller/control"),
            ("state_topic", "/bbox_zone_controller/state"),
            ("status_topic", "/bbox_zone_controller/status"),
            ("active_on_start", False),
            ("dry_run", False),
            ("timer_rate_hz", 20.0),
            ("status_rate_hz", 5.0),
            ("detection_timeout_s", 0.5),
            ("min_confidence", 0.25),
            ("fallback_image_width", 1280.0),
            ("fallback_image_height", 720.0),
            ("target_classes", "12,20,6,8,apple,banana,orange,pineapple"),
            ("point1_x", -0.8600),
            ("point1_y", 0.9900),
            ("point2_x", -0.7600),
            ("point2_y", 0.8333),
            ("point3_x", 0.7375),
            ("point3_y", 0.9933),
            ("point4_x", 0.5825),
            ("point4_y", 0.7367),
            ("straight_linear_x", 0.10),
            ("avoid_turn_linear_x", 0.06),
            ("avoid_turn_angular_z", 0.45),
            ("target_forward_linear_x", 0.08),
            ("target_center_tolerance", 0.10),
            ("target_angular_gain", 0.80),
            ("target_min_angular_z", 0.10),
            ("target_max_angular_z", 0.45),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)

        self.validate_parameters()
        self.geometry = ZoneGeometry(
            point1=(self.get_float("point1_x"), self.get_float("point1_y")),
            point2=(self.get_float("point2_x"), self.get_float("point2_y")),
            point3=(self.get_float("point3_x"), self.get_float("point3_y")),
            point4=(self.get_float("point4_x"), self.get_float("point4_y")),
        )
        self.motion_settings = MotionSettings(
            straight_linear_x=self.get_float("straight_linear_x"),
            avoid_turn_linear_x=self.get_float("avoid_turn_linear_x"),
            avoid_turn_angular_z=self.get_float("avoid_turn_angular_z"),
            target_forward_linear_x=self.get_float("target_forward_linear_x"),
            target_center_tolerance=self.get_float("target_center_tolerance"),
            target_angular_gain=self.get_float("target_angular_gain"),
            target_min_angular_z=self.get_float("target_min_angular_z"),
            target_max_angular_z=self.get_float("target_max_angular_z"),
        )
        self.target_classes = parse_class_list(
            self.get_parameter("target_classes").value
        )
        if not self.target_classes:
            raise ValueError("target_classes cannot be empty")

        self.active = bool(self.get_parameter("active_on_start").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.latest_candidate = None
        self.latest_frame_received_at = None
        self.last_decision = MotionDecision(0.0, 0.0, "inactive")
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
        self.detections_sub = self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
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

        state = "active" if self.active else "inactive"
        self.get_logger().info(
            f"Largest-bbox zone controller ready ({state}); "
            f"target_classes={sorted(self.target_classes)}"
        )
        self.publish_state()

    def validate_parameters(self) -> None:
        positive = (
            "timer_rate_hz",
            "status_rate_hz",
            "detection_timeout_s",
            "fallback_image_width",
            "fallback_image_height",
        )
        for name in positive:
            if self.get_float(name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        confidence = self.get_float("min_confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("min_confidence must be between 0 and 1")

    def detections_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid detections JSON: {exc}")
            return
        detections = payload.get("detections")
        if not isinstance(detections, list):
            self.get_logger().warning("Detections JSON has no list field named 'detections'.")
            return
        try:
            image_width = float(
                payload.get("image_width")
                or self.get_float("fallback_image_width")
            )
            image_height = float(
                payload.get("image_height")
                or self.get_float("fallback_image_height")
            )
            candidate = select_largest_candidate(
                detections,
                image_width,
                image_height,
                self.get_float("min_confidence"),
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid detection payload: {exc}")
            return
        self.latest_candidate = candidate
        self.latest_frame_received_at = self.get_clock().now()

    def control_callback(self, msg: String) -> None:
        command = msg.data.strip().casefold()
        if command == "start":
            self.active = True
            self.get_logger().info("Bbox zone control started")
        elif command in ("stop", "reset"):
            self.active = False
            self.stop_robot()
            self.get_logger().info("Bbox zone control stopped")
        else:
            self.get_logger().warning("Unknown control command; use start, stop, or reset")
            return
        self.publish_state()
        self.publish_status(force=True)

    def timer_callback(self) -> None:
        if not self.active:
            self.last_decision = MotionDecision(0.0, 0.0, "inactive")
            self.stop_robot()
        elif not self.frame_is_recent():
            self.last_decision = MotionDecision(0.0, 0.0, "detection_stale_stop")
            self.stop_robot()
        else:
            self.last_decision = decide_motion(
                self.latest_candidate,
                self.target_classes,
                self.geometry,
                self.motion_settings,
            )
            self.publish_cmd_vel(
                self.last_decision.linear_x,
                self.last_decision.angular_z,
            )
        self.publish_state()
        self.publish_status()

    def frame_is_recent(self) -> bool:
        if self.latest_frame_received_at is None:
            return False
        return (
            self.seconds_since(self.latest_frame_received_at)
            <= self.get_float("detection_timeout_s")
        )

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.last_decision.mode
        self.state_pub.publish(msg)

    def publish_status(self, force: bool = False) -> None:
        now = self.get_clock().now()
        if (
            not force
            and self.last_status_at is not None
            and self.seconds_between(now, self.last_status_at)
            < 1.0 / self.get_float("status_rate_hz")
        ):
            return
        candidate = self.latest_candidate
        left_boundary = None
        right_boundary = None
        if candidate is not None:
            left_boundary, right_boundary = self.geometry.boundaries_at(
                candidate.center_y
            )
        payload = {
            "active": self.active,
            "dry_run": self.dry_run,
            "frame_recent": self.frame_is_recent(),
            "mode": self.last_decision.mode,
            "linear_x": self.last_decision.linear_x,
            "angular_z": self.last_decision.angular_z,
            "is_target": self.last_decision.is_target,
            "zone": (
                None
                if self.last_decision.zone is None
                else self.last_decision.zone.value
            ),
            "selected": (
                None
                if candidate is None
                else {
                    "class_id": candidate.class_id,
                    "class_name": candidate.class_name,
                    "confidence": candidate.confidence,
                    "center_x": candidate.center_x,
                    "center_y": candidate.center_y,
                    "area_ratio": candidate.area_ratio,
                    "left_boundary_x": left_boundary,
                    "right_boundary_x": right_boundary,
                }
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)
        self.last_status_at = now

    def get_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def seconds_since(self, older) -> float:
        return self.seconds_between(self.get_clock().now(), older)

    @staticmethod
    def seconds_between(newer, older) -> float:
        return (newer - older).nanoseconds / 1_000_000_000.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BboxZoneController()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
