import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, SetBusServoState
from std_msgs.msg import String

from .control_logic import (
    MotionDecision,
    MotionSettings,
    ZoneGeometry,
    decide_motion,
    parse_class_list,
    pickup_is_ready,
    select_largest_candidate,
)


class BboxZoneController(Node):
    GRAB_TRACKING = "TRACKING"
    GRAB_OPENING = "OPENING"
    GRAB_FINAL_FORWARD = "FINAL_FORWARD"
    GRAB_CLOSING = "CLOSING"

    def __init__(self) -> None:
        super().__init__("bbox_zone_controller")

        defaults = (
            ("detections_topic", "/yolo/detections"),
            ("cmd_vel_topic", "/cmd_vel"),
            ("control_topic", "/bbox_zone_controller/control"),
            ("state_topic", "/bbox_zone_controller/state"),
            ("status_topic", "/bbox_zone_controller/status"),
            ("bus_servo_topic", "/ros_robot_controller/bus_servo/set_state"),
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
            ("gripper_enabled", True),
            ("gripper_servo_id", 1),
            ("gripper_open_position", 1000),
            ("gripper_closed_position", 300),
            ("gripper_move_duration_s", 0.5),
            ("grab_center_tolerance", 0.18),
            ("grab_area_ratio", 0.70),
            ("grab_detection_timeout_s", 0.25),
            ("final_forward_linear_x", 0.20),
            ("final_forward_duration_s", 1.0),
            ("grab_duration_s", 1.0),
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
        self.grab_state = self.GRAB_TRACKING
        self.grab_state_started_at = self.get_clock().now()
        self.pickup_label = None
        self.picked_count = 0

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
        self.bus_servo_pub = self.create_publisher(
            SetBusServoState,
            str(self.get_parameter("bus_servo_topic").value),
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
        if self.active:
            self.command_gripper(open_gripper=False)
        self.publish_state()

    def validate_parameters(self) -> None:
        positive = (
            "timer_rate_hz",
            "status_rate_hz",
            "detection_timeout_s",
            "fallback_image_width",
            "fallback_image_height",
            "gripper_move_duration_s",
            "grab_detection_timeout_s",
            "final_forward_duration_s",
            "grab_duration_s",
        )
        for name in positive:
            if self.get_float(name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        confidence = self.get_float("min_confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("min_confidence must be between 0 and 1")
        for name in ("grab_center_tolerance", "grab_area_ratio"):
            value = self.get_float(name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between 0 and 1")
        if self.get_float("final_forward_linear_x") < 0.0:
            raise ValueError("final_forward_linear_x must be nonnegative")
        if int(self.get_parameter("gripper_servo_id").value) <= 0:
            raise ValueError("gripper_servo_id must be positive")
        for name in ("gripper_open_position", "gripper_closed_position"):
            position = int(self.get_parameter(name).value)
            if not 0 <= position <= 1000:
                raise ValueError(f"{name} must be between 0 and 1000")

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
            self.change_grab_state(self.GRAB_TRACKING)
            self.pickup_label = None
            self.command_gripper(open_gripper=False)
            self.active = True
            self.last_decision = MotionDecision(0.0, 0.0, "waiting_for_detection")
            self.get_logger().info("Bbox zone control started")
        elif command == "stop":
            self.active = False
            self.stop_robot()
            self.last_decision = MotionDecision(0.0, 0.0, "inactive")
            self.get_logger().info("Bbox zone control stopped")
        elif command == "reset":
            self.active = False
            self.stop_robot()
            self.change_grab_state(self.GRAB_TRACKING)
            self.pickup_label = None
            self.picked_count = 0
            self.command_gripper(open_gripper=False)
            self.last_decision = MotionDecision(0.0, 0.0, "inactive")
            self.get_logger().info("Bbox zone control reset")
        elif command == "open":
            self.command_gripper(open_gripper=True)
        elif command == "close":
            self.command_gripper(open_gripper=False)
        else:
            self.get_logger().warning(
                "Unknown control command; use start, stop, reset, open, or close"
            )
            return
        self.publish_state()
        self.publish_status(force=True)

    def timer_callback(self) -> None:
        if not self.active:
            self.last_decision = MotionDecision(0.0, 0.0, "inactive")
            self.stop_robot()
        elif not self.frame_is_recent():
            if self.grab_state == self.GRAB_TRACKING:
                self.last_decision = MotionDecision(
                    0.0,
                    0.0,
                    "detection_stale_stop",
                )
                self.stop_robot()
            else:
                self.run_grab_sequence()
        else:
            if self.grab_state != self.GRAB_TRACKING:
                self.run_grab_sequence()
            elif self.start_grab_if_ready():
                self.apply_decision(MotionDecision(0.0, 0.0, "grab_opening", True))
            else:
                self.apply_decision(
                    decide_motion(
                        self.latest_candidate,
                        self.target_classes,
                        self.geometry,
                        self.motion_settings,
                    )
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

    def start_grab_if_ready(self) -> bool:
        if not bool(self.get_parameter("gripper_enabled").value):
            return False
        if self.latest_frame_received_at is None or (
            self.seconds_since(self.latest_frame_received_at)
            > self.get_float("grab_detection_timeout_s")
        ):
            return False
        if not pickup_is_ready(
            self.latest_candidate,
            self.target_classes,
            self.get_float("grab_center_tolerance"),
            self.get_float("grab_area_ratio"),
        ):
            return False
        self.pickup_label = self.latest_candidate.class_name
        self.stop_robot()
        self.command_gripper(open_gripper=True)
        self.change_grab_state(self.GRAB_OPENING)
        return True

    def run_grab_sequence(self) -> None:
        if self.grab_state == self.GRAB_OPENING:
            if self.grab_state_age_s() < self.get_float("gripper_move_duration_s"):
                self.apply_decision(MotionDecision(0.0, 0.0, "grab_opening", True))
                return
            self.change_grab_state(self.GRAB_FINAL_FORWARD)

        if self.grab_state == self.GRAB_FINAL_FORWARD:
            if self.grab_state_age_s() < self.get_float("final_forward_duration_s"):
                self.apply_decision(
                    MotionDecision(
                        self.get_float("final_forward_linear_x"),
                        0.0,
                        "grab_final_forward",
                        True,
                    )
                )
                return
            self.command_gripper(open_gripper=False)
            self.change_grab_state(self.GRAB_CLOSING)

        if self.grab_state == self.GRAB_CLOSING:
            if self.grab_state_age_s() < self.get_float("grab_duration_s"):
                self.apply_decision(MotionDecision(0.0, 0.0, "grab_closing", True))
                return
            self.picked_count += 1
            label = self.pickup_label or "unknown"
            self.pickup_label = None
            self.change_grab_state(self.GRAB_TRACKING)
            self.get_logger().info(
                f"Pickup complete: label={label}, picked_count={self.picked_count}"
            )
            self.apply_decision(MotionDecision(0.0, 0.0, "grab_complete", True))

    def apply_decision(self, decision: MotionDecision) -> None:
        self.last_decision = decision
        self.publish_cmd_vel(decision.linear_x, decision.angular_z)

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)

    def command_gripper(self, open_gripper: bool) -> None:
        action = "open" if open_gripper else "close"
        if not bool(self.get_parameter("gripper_enabled").value):
            self.get_logger().info(f"Gripper {action} skipped because it is disabled")
            return
        if self.dry_run:
            self.get_logger().info(f"Gripper {action} skipped in dry-run mode")
            return
        position_name = (
            "gripper_open_position" if open_gripper else "gripper_closed_position"
        )
        state = BusServoState()
        state.present_id = [1, int(self.get_parameter("gripper_servo_id").value)]
        state.position = [1, int(self.get_parameter(position_name).value)]

        msg = SetBusServoState()
        msg.duration = self.get_float("gripper_move_duration_s")
        msg.state = [state]
        self.bus_servo_pub.publish(msg)
        self.get_logger().info(
            f"Gripper command: {action} position={state.position[1]}"
        )

    def change_grab_state(self, state: str) -> None:
        self.grab_state = state
        self.grab_state_started_at = self.get_clock().now()

    def grab_state_age_s(self) -> float:
        return self.seconds_since(self.grab_state_started_at)

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
            "grab_state": self.grab_state,
            "pickup_label": self.pickup_label,
            "picked_count": self.picked_count,
            "selected": (
                None
                if candidate is None
                else {
                    "class_id": candidate.class_id,
                    "class_name": candidate.class_name,
                    "confidence": candidate.confidence,
                    "center_x": candidate.center_x,
                    "center_y": candidate.center_y,
                    "bottom_y": candidate.bottom_y,
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
