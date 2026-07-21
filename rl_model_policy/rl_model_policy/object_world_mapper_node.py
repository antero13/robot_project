import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ros2_yolo_detector.class_id_filter import parse_class_ids

from rl_model_policy.observation import quaternion_to_yaw
from robot_status_gui.object_localization import CalibrationObjectLocalizer


class ObjectWorldMapperNode(Node):
    def __init__(self):
        super().__init__("rl_object_world_mapper")

        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("output_topic", "/rl_estimated_objects")
        self.declare_parameter("policy_state_topic", "/rl_model_policy_state")
        self.declare_parameter("calibration_path", "")
        self.declare_parameter("target_classes", "")
        self.declare_parameter("avoid_classes", "")
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("retention_s", 180.0)
        self.declare_parameter("min_confirmations", 2)
        self.declare_parameter("confirmation_window_s", 1.0)
        self.declare_parameter("association_radius_m", 0.30)
        self.declare_parameter("position_smoothing_alpha", 0.35)
        self.declare_parameter("pickup_remove_radius_m", 0.75)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("arena_half_extent_m", 2.0)
        self.declare_parameter("horizontal_extrapolation_margin", 0.015)
        self.declare_parameter("vertical_extrapolation_margin", 0.012)

        self.target_classes = parse_class_ids(
            self.get_parameter("target_classes").value
        )
        self.avoid_classes = parse_class_ids(
            self.get_parameter("avoid_classes").value
        )
        self.pose_timeout_s = self.get_float("pose_timeout_s")
        self.retention_s = self.get_float("retention_s")
        self.min_confirmations = int(
            self.get_parameter("min_confirmations").value
        )
        self.confirmation_window_s = self.get_float("confirmation_window_s")
        self.association_radius = self.get_float("association_radius_m")
        self.smoothing_alpha = self.get_float("position_smoothing_alpha")
        self.pickup_remove_radius = self.get_float("pickup_remove_radius_m")
        publish_rate_hz = self.get_float("publish_rate_hz")
        self._validate_parameters(publish_rate_hz)

        calibration_value = str(
            self.get_parameter("calibration_path").value
        ).strip()
        if calibration_value:
            self.calibration_path = Path(calibration_value).expanduser()
        else:
            self.calibration_path = Path(
                get_package_share_directory("robot_status_gui")
            ) / "config" / "distance_normalized_points.csv"
        self.localizer = None
        self.calibration_error = None
        try:
            self.localizer = CalibrationObjectLocalizer(
                calibration_path=self.calibration_path,
                arena_half_extent_m=self.get_float("arena_half_extent_m"),
                horizontal_extrapolation_margin=self.get_float(
                    "horizontal_extrapolation_margin"
                ),
                vertical_extrapolation_margin=self.get_float(
                    "vertical_extrapolation_margin"
                ),
            )
        except (OSError, ValueError) as exc:
            self.calibration_error = str(exc)
            self.get_logger().error(
                f"Object map calibration unavailable: {self.calibration_error}"
            )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_pose_time = None
        self.tracked_objects = {}
        self.next_track_id = 1
        self.last_detection_count = 0
        self.last_mapped_count = 0
        self.stored_object_count = 0

        self.output_pub = self.create_publisher(
            String,
            self.get_parameter("output_topic").value,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self.odometry_callback,
            10,
        )
        self.detections_sub = self.create_subscription(
            String,
            self.get_parameter("detections_topic").value,
            self.detections_callback,
            10,
        )
        self.policy_sub = self.create_subscription(
            String,
            self.get_parameter("policy_state_topic").value,
            self.policy_state_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_objects)

        self.get_logger().info(
            "Mapping YOLO bbox centers with measured calibration data; "
            f"calibration={self.calibration_path}, "
            f"output={self.get_parameter('output_topic').value}"
        )

    def _validate_parameters(self, publish_rate_hz):
        positive_values = {
            "pose_timeout_s": self.pose_timeout_s,
            "retention_s": self.retention_s,
            "confirmation_window_s": self.confirmation_window_s,
            "association_radius_m": self.association_radius,
            "pickup_remove_radius_m": self.pickup_remove_radius,
            "publish_rate_hz": publish_rate_hz,
        }
        invalid = [name for name, value in positive_values.items() if value <= 0.0]
        if invalid:
            raise ValueError(f"parameters must be positive: {', '.join(invalid)}")
        if self.min_confirmations <= 0:
            raise ValueError("min_confirmations must be positive")
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("position_smoothing_alpha must be in (0, 1]")

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_x = float(position.x)
        self.robot_y = float(position.y)
        self.robot_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.latest_pose_time = self.get_clock().now()

    def detections_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid detections JSON: {exc}")
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return
        self.last_detection_count = len(detections)
        self.last_mapped_count = 0
        if not self.pose_is_fresh():
            return

        try:
            image_width = float(payload.get("image_width", 0.0))
            image_height = float(payload.get("image_height", 0.0))
        except (TypeError, ValueError):
            return
        if image_width <= 0.0 or image_height <= 0.0:
            return

        now_ns = self.get_clock().now().nanoseconds
        frame_estimates = []
        for detection in detections:
            estimate = self.localize_detection(
                detection,
                image_width,
                image_height,
            )
            if estimate is None:
                continue
            estimate["seen_at_ns"] = now_ns
            frame_estimates.append(estimate)

        self.last_mapped_count = len(frame_estimates)
        self.update_tracks(frame_estimates, now_ns)

    def update_tracks(self, frame_estimates, now_ns):
        matched_tracks = set()
        for estimate in sorted(
            frame_estimates,
            key=lambda item: float(item.get("confidence", 0.0)),
            reverse=True,
        ):
            track_id = self.find_matching_track(estimate, matched_tracks)
            if track_id is None:
                track_id = f"object_{self.next_track_id:04d}"
                self.next_track_id += 1
                estimate["id"] = track_id
                estimate["hit_count"] = 1
                self.tracked_objects[track_id] = estimate
            else:
                previous = self.tracked_objects[track_id]
                estimate = self.merge_track(previous, estimate, now_ns)
                estimate["id"] = track_id
                self.tracked_objects[track_id] = estimate
            matched_tracks.add(track_id)

    def find_matching_track(self, estimate, excluded_tracks):
        candidates = []
        for track_id, tracked in self.tracked_objects.items():
            if track_id in excluded_tracks:
                continue
            if tracked.get("class_name") != estimate.get("class_name"):
                continue
            distance = math.hypot(
                float(tracked["arena_x"]) - float(estimate["arena_x"]),
                float(tracked["arena_y"]) - float(estimate["arena_y"]),
            )
            if distance <= self.association_radius:
                candidates.append((distance, track_id))
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def merge_track(self, previous, current, now_ns):
        alpha = self.smoothing_alpha
        current["arena_x"] = self.smoothed(
            previous["arena_x"], current["arena_x"], alpha
        )
        current["arena_y"] = self.smoothed(
            previous["arena_y"], current["arena_y"], alpha
        )
        current["map_x"] = current["arena_x"] + 2.0
        current["map_y"] = current["arena_y"] + 2.0
        recent = now_ns - int(previous.get("seen_at_ns", 0)) <= int(
            self.confirmation_window_s * 1_000_000_000
        )
        current["hit_count"] = (
            int(previous.get("hit_count", 0)) + 1 if recent else 1
        )
        return current

    def policy_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        mission = payload.get("mission", {})
        if isinstance(mission, dict) and "total_collected_count" in mission:
            try:
                new_count = int(mission["total_collected_count"])
            except (TypeError, ValueError):
                return
        else:
            stored_objects = payload.get("stored_objects", [])
            if not isinstance(stored_objects, list):
                return
            new_count = len(stored_objects)
        if new_count < self.stored_object_count:
            self.stored_object_count = new_count
            self.tracked_objects.clear()
            return
        removed_count = new_count - self.stored_object_count
        self.stored_object_count = new_count
        for _ in range(removed_count):
            self.remove_nearest_collected_target()

    def remove_nearest_collected_target(self):
        target_items = [
            (key, value)
            for key, value in self.tracked_objects.items()
            if value.get("role") == "target"
        ]
        if not target_items:
            return
        key, target = min(
            target_items,
            key=lambda item: math.hypot(
                float(item[1]["arena_x"]) - self.robot_x,
                float(item[1]["arena_y"]) - self.robot_y,
            ),
        )
        distance = math.hypot(
            float(target["arena_x"]) - self.robot_x,
            float(target["arena_y"]) - self.robot_y,
        )
        if distance <= self.pickup_remove_radius:
            del self.tracked_objects[key]

    def localize_detection(self, detection, image_width, image_height):
        if self.localizer is None:
            return None
        bbox = detection.get("bbox_xyxy", {})
        try:
            center_x = (float(bbox["x1"]) + float(bbox["x2"])) * 0.5
            center_y = (float(bbox["y1"]) + float(bbox["y2"])) * 0.5
            confidence = float(detection.get("confidence", 0.0))
        except (KeyError, TypeError, ValueError):
            return None

        image_x = (center_x - image_width * 0.5) / (image_width * 0.5)
        image_y = center_y / image_height
        localized = self.localizer.localize(
            image_x=image_x,
            image_y=image_y,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
        )
        if localized is None:
            return None

        try:
            class_id = int(detection["class_id"])
        except (KeyError, TypeError, ValueError):
            return None
        class_name = str(detection.get("class_name", class_id))
        return {
            "class_id": class_id,
            "class_name": class_name,
            "role": self.class_role(class_id),
            "confidence": confidence,
            "arena_x": localized.x,
            "arena_y": localized.y,
            "map_x": localized.x + 2.0,
            "map_y": localized.y + 2.0,
            "method": localized.method,
            "interpolation_span_m": localized.interpolation_span_m,
            "camera_lateral_m": localized.lateral_m,
            "camera_forward_m": localized.forward_m,
            "image_x": image_x,
            "image_y": image_y,
        }

    def publish_objects(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        max_age_ns = int(self.retention_s * 1_000_000_000)
        self.tracked_objects = {
            key: value
            for key, value in self.tracked_objects.items()
            if now_ns - int(value["seen_at_ns"]) <= max_age_ns
        }

        objects = []
        confirmed_objects = [
            tracked
            for tracked in self.tracked_objects.values()
            if int(tracked.get("hit_count", 0)) >= self.min_confirmations
        ]
        for tracked in sorted(confirmed_objects, key=lambda item: item["id"]):
            output = dict(tracked)
            seen_at_ns = output.pop("seen_at_ns")
            output["age_s"] = max(
                0.0,
                (now_ns - seen_at_ns) / 1_000_000_000.0,
            )
            objects.append(output)

        pose_fresh = self.pose_is_fresh()
        if self.localizer is None:
            mapper_status = "calibration_error"
        elif not pose_fresh:
            mapper_status = "waiting_for_odometry"
        elif self.last_detection_count and not self.last_mapped_count:
            mapper_status = "detections_outside_calibration"
        else:
            mapper_status = "ready"

        message = String()
        message.data = json.dumps(
            {
                "mapper_status": mapper_status,
                "calibration_loaded": self.localizer is not None,
                "calibration_file": self.calibration_path.name,
                "calibration_error": self.calibration_error,
                "localization_method": "calibration_interpolation",
                "pose_fresh": pose_fresh,
                "detection_count": self.last_detection_count,
                "mapped_count": self.last_mapped_count,
                "objects": objects,
            },
            separators=(",", ":"),
        )
        self.output_pub.publish(message)

    def pose_is_fresh(self):
        if self.latest_pose_time is None:
            return False
        age = self.get_clock().now() - self.latest_pose_time
        return age.nanoseconds / 1_000_000_000.0 <= self.pose_timeout_s

    def class_role(self, class_id):
        if self.target_classes and class_id in self.target_classes:
            return "target"
        if self.avoid_classes and class_id in self.avoid_classes:
            return "avoid"
        if self.target_classes:
            return "avoid" if not self.avoid_classes else "other"
        return "target"

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def smoothed(previous, current, alpha):
        return float(previous) + (float(current) - float(previous)) * float(alpha)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectWorldMapperNode()
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
