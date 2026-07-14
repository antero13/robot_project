import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from rl_model_policy.object_localization import GridObjectLocalizer
from rl_model_policy.observation import quaternion_to_yaw


class ObjectWorldMapperNode(Node):
    def __init__(self):
        super().__init__("rl_object_world_mapper")

        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("output_topic", "/rl_estimated_objects")
        self.declare_parameter("policy_state_topic", "/rl_model_policy_state")
        self.declare_parameter("target_classes", "")
        self.declare_parameter("avoid_classes", "")
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("retention_s", 180.0)
        self.declare_parameter("min_confirmations", 2)
        self.declare_parameter("confirmation_window_s", 1.0)
        self.declare_parameter("pickup_remove_radius_m", 0.75)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("horizontal_fov_deg", 80.0)
        self.declare_parameter("vertical_fov_deg", 50.0)
        self.declare_parameter("camera_height_m", 0.18)
        self.declare_parameter("camera_pitch_deg", 15.0)
        self.declare_parameter("object_center_height_m", 0.04)
        self.declare_parameter("max_range_m", 4.5)
        self.declare_parameter("max_x_error", 0.35)
        self.declare_parameter("max_y_error", 0.22)
        self.declare_parameter("arena_half_extent_m", 2.0)

        self.target_classes = self.parse_class_list(
            self.get_parameter("target_classes").value
        )
        self.avoid_classes = self.parse_class_list(
            self.get_parameter("avoid_classes").value
        )
        self.pose_timeout_s = self.get_float("pose_timeout_s")
        self.retention_s = self.get_float("retention_s")
        self.min_confirmations = int(
            self.get_parameter("min_confirmations").value
        )
        self.confirmation_window_s = self.get_float("confirmation_window_s")
        self.pickup_remove_radius = self.get_float("pickup_remove_radius_m")
        publish_rate_hz = self.get_float("publish_rate_hz")
        if self.pose_timeout_s <= 0.0 or self.retention_s <= 0.0:
            raise ValueError("pose_timeout_s and retention_s must be positive")
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if (
            self.min_confirmations <= 0
            or self.confirmation_window_s <= 0.0
            or self.pickup_remove_radius <= 0.0
        ):
            raise ValueError("object confirmation parameters must be positive")

        self.localizer = GridObjectLocalizer(
            horizontal_fov_deg=self.get_float("horizontal_fov_deg"),
            vertical_fov_deg=self.get_float("vertical_fov_deg"),
            camera_height_m=self.get_float("camera_height_m"),
            camera_pitch_deg=self.get_float("camera_pitch_deg"),
            object_center_height_m=self.get_float("object_center_height_m"),
            max_range_m=self.get_float("max_range_m"),
            max_x_error=self.get_float("max_x_error"),
            max_y_error=self.get_float("max_y_error"),
            arena_half_extent_m=self.get_float("arena_half_extent_m"),
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_pose_time = None
        self.tracked_objects = {}
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
            "Mapping YOLO bbox centers onto the 42 arena candidate points; "
            f"publishing {self.get_parameter('output_topic').value}"
        )

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

        image_width = float(payload.get("image_width", 0.0))
        image_height = float(payload.get("image_height", 0.0))
        if image_width <= 0.0 or image_height <= 0.0:
            return

        now_ns = self.get_clock().now().nanoseconds
        frame_estimates = {}
        for detection in detections:
            estimate = self.localize_detection(
                detection,
                image_width,
                image_height,
            )
            if estimate is None:
                continue
            estimate["seen_at_ns"] = now_ns
            key = estimate["id"]
            previous = frame_estimates.get(key)
            if previous is None or estimate["confidence"] > previous["confidence"]:
                frame_estimates[key] = estimate

        self.last_mapped_count = len(frame_estimates)
        for key, estimate in frame_estimates.items():
            previous = self.tracked_objects.get(key)
            same_class = (
                previous is not None
                and previous.get("class_name") == estimate.get("class_name")
                and now_ns - int(previous.get("seen_at_ns", 0))
                <= int(self.confirmation_window_s * 1_000_000_000)
            )
            estimate["hit_count"] = (
                int(previous.get("hit_count", 0)) + 1 if same_class else 1
            )
            self.tracked_objects[key] = estimate

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

        class_id = detection.get("class_id", "")
        class_name = str(detection.get("class_name", class_id))
        class_keys = {class_name, str(class_id)}
        candidate = localized.candidate
        if candidate is None:
            object_id = f"projected_{localized.x:.2f}_{localized.y:.2f}"
            column = None
            row = None
        else:
            object_id = f"grid_{candidate.column}_{candidate.row}"
            column = candidate.column
            row = candidate.row

        return {
            "id": object_id,
            "class_id": class_id,
            "class_name": class_name,
            "role": self.class_role(class_keys),
            "confidence": confidence,
            "arena_x": localized.x,
            "arena_y": localized.y,
            "map_x": localized.x + 2.0,
            "map_y": localized.y + 2.0,
            "column": column,
            "row": row,
            "method": localized.method,
            "localization_error": localized.error,
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
            if now_ns - value["seen_at_ns"] <= max_age_ns
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
            output["age_s"] = max(0.0, (now_ns - seen_at_ns) / 1_000_000_000.0)
            objects.append(output)

        message = String()
        message.data = json.dumps(
            {
                "pose_fresh": self.pose_is_fresh(),
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

    def class_role(self, class_keys):
        if self.target_classes and class_keys & self.target_classes:
            return "target"
        if self.avoid_classes and class_keys & self.avoid_classes:
            return "avoid"
        if self.target_classes:
            return "avoid" if not self.avoid_classes else "other"
        return "target"

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def parse_class_list(value):
        if value is None:
            return set()
        if isinstance(value, (list, tuple)):
            return {str(item).strip() for item in value if str(item).strip()}
        return {item.strip() for item in str(value).split(",") if item.strip()}


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
