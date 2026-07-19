import json
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .detection_geometry import bbox_to_normalized_point
from .target_lock import (
    candidate_matches_lock,
    lock_from_candidate,
    select_locked_candidate,
)


class DetectionsToTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("detections_to_target_node")

        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("target_topic", "/target_object")
        self.declare_parameter("target_label_topic", "/target_label")
        self.declare_parameter("target_visibility_topic", "/target_visible")
        self.declare_parameter("target_center_y_topic", "/target_center_y")
        self.declare_parameter("avoid_topic", "/avoid_object")
        self.declare_parameter("avoid_label_topic", "/avoid_label")
        self.declare_parameter("avoid_objects_topic", "/avoid_objects")
        self.declare_parameter("target_classes", "")
        self.declare_parameter("avoid_classes", "")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("target_center_weight", 0.25)
        self.declare_parameter("avoid_target_iou_threshold", 0.35)
        self.declare_parameter("target_lock_enabled", True)
        self.declare_parameter("target_lock_timeout_s", 0.80)
        self.declare_parameter("target_lock_iou_threshold", 0.10)
        self.declare_parameter("target_lock_center_distance", 0.20)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.target_label_topic = self.get_parameter("target_label_topic").value
        self.target_visibility_topic = self.get_parameter("target_visibility_topic").value
        self.target_center_y_topic = self.get_parameter("target_center_y_topic").value
        self.avoid_topic = self.get_parameter("avoid_topic").value
        self.avoid_label_topic = self.get_parameter("avoid_label_topic").value
        self.avoid_objects_topic = self.get_parameter("avoid_objects_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.target_center_weight = float(self.get_parameter("target_center_weight").value)
        self.avoid_target_iou_threshold = float(self.get_parameter("avoid_target_iou_threshold").value)
        self.target_classes = self.parse_class_list(self.get_parameter("target_classes").value)
        self.avoid_classes = self.parse_class_list(self.get_parameter("avoid_classes").value)
        self.target_lock = None
        self.target_lock_last_seen_s = None

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.target_label_pub = self.create_publisher(String, self.target_label_topic, 10)
        self.target_visibility_pub = self.create_publisher(
            Bool,
            self.target_visibility_topic,
            10,
        )
        self.target_center_y_pub = self.create_publisher(
            Float32,
            self.target_center_y_topic,
            10,
        )
        self.avoid_pub = self.create_publisher(PointStamped, self.avoid_topic, 10)
        self.avoid_label_pub = self.create_publisher(String, self.avoid_label_topic, 10)
        self.avoid_objects_pub = self.create_publisher(String, self.avoid_objects_topic, 10)
        self.detections_sub = self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            f"Converting {self.detections_topic} to {self.target_topic}. "
            f"target_classes={sorted(self.target_classes) if self.target_classes else 'all'}"
        )

    def detections_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid detections JSON: {exc}")
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            self.get_logger().warning("Detections JSON has no list field named 'detections'.")
            return

        image_width = self.get_image_size(payload, "image_width")
        image_height = self.get_image_size(payload, "image_height")
        header = self.make_header(payload)

        target_candidates = []
        avoid_candidates = []
        for detection in detections:
            converted = self.convert_detection(detection, image_width, image_height, header)
            if converted is None:
                continue

            class_name, class_keys, point_msg, bbox_xyxy, center_y_ratio = converted
            if self.is_target(class_keys):
                target_candidates.append(
                    (point_msg.point.y, class_name, point_msg, bbox_xyxy, center_y_ratio)
                )
            elif self.is_avoid(class_keys):
                avoid_candidates.append(
                    (point_msg.point.y, class_name, point_msg, bbox_xyxy, center_y_ratio)
                )

        target_candidate = self.select_target_candidate(target_candidates)
        avoid_candidates = self.filter_overlapping_avoid_candidates(
            avoid_candidates,
            target_candidates,
        )
        avoid_candidates = self.filter_locked_target_from_avoid(avoid_candidates)
        visibility_msg = Bool()
        visibility_msg.data = target_candidate is not None
        self.target_visibility_pub.publish(visibility_msg)
        self.publish_candidate(target_candidate, self.target_pub, self.target_label_pub)
        self.publish_target_center_y(target_candidate)
        self.publish_best(avoid_candidates, self.avoid_pub, self.avoid_label_pub)
        self.publish_avoid_objects(avoid_candidates, header)

    def convert_detection(
        self,
        detection: dict[str, Any],
        image_width: float,
        image_height: float,
        header,
    ) -> tuple[str, set[str], PointStamped, tuple[float, float, float, float], float] | None:
        class_name = str(detection.get("class_name", detection.get("class_id", "")))
        class_id = detection.get("class_id", "")
        class_keys = {class_name, str(class_id)}
        confidence = float(detection.get("confidence", 0.0))
        if confidence < self.min_confidence:
            return None

        bbox = detection.get("bbox_xyxy", {})
        try:
            x1 = float(bbox["x1"])
            y1 = float(bbox["y1"])
            x2 = float(bbox["x2"])
            y2 = float(bbox["y2"])
        except (KeyError, TypeError, ValueError):
            return None

        bbox_width = max(0.0, x2 - x1)
        bbox_height = max(0.0, y2 - y1)
        if bbox_width <= 0.0 or bbox_height <= 0.0 or image_width <= 0.0 or image_height <= 0.0:
            return None

        normalized = bbox_to_normalized_point(
            x1,
            y1,
            x2,
            y2,
            image_width,
            image_height,
        )

        out = PointStamped()
        out.header = header
        out.point.x = normalized.x
        # RL uses bbox-bottom closeness. The GUI mapper independently reads
        # the raw bbox center from /yolo/detections for CSV calibration.
        out.point.y = normalized.policy_y
        out.point.z = confidence
        return class_name, class_keys, out, (x1, y1, x2, y2), normalized.y

    def filter_overlapping_avoid_candidates(self, avoid_candidates, target_candidates):
        threshold = self.avoid_target_iou_threshold
        if threshold <= 0.0 or not avoid_candidates or not target_candidates:
            return avoid_candidates

        filtered = []
        for avoid_candidate in avoid_candidates:
            avoid_bbox = avoid_candidate[3]
            overlaps_target = any(
                self.bbox_iou(avoid_bbox, target_candidate[3]) >= threshold
                for target_candidate in target_candidates
            )
            if not overlaps_target:
                filtered.append(avoid_candidate)
        return filtered

    def filter_locked_target_from_avoid(self, avoid_candidates):
        if self.target_lock is None:
            return avoid_candidates
        return [
            candidate
            for candidate in avoid_candidates
            if not candidate_matches_lock(
                candidate,
                self.target_lock,
                iou_threshold=float(
                    self.get_parameter("target_lock_iou_threshold").value
                ),
                center_distance_threshold=float(
                    self.get_parameter("target_lock_center_distance").value
                ),
            )
        ]

    def select_target_candidate(self, candidates):
        if not bool(self.get_parameter("target_lock_enabled").value):
            return None if not candidates else self.best_target_candidate(candidates)

        now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if (
            self.target_lock_last_seen_s is not None
            and now_s - self.target_lock_last_seen_s
            > float(self.get_parameter("target_lock_timeout_s").value)
        ):
            self.target_lock = None
            self.target_lock_last_seen_s = None

        selected = select_locked_candidate(
            candidates,
            self.target_lock,
            score=self.target_candidate_score,
            iou_threshold=float(
                self.get_parameter("target_lock_iou_threshold").value
            ),
            center_distance_threshold=float(
                self.get_parameter("target_lock_center_distance").value
            ),
        )
        if selected is not None:
            self.target_lock = lock_from_candidate(selected)
            self.target_lock_last_seen_s = now_s
        return selected

    def best_target_candidate(self, candidates):
        return max(candidates, key=self.target_candidate_score)

    def target_candidate_score(self, candidate):
        _, _, point_msg, _, _ = candidate
        closeness = float(point_msg.point.y)
        x_error = abs(float(point_msg.point.x))
        return closeness - self.target_center_weight * x_error

    def publish_best(self, candidates, point_pub, label_pub) -> None:
        if not candidates:
            return

        self.publish_candidate(max(candidates, key=lambda item: item[0]), point_pub, label_pub)

    def publish_candidate(self, candidate, point_pub, label_pub) -> None:
        if candidate is None:
            return

        _, class_name, point_msg, _, _ = candidate
        point_pub.publish(point_msg)

        label_msg = String()
        label_msg.data = class_name
        label_pub.publish(label_msg)

    def publish_target_center_y(self, candidate) -> None:
        if candidate is None:
            return
        center_y_msg = Float32()
        center_y_msg.data = float(candidate[4])
        self.target_center_y_pub.publish(center_y_msg)

    def publish_avoid_objects(self, candidates, header) -> None:
        payload = {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
            "objects": [
                {
                    "class_name": class_name,
                    "x": float(point_msg.point.x),
                    "y": float(point_msg.point.y),
                    "center_y": float(center_y_ratio),
                    "bottom_y": float(point_msg.point.y),
                    "confidence": float(point_msg.point.z),
                    "bbox_xyxy": {
                        "x1": float(bbox_xyxy[0]),
                        "y1": float(bbox_xyxy[1]),
                        "x2": float(bbox_xyxy[2]),
                        "y2": float(bbox_xyxy[3]),
                    },
                }
                for _, class_name, point_msg, bbox_xyxy, center_y_ratio in sorted(
                    candidates,
                    key=lambda item: item[0],
                    reverse=True,
                )
            ],
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.avoid_objects_pub.publish(msg)

    def is_target(self, class_keys: set[str]) -> bool:
        if not self.target_classes:
            if self.avoid_classes and bool(class_keys & self.avoid_classes):
                return False
            return True
        return bool(class_keys & self.target_classes)

    def is_avoid(self, class_keys: set[str]) -> bool:
        if self.avoid_classes:
            return bool(class_keys & self.avoid_classes)
        return bool(self.target_classes) and not bool(class_keys & self.target_classes)

    def get_image_size(self, payload: dict[str, Any], key: str) -> float:
        value = payload.get(key)
        if value is not None:
            return float(value)
        return float(self.get_parameter(key).value)

    def make_header(self, payload: dict[str, Any]):
        header = PointStamped().header
        stamp = payload.get("stamp", {})
        header.stamp.sec = int(stamp.get("sec", 0))
        header.stamp.nanosec = int(stamp.get("nanosec", 0))
        header.frame_id = str(payload.get("frame_id", "camera"))
        return header

    @staticmethod
    def parse_class_list(value) -> set[str]:
        if value is None:
            return set()
        if isinstance(value, (list, tuple)):
            return {str(item).strip() for item in value if str(item).strip()}
        return {item.strip() for item in str(value).split(",") if item.strip()}

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def bbox_iou(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        first_x1, first_y1, first_x2, first_y2 = first
        second_x1, second_y1, second_x2, second_y2 = second

        intersection_x1 = max(first_x1, second_x1)
        intersection_y1 = max(first_y1, second_y1)
        intersection_x2 = min(first_x2, second_x2)
        intersection_y2 = min(first_y2, second_y2)
        intersection_width = max(0.0, intersection_x2 - intersection_x1)
        intersection_height = max(0.0, intersection_y2 - intersection_y1)
        intersection_area = intersection_width * intersection_height

        first_area = max(0.0, first_x2 - first_x1) * max(0.0, first_y2 - first_y1)
        second_area = max(0.0, second_x2 - second_x1) * max(0.0, second_y2 - second_y1)
        union_area = first_area + second_area - intersection_area
        if union_area <= 0.0:
            return 0.0
        return intersection_area / union_area


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionsToTargetNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
