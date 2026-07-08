import json
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class DetectionsToTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("detections_to_target_node")

        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("target_topic", "/target_object")
        self.declare_parameter("target_label_topic", "/target_label")
        self.declare_parameter("avoid_topic", "/avoid_object")
        self.declare_parameter("avoid_label_topic", "/avoid_label")
        self.declare_parameter("avoid_objects_topic", "/avoid_objects")
        self.declare_parameter("target_classes", "")
        self.declare_parameter("avoid_classes", "")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("target_lock_enabled", True)
        self.declare_parameter("target_lock_timeout_s", 0.7)
        self.declare_parameter("target_lock_iou_threshold", 0.20)
        self.declare_parameter("target_lock_x_margin", 0.30)
        self.declare_parameter("target_lock_y_margin", 0.20)
        self.declare_parameter("target_switch_y_margin", 0.12)
        self.declare_parameter("target_switch_score_margin", 0.25)
        self.declare_parameter("target_center_weight", 0.25)
        self.declare_parameter("avoid_target_iou_threshold", 0.35)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.target_label_topic = self.get_parameter("target_label_topic").value
        self.avoid_topic = self.get_parameter("avoid_topic").value
        self.avoid_label_topic = self.get_parameter("avoid_label_topic").value
        self.avoid_objects_topic = self.get_parameter("avoid_objects_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.target_lock_enabled = bool(self.get_parameter("target_lock_enabled").value)
        self.target_lock_timeout_s = float(self.get_parameter("target_lock_timeout_s").value)
        self.target_lock_iou_threshold = float(self.get_parameter("target_lock_iou_threshold").value)
        self.target_lock_x_margin = float(self.get_parameter("target_lock_x_margin").value)
        self.target_lock_y_margin = float(self.get_parameter("target_lock_y_margin").value)
        self.target_switch_y_margin = float(self.get_parameter("target_switch_y_margin").value)
        self.target_switch_score_margin = float(self.get_parameter("target_switch_score_margin").value)
        self.target_center_weight = float(self.get_parameter("target_center_weight").value)
        self.avoid_target_iou_threshold = float(self.get_parameter("avoid_target_iou_threshold").value)
        self.target_classes = self.parse_class_list(self.get_parameter("target_classes").value)
        self.avoid_classes = self.parse_class_list(self.get_parameter("avoid_classes").value)
        self.locked_target_track_id = None
        self.locked_target_bbox = None
        self.locked_target_point = None
        self.locked_target_time = None

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.target_label_pub = self.create_publisher(String, self.target_label_topic, 10)
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

            class_name, class_keys, point_msg, bbox_xyxy, track_id, center_y_ratio = converted
            if self.is_target(class_keys):
                target_candidates.append((point_msg.point.y, class_name, point_msg, bbox_xyxy, track_id, center_y_ratio))
            elif self.is_avoid(class_keys):
                avoid_candidates.append((point_msg.point.y, class_name, point_msg, bbox_xyxy, track_id, center_y_ratio))

        avoid_candidates = self.filter_overlapping_avoid_candidates(avoid_candidates, target_candidates)
        target_candidate = self.select_target_candidate(target_candidates)
        self.publish_candidate(target_candidate, self.target_pub, self.target_label_pub)
        self.publish_best(avoid_candidates, self.avoid_pub, self.avoid_label_pub)
        self.publish_avoid_objects(avoid_candidates, header)

    def convert_detection(
        self,
        detection: dict[str, Any],
        image_width: float,
        image_height: float,
        header,
    ) -> tuple[str, set[str], PointStamped, tuple[float, float, float, float], int | None, float] | None:
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

        bbox_center_x = (x1 + x2) * 0.5
        bbox_center_y = (y1 + y2) * 0.5
        image_center_x = image_width * 0.5
        normalized_x_error = (bbox_center_x - image_center_x) / image_center_x
        bottom_y_ratio = y2 / image_height
        center_y_ratio = self.clamp(bbox_center_y / image_height, 0.0, 1.0)

        out = PointStamped()
        out.header = header
        out.point.x = self.clamp(normalized_x_error, -1.0, 1.0)
        out.point.y = self.clamp(bottom_y_ratio, 0.0, 1.0)
        out.point.z = confidence
        return class_name, class_keys, out, (x1, y1, x2, y2), self.parse_track_id(detection), center_y_ratio

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

    def select_target_candidate(self, candidates):
        if not candidates:
            self.clear_expired_target_lock()
            return None
        if not self.target_lock_enabled:
            return self.update_target_lock(self.best_target_candidate(candidates))

        best_candidate = self.best_target_candidate(candidates)
        if not self.has_active_target_lock():
            return self.update_target_lock(best_candidate)

        locked_candidates = [
            candidate for candidate in candidates
            if self.candidate_matches_locked_target(candidate)
        ]
        if not locked_candidates:
            return self.update_target_lock(best_candidate)

        locked_candidate = self.best_target_candidate(locked_candidates)
        if self.should_switch_target(best_candidate, locked_candidate):
            return self.update_target_lock(best_candidate)
        return self.update_target_lock(locked_candidate)

    def best_target_candidate(self, candidates):
        return max(candidates, key=self.target_candidate_score)

    def target_candidate_score(self, candidate):
        _, _, point_msg, _, _, _ = candidate
        closeness = float(point_msg.point.y)
        x_error = abs(float(point_msg.point.x))
        return closeness - self.target_center_weight * x_error

    def should_switch_target(self, new_candidate, locked_candidate):
        if new_candidate is locked_candidate:
            return False

        new_y = float(new_candidate[2].point.y)
        locked_y = float(locked_candidate[2].point.y)
        if new_y >= locked_y + self.target_switch_y_margin:
            return True

        new_score = self.target_candidate_score(new_candidate)
        locked_score = self.target_candidate_score(locked_candidate)
        return new_score >= locked_score + self.target_switch_score_margin

    def candidate_matches_locked_target(self, candidate):
        if self.locked_target_bbox is None or self.locked_target_point is None:
            return False

        _, _, point_msg, bbox_xyxy, track_id, _ = candidate
        if track_id is not None and track_id == self.locked_target_track_id:
            return True

        if self.bbox_iou(bbox_xyxy, self.locked_target_bbox) >= self.target_lock_iou_threshold:
            return True

        locked_x, locked_y = self.locked_target_point
        x_gap = abs(float(point_msg.point.x) - locked_x)
        y_gap = abs(float(point_msg.point.y) - locked_y)
        return x_gap <= self.target_lock_x_margin and y_gap <= self.target_lock_y_margin

    def has_active_target_lock(self):
        if self.locked_target_time is None:
            return False

        elapsed = self.get_clock().now() - self.locked_target_time
        if elapsed.nanoseconds / 1_000_000_000.0 > self.target_lock_timeout_s:
            self.clear_target_lock()
            return False
        return True

    def clear_expired_target_lock(self):
        if self.locked_target_time is not None:
            self.has_active_target_lock()

    def update_target_lock(self, candidate):
        if candidate is None:
            return None

        _, _, point_msg, bbox_xyxy, track_id, _ = candidate
        self.locked_target_track_id = track_id
        self.locked_target_bbox = bbox_xyxy
        self.locked_target_point = (float(point_msg.point.x), float(point_msg.point.y))
        self.locked_target_time = self.get_clock().now()
        return candidate

    def clear_target_lock(self):
        self.locked_target_track_id = None
        self.locked_target_bbox = None
        self.locked_target_point = None
        self.locked_target_time = None

    def publish_best(self, candidates, point_pub, label_pub) -> None:
        if not candidates:
            return

        self.publish_candidate(max(candidates, key=lambda item: item[0]), point_pub, label_pub)

    def publish_candidate(self, candidate, point_pub, label_pub) -> None:
        if candidate is None:
            return

        _, class_name, point_msg, _, _, _ = candidate
        point_pub.publish(point_msg)

        label_msg = String()
        label_msg.data = class_name
        label_pub.publish(label_msg)

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
                    "confidence": float(point_msg.point.z),
                    "track_id": track_id,
                    "bbox_xyxy": {
                        "x1": float(bbox_xyxy[0]),
                        "y1": float(bbox_xyxy[1]),
                        "x2": float(bbox_xyxy[2]),
                        "y2": float(bbox_xyxy[3]),
                    },
                }
                for _, class_name, point_msg, bbox_xyxy, track_id, center_y_ratio in sorted(
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
    def parse_track_id(detection: dict[str, Any]) -> int | None:
        value = detection.get("stable_track_id", detection.get("track_id"))
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

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
