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
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.target_label_topic = self.get_parameter("target_label_topic").value
        self.avoid_topic = self.get_parameter("avoid_topic").value
        self.avoid_label_topic = self.get_parameter("avoid_label_topic").value
        self.avoid_objects_topic = self.get_parameter("avoid_objects_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.target_classes = self.parse_class_list(self.get_parameter("target_classes").value)
        self.avoid_classes = self.parse_class_list(self.get_parameter("avoid_classes").value)

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

            class_name, class_keys, point_msg = converted
            if self.is_target(class_keys):
                target_candidates.append((point_msg.point.y, class_name, point_msg))
            elif self.is_avoid(class_keys):
                avoid_candidates.append((point_msg.point.y, class_name, point_msg))

        self.publish_best(target_candidates, self.target_pub, self.target_label_pub)
        self.publish_best(avoid_candidates, self.avoid_pub, self.avoid_label_pub)
        self.publish_avoid_objects(avoid_candidates, header)

    def convert_detection(
        self,
        detection: dict[str, Any],
        image_width: float,
        image_height: float,
        header,
    ) -> tuple[str, set[str], PointStamped] | None:
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
        image_center_x = image_width * 0.5
        normalized_x_error = (bbox_center_x - image_center_x) / image_center_x
        bottom_y_ratio = y2 / image_height

        out = PointStamped()
        out.header = header
        out.point.x = self.clamp(normalized_x_error, -1.0, 1.0)
        out.point.y = self.clamp(bottom_y_ratio, 0.0, 1.0)
        out.point.z = confidence
        return class_name, class_keys, out

    def publish_best(self, candidates, point_pub, label_pub) -> None:
        if not candidates:
            return

        _, class_name, point_msg = max(candidates, key=lambda item: item[0])
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
                    "confidence": float(point_msg.point.z),
                }
                for _, class_name, point_msg in sorted(candidates, key=lambda item: item[0], reverse=True)
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
