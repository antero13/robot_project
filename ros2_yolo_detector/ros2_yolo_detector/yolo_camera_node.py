import json
import time
from pathlib import Path
from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from .frame_correction import FrameCorrector


class YoloCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_camera_node")

        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("input_mode", "topic")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("raw_topic", "/camera/image_raw")
        self.declare_parameter("annotated_topic", "/yolo/annotated_image")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("device", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("correction_enabled", False)
        self.declare_parameter("correction_gamma", 0.65)
        self.declare_parameter("correction_clahe_clip_limit", 1.2)
        self.declare_parameter("correction_clahe_tile_grid", 8)
        self.declare_parameter("correction_chroma_gain", 1.3)
        self.declare_parameter("tracker_enabled", True)
        self.declare_parameter("tracker_config", "bytetrack.yaml")
        self.declare_parameter("tracker_persist", True)
        self.declare_parameter("stable_tracking_enabled", True)
        self.declare_parameter("stable_track_timeout_s", 1.0)
        self.declare_parameter("stable_track_iou_threshold", 0.15)
        self.declare_parameter("stable_track_center_ratio", 0.75)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("queue_size", 1)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 0)
        self.declare_parameter("camera_height", 0)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("camera_frame_id", "camera")
        self.declare_parameter("camera_auto_exposure", False)
        self.declare_parameter("camera_exposure", 200.0)
        self.declare_parameter("camera_manual_auto_exposure_value", 1.0)
        self.declare_parameter("camera_auto_exposure_value", 3.0)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.input_mode = self.get_parameter("input_mode").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.raw_topic = self.get_parameter("raw_topic").get_parameter_value().string_value
        self.annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.confidence = self.get_parameter("confidence").get_parameter_value().double_value
        self.iou = self.get_parameter("iou").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.imgsz = self.get_parameter("imgsz").get_parameter_value().integer_value
        self.correction_enabled = self.get_parameter(
            "correction_enabled"
        ).get_parameter_value().bool_value
        self.correction_gamma = self.get_parameter(
            "correction_gamma"
        ).get_parameter_value().double_value
        self.correction_clahe_clip_limit = self.get_parameter(
            "correction_clahe_clip_limit"
        ).get_parameter_value().double_value
        self.correction_clahe_tile_grid = self.get_parameter(
            "correction_clahe_tile_grid"
        ).get_parameter_value().integer_value
        self.correction_chroma_gain = self.get_parameter(
            "correction_chroma_gain"
        ).get_parameter_value().double_value
        self.tracker_enabled = self.get_parameter("tracker_enabled").get_parameter_value().bool_value
        self.tracker_config = self.get_parameter("tracker_config").get_parameter_value().string_value
        self.tracker_persist = self.get_parameter("tracker_persist").get_parameter_value().bool_value
        self.stable_tracking_enabled = self.get_parameter(
            "stable_tracking_enabled"
        ).get_parameter_value().bool_value
        self.stable_track_timeout_s = self.get_parameter(
            "stable_track_timeout_s"
        ).get_parameter_value().double_value
        self.stable_track_iou_threshold = self.get_parameter(
            "stable_track_iou_threshold"
        ).get_parameter_value().double_value
        self.stable_track_center_ratio = self.get_parameter(
            "stable_track_center_ratio"
        ).get_parameter_value().double_value
        self.publish_annotated = self.get_parameter("publish_annotated").get_parameter_value().bool_value
        self.publish_raw = self.get_parameter("publish_raw").get_parameter_value().bool_value
        queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        self.camera_index = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.camera_width = self.get_parameter("camera_width").get_parameter_value().integer_value
        self.camera_height = self.get_parameter("camera_height").get_parameter_value().integer_value
        self.camera_fps = self.get_parameter("camera_fps").get_parameter_value().double_value
        self.camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value
        self.camera_auto_exposure = self.get_parameter("camera_auto_exposure").get_parameter_value().bool_value
        self.camera_exposure = self.get_parameter("camera_exposure").get_parameter_value().double_value
        self.camera_manual_auto_exposure_value = (
            self.get_parameter("camera_manual_auto_exposure_value").get_parameter_value().double_value
        )
        self.camera_auto_exposure_value = (
            self.get_parameter("camera_auto_exposure_value").get_parameter_value().double_value
        )

        if self.imgsz <= 0:
            raise ValueError("imgsz must be greater than 0")

        self.bridge = CvBridge()
        self.frame_corrector = FrameCorrector(
            enabled=self.correction_enabled,
            gamma=self.correction_gamma,
            clahe_clip_limit=self.correction_clahe_clip_limit,
            clahe_tile_grid=self.correction_clahe_tile_grid,
            chroma_gain=self.correction_chroma_gain,
        )
        self.model = self._load_model(self.model_path)
        self.camera = None
        self.camera_timer = None
        self.image_sub = None
        self.stable_tracks = {}
        self.next_stable_track_id = 1

        self.detections_pub = self.create_publisher(String, self.detections_topic, queue_size)
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, queue_size)
        self.raw_pub = None
        if self.publish_raw:
            self.raw_pub = self.create_publisher(Image, self.raw_topic, queue_size)

        qos = QoSProfile(
            depth=max(1, queue_size),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        if self.input_mode == "topic":
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos,
            )
            self.get_logger().info(f"Subscribing image topic: {self.image_topic}")
        elif self.input_mode == "camera":
            self._start_camera()
        else:
            raise ValueError("input_mode must be either 'topic' or 'camera'")

        self.get_logger().info(f"YOLO model loaded: {self.model_path}")
        self.get_logger().info(
            "Inference preprocessing: "
            f"imgsz={self.imgsz}, enabled={self.correction_enabled}, "
            f"gamma={self.correction_gamma}, "
            f"clahe_clip_limit={self.correction_clahe_clip_limit}, "
            f"clahe_tile_grid={self.correction_clahe_tile_grid}, "
            f"chroma_gain={self.correction_chroma_gain}"
        )
        if self.tracker_enabled:
            self.get_logger().info(f"ByteTrack enabled: tracker={self.tracker_config}")
        self.get_logger().info(f"Publishing detections: {self.detections_topic}")
        if self.annotated_pub is not None:
            self.get_logger().info(f"Publishing annotated images: {self.annotated_topic}")
        if self.raw_pub is not None:
            self.get_logger().info(f"Publishing raw camera images: {self.raw_topic}")

    def _load_model(self, model_path: str) -> Any:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed. Run: pip install ultralytics"
            ) from exc

        resolved_path = Path(model_path).expanduser()
        if not resolved_path.exists():
            raise FileNotFoundError(f"YOLO model file does not exist: {resolved_path}")

        return YOLO(str(resolved_path))

    def _start_camera(self) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("opencv-python is not installed. Run: pip install opencv-python") from exc

        self.cv2 = cv2
        self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            raise RuntimeError(f"Camera index {self.camera_index} could not be opened.")

        if self.camera_width > 0:
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if self.camera_fps > 0:
            self.camera.set(cv2.CAP_PROP_FPS, self.camera_fps)
        self._configure_exposure()

        timer_period = 1.0 / max(self.camera_fps, 1.0)
        self.camera_timer = self.create_timer(timer_period, self.camera_callback)
        self.get_logger().info(f"Opened camera index: {self.camera_index}")
        self._log_camera_settings()

    def _configure_exposure(self) -> None:
        if self.camera is None:
            return

        if self.camera_auto_exposure:
            self.camera.set(self.cv2.CAP_PROP_AUTO_EXPOSURE, self.camera_auto_exposure_value)
            self.get_logger().info("Camera auto exposure enabled")
            return

        self.camera.set(self.cv2.CAP_PROP_AUTO_EXPOSURE, self.camera_manual_auto_exposure_value)
        time.sleep(0.1)
        self.camera.set(self.cv2.CAP_PROP_EXPOSURE, self.camera_exposure)
        self.get_logger().info(f"Camera manual exposure requested: {self.camera_exposure}")

    def _log_camera_settings(self) -> None:
        if self.camera is None:
            return

        width = self.camera.get(self.cv2.CAP_PROP_FRAME_WIDTH)
        height = self.camera.get(self.cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.camera.get(self.cv2.CAP_PROP_FPS)
        exposure = self.camera.get(self.cv2.CAP_PROP_EXPOSURE)
        auto_exposure = self.camera.get(self.cv2.CAP_PROP_AUTO_EXPOSURE)
        self.get_logger().info(
            "Camera settings: "
            f"width={width}, height={height}, fps={fps}, "
            f"exposure={exposure}, auto_exposure={auto_exposure}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert ROS image to OpenCV image: {exc}")
            return

        self._run_inference(frame, msg.header)

    def camera_callback(self) -> None:
        if self.camera is None:
            return

        ok, frame = self.camera.read()
        if not ok:
            self.get_logger().warning("Failed to read frame from camera.")
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame_id

        if self.raw_pub is not None:
            raw_msg = self.frame_to_image_msg(frame, header, encoding="bgr8")
            self.raw_pub.publish(raw_msg)

        self._run_inference(frame, header)

    def _run_inference(self, frame: Any, header: Header) -> None:
        image_height, image_width = frame.shape[:2]
        try:
            inference_frame = self.frame_corrector.apply(frame)
            inference_kwargs = {
                "source": inference_frame,
                "conf": self.confidence,
                "iou": self.iou,
                "imgsz": self.imgsz,
                "verbose": False,
            }
            if self.device:
                inference_kwargs["device"] = self.device

            if self.tracker_enabled:
                inference_kwargs["persist"] = self.tracker_persist
                inference_kwargs["tracker"] = self.tracker_config
                results = self.model.track(**inference_kwargs)
            else:
                results = self.model.predict(**inference_kwargs)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        if not results:
            self._publish_detections(header, [], image_width, image_height)
            return

        result = results[0]
        detections = self._result_to_detections(result)
        if self.stable_tracking_enabled:
            self._assign_stable_track_ids(detections)
        self._publish_detections(header, detections, image_width, image_height)

        if self.annotated_pub is not None:
            annotated = result.plot()
            try:
                annotated_msg = self.frame_to_image_msg(annotated, header, encoding="bgr8")
                self.annotated_pub.publish(annotated_msg)
            except Exception as exc:
                self.get_logger().warning(f"Failed to publish annotated image: {exc}")

    def _result_to_detections(self, result: Any) -> list[dict[str, Any]]:
        detections = []
        names = result.names if hasattr(result, "names") else {}
        boxes = result.boxes

        if boxes is None:
            return detections

        for box in boxes:
            xyxy = box.xyxy[0].detach().cpu().tolist()
            class_id = int(box.cls[0].detach().cpu().item())
            confidence = float(box.conf[0].detach().cpu().item())
            detection = {
                "class_id": class_id,
                "class_name": names.get(class_id, str(class_id)),
                "confidence": confidence,
                "bbox_xyxy": {
                    "x1": float(xyxy[0]),
                    "y1": float(xyxy[1]),
                    "x2": float(xyxy[2]),
                    "y2": float(xyxy[3]),
                },
            }

            track_id = self._box_track_id(box)
            if track_id is not None:
                detection["track_id"] = track_id

            detections.append(detection)

        return detections

    @staticmethod
    def _box_track_id(box: Any) -> int | None:
        track_id = getattr(box, "id", None)
        if track_id is None:
            return None

        try:
            return int(track_id[0].detach().cpu().item())
        except (AttributeError, IndexError, TypeError, ValueError):
            return None

    def _assign_stable_track_ids(self, detections: list[dict[str, Any]]) -> None:
        now_s = self._now_s()
        self._prune_stable_tracks(now_s)

        assigned_stable_ids = set()
        for detection in sorted(detections, key=lambda item: item.get("confidence", 0.0), reverse=True):
            stable_id = self._find_stable_track_match(detection, assigned_stable_ids)
            if stable_id is None:
                stable_id = self.next_stable_track_id
                self.next_stable_track_id += 1

            detection["stable_track_id"] = stable_id
            self.stable_tracks[stable_id] = {
                "bbox_xyxy": self._detection_bbox_tuple(detection),
                "class_id": detection.get("class_id"),
                "track_id": detection.get("track_id"),
                "last_seen_s": now_s,
            }
            assigned_stable_ids.add(stable_id)

    def _find_stable_track_match(self, detection: dict[str, Any], assigned_stable_ids: set[int]) -> int | None:
        raw_track_id = detection.get("track_id")
        class_id = detection.get("class_id")

        if raw_track_id is not None:
            for stable_id, track in self.stable_tracks.items():
                if stable_id in assigned_stable_ids:
                    continue
                if track.get("track_id") == raw_track_id and track.get("class_id") == class_id:
                    return stable_id

        best_stable_id = None
        best_score = 0.0
        detection_bbox = self._detection_bbox_tuple(detection)
        for stable_id, track in self.stable_tracks.items():
            if stable_id in assigned_stable_ids:
                continue
            if track.get("class_id") != class_id:
                continue

            track_bbox = track.get("bbox_xyxy")
            if track_bbox is None:
                continue

            iou = self._bbox_iou(detection_bbox, track_bbox)
            center_similarity = self._bbox_center_similarity(detection_bbox, track_bbox)
            score = max(iou, center_similarity)
            if score > best_score:
                best_score = score
                best_stable_id = stable_id

        if best_score >= self.stable_track_iou_threshold:
            return best_stable_id
        return None

    def _prune_stable_tracks(self, now_s: float) -> None:
        stale_ids = [
            stable_id for stable_id, track in self.stable_tracks.items()
            if now_s - float(track.get("last_seen_s", 0.0)) > self.stable_track_timeout_s
        ]
        for stable_id in stale_ids:
            del self.stable_tracks[stable_id]

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _detection_bbox_tuple(detection: dict[str, Any]) -> tuple[float, float, float, float]:
        bbox = detection["bbox_xyxy"]
        return (
            float(bbox["x1"]),
            float(bbox["y1"]),
            float(bbox["x2"]),
            float(bbox["y2"]),
        )

    def _bbox_center_similarity(
        self,
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        first_x1, first_y1, first_x2, first_y2 = first
        second_x1, second_y1, second_x2, second_y2 = second
        first_cx = (first_x1 + first_x2) * 0.5
        first_cy = (first_y1 + first_y2) * 0.5
        second_cx = (second_x1 + second_x2) * 0.5
        second_cy = (second_y1 + second_y2) * 0.5

        first_width = max(1.0, first_x2 - first_x1)
        first_height = max(1.0, first_y2 - first_y1)
        second_width = max(1.0, second_x2 - second_x1)
        second_height = max(1.0, second_y2 - second_y1)
        scale = max(first_width, first_height, second_width, second_height, 1.0)

        dx = first_cx - second_cx
        dy = first_cy - second_cy
        normalized_distance = ((dx * dx + dy * dy) ** 0.5) / scale
        if normalized_distance > self.stable_track_center_ratio:
            return 0.0
        return 1.0 - normalized_distance / self.stable_track_center_ratio

    @staticmethod
    def _bbox_iou(
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

    def _publish_detections(
        self,
        header: Header,
        detections: list[dict[str, Any]],
        image_width: int | None = None,
        image_height: int | None = None,
    ) -> None:
        payload = {
            "stamp": {
                "sec": header.stamp.sec,
                "nanosec": header.stamp.nanosec,
            },
            "frame_id": header.frame_id,
            "image_width": image_width,
            "image_height": image_height,
            "detections": detections,
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(out)

    def frame_to_image_msg(self, frame: Any, header: Header, encoding: str = "bgr8") -> Image:
        if not frame.flags["C_CONTIGUOUS"]:
            frame = frame.copy()

        msg = Image()
        msg.header = header
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.is_bigendian = 0
        msg.encoding = encoding

        if len(frame.shape) == 2:
            msg.encoding = "mono8"
            channels = 1
        else:
            channels = int(frame.shape[2])
            if channels == 1:
                msg.encoding = "mono8"
            elif channels == 3:
                msg.encoding = encoding
            elif channels == 4:
                msg.encoding = "bgra8"
            else:
                raise ValueError(f"Unsupported image channel count: {channels}")

        msg.step = int(msg.width * channels * frame.dtype.itemsize)
        msg.data = frame.tobytes()
        return msg

    def destroy_node(self) -> bool:
        if self.camera is not None:
            self.camera.release()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
