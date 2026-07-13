import json
import time
from pathlib import Path
from typing import Any

import cv2
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
        self.get_logger().info("Per-frame YOLO prediction enabled; tracking is disabled")
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

            results = self.model.predict(**inference_kwargs)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        result = results[0] if results else None
        detections = self._result_to_detections(result) if result is not None else []
        self._publish_detections(header, detections, image_width, image_height)

        if self.annotated_pub is not None and result is not None:
            annotated = result.plot()
            self._draw_normalized_coordinates(
                annotated,
                detections,
                image_width,
                image_height,
            )
            try:
                annotated_msg = self.frame_to_image_msg(annotated, header, encoding="bgr8")
                self.annotated_pub.publish(annotated_msg)
            except Exception as exc:
                self.get_logger().warning(f"Failed to publish annotated image: {exc}")

    @staticmethod
    def _draw_normalized_coordinates(
        frame: Any,
        detections: list[dict[str, Any]],
        image_width: int,
        image_height: int,
    ) -> None:
        if image_width <= 0 or image_height <= 0:
            return

        for detection in detections:
            bbox = detection.get("bbox_xyxy", {})
            try:
                x1 = float(bbox["x1"])
                x2 = float(bbox["x2"])
                y2 = float(bbox["y2"])
            except (KeyError, TypeError, ValueError):
                continue

            center_x = (x1 + x2) * 0.5
            normalized_x = max(-1.0, min(1.0, (center_x - image_width * 0.5) / (image_width * 0.5)))
            normalized_y = max(0.0, min(1.0, y2 / image_height))
            label = f"x={normalized_x:+.3f}  y={normalized_y:.3f}"

            marker_x = max(0, min(image_width - 1, int(round(center_x))))
            marker_y = max(0, min(image_height - 1, int(round(y2))))
            cv2.drawMarker(
                frame,
                (marker_x, marker_y),
                (0, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=2,
            )

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.55
            thickness = 2
            (text_width, text_height), baseline = cv2.getTextSize(
                label,
                font,
                font_scale,
                thickness,
            )
            text_x = max(0, min(image_width - text_width - 8, int(round(x1))))
            text_y = marker_y + text_height + 10
            if text_y + baseline + 4 >= image_height:
                text_y = max(text_height + 4, marker_y - 10)

            cv2.rectangle(
                frame,
                (text_x, text_y - text_height - 4),
                (text_x + text_width + 8, text_y + baseline + 4),
                (0, 0, 0),
                thickness=-1,
            )
            cv2.putText(
                frame,
                label,
                (text_x + 4, text_y),
                font,
                font_scale,
                (0, 255, 255),
                thickness,
                lineType=cv2.LINE_AA,
            )

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

            detections.append(detection)

        return detections

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
