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
        self.declare_parameter("tracker_enabled", True)
        self.declare_parameter("tracker_config", "bytetrack.yaml")
        self.declare_parameter("tracker_persist", True)
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
        self.tracker_enabled = self.get_parameter("tracker_enabled").get_parameter_value().bool_value
        self.tracker_config = self.get_parameter("tracker_config").get_parameter_value().string_value
        self.tracker_persist = self.get_parameter("tracker_persist").get_parameter_value().bool_value
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

        self.bridge = CvBridge()
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
            inference_kwargs = {
                "source": frame,
                "conf": self.confidence,
                "iou": self.iou,
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
