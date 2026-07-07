# ros2_yolo_detector

ROS 2 Python node for running a YOLO `best.pt` model on Jetson Nano camera images.

Input modes:

- `input_mode:=topic`: subscribe to an existing ROS image topic.
- `input_mode:=camera`: open a USB camera directly with OpenCV/V4L2.

## Install on Jetson

Put this package under your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
# copy ros2_yolo_detector here
```

Install dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge
sudo apt install ros-$ROS_DISTRO-v4l2-camera
python3 -m pip install -r ~/ros2_ws/src/ros2_yolo_detector/requirements.txt
```

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_yolo_detector
source install/setup.bash
```

## Recommended model path

Place your model somewhere outside the source code, for example:

```bash
/home/jetson/models/best.pt
```

## Run with USB camera

Recommended Jetson setup: run `v4l2_camera` with the camera parameters below, then run YOLO on the published image topic.

This matches:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0 \
  -p image_size:="[1280,720]" \
  -p time_per_frame:="[1,10]" \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p power_line_frequency:=2 \
  -p auto_exposure:=1 \
  -p exposure_time_absolute:=200 \
  -p gain:=20
```

Use this launch file to start both `v4l2_camera` and YOLO:

```bash
ros2 launch ros2_yolo_detector v4l2_yolo_camera.launch.py \
  model_path:=/home/jetson/models/best.pt \
  video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0 \
  image_width:=1280 \
  image_height:=720 \
  time_per_frame_numerator:=1 \
  time_per_frame_denominator:=10 \
  pixel_format:=YUYV \
  output_encoding:=rgb8 \
  power_line_frequency:=2 \
  auto_exposure:=1 \
  exposure_time_absolute:=200 \
  gain:=20
```

The YOLO node subscribes to `/image_raw` by default in this launch file. It also starts
`detections_to_target_node` by default so mission nodes can consume `/target_object`.

## Run with direct OpenCV camera

This mode is still available, but the `v4l2_camera` launch above is preferred when you need exact V4L2 controls.

```bash
ros2 launch ros2_yolo_detector yolo_camera.launch.py \
  model_path:=/home/jetson/models/best.pt \
  input_mode:=camera \
  camera_index:=0
```

## Run with ROS camera topic

Use this if another camera driver is already publishing images:

```bash
ros2 launch ros2_yolo_detector yolo_camera.launch.py \
  model_path:=/home/jetson/models/best.pt \
  input_mode:=topic \
  image_topic:=/camera/image_raw
```

## Published topics

- `/yolo/detections`: `std_msgs/msg/String`, JSON detection result
- `/target_object`: `geometry_msgs/msg/PointStamped`, normalized target center and area
- `/target_label`: `std_msgs/msg/String`, selected target class name
- `/yolo/annotated_image`: `sensor_msgs/msg/Image`, image with boxes drawn
- `/camera/image_raw`: optional raw camera image when `publish_raw:=true`

Detection JSON example:

```json
{
  "stamp": {"sec": 123, "nanosec": 456},
  "frame_id": "camera",
  "detections": [
    {
      "class_id": 0,
      "class_name": "target",
      "confidence": 0.91,
      "bbox_xyxy": {"x1": 120.0, "y1": 80.0, "x2": 240.0, "y2": 210.0}
    }
  ]
}
```

Mission nodes can subscribe to `/target_object`; lower-level consumers can subscribe to
`/yolo/detections` for the full class, confidence, and bounding-box payload.
