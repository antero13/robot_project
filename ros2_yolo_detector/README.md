# ros2_yolo_detector

ROS 2 Python node for running a YOLO `.pt` or TensorRT `.engine` model on
Jetson camera images.

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

## Included model path

The default launch files use the model included in this package:

```text
ros2_yolo_detector/models/best.pt
```

After `colcon build`, the model is installed under the package share directory, so
you do not need to pass `model_path` unless you want to use a different model.

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
  gain:=20 \
  imgsz:=800 \
  correction_enabled:=true \
  correction_gamma:=0.65 \
  correction_clahe_clip_limit:=1.2 \
  correction_clahe_tile_grid:=8 \
  correction_chroma_gain:=1.3 \
  correction_backend:=auto \
  correction_device:=cuda:0
```

The YOLO node subscribes to `/image_raw` by default in this launch file. It also starts
`detections_to_target_node` by default so mission nodes can consume `/target_object`.

## Real-time image correction

The YOLO node applies deterministic correction immediately before inference. The
camera's `/image_raw` topic is not modified. Default inference settings match the
corrected training dataset:

```text
imgsz: 800
correction_enabled: true
correction_gamma: 0.65
correction_clahe_clip_limit: 1.2
correction_clahe_tile_grid: 8
correction_chroma_gain: 1.3
```

`correction_backend:=auto` uses PyTorch and Kornia to apply gamma, LAB CLAHE,
chroma gain, resize, and letterbox on CUDA. The corrected CUDA tensor is passed
directly to `YOLO.predict()`, so it is not downloaded to CPU and uploaded again
for TensorRT. Bounding boxes are transformed back to the original camera frame
before publication. If CUDA or Kornia is unavailable, `auto` logs a warning and
falls back to the existing OpenCV CPU correction. Use
`correction_backend:=cuda` to fail startup instead of falling back, or
`correction_backend:=cpu` to force the original path.

Disable all correction without changing the remaining settings by passing
`correction_enabled:=false` to a launch file.

For the integrated RL launch, pass the TensorRT engine and preprocessing options
at startup:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/absolute/path/to/best.engine \
  secondary_yolo_model_path:=/absolute/path/to/best_secondary.engine \
  yolo_imgsz:=800 \
  correction_backend:=auto \
  correction_device:=cuda:0
```

The startup log reports the selected correction backend. Install dependencies
from `requirements.txt` before rebuilding so the CUDA path has Kornia available.
Every five seconds the node also reports output Hz, total pipeline latency, and
Ultralytics preprocess, inference, and postprocess latency. Set the node
parameter `performance_log_interval_s` to `0` to disable these logs. The
integrated RL launch exposes it as `yolo_performance_log_interval_s`.

## Two-stage fruit classification

The primary detector passes classes `0` through `3` directly to ROS. Primary
classes `4` through `7` are treated as fruit-cube candidates: their crops are
classified by the secondary YOLO model, whose classes `0` through `3` are
published as final classes `4` through `7`. If `secondary_model_path` is empty,
the node looks for `best_secondary.pt` beside the primary model. Pass an explicit
`.pt` or `.engine` path when the secondary model uses a different filename.

`target_classes` and `avoid_classes` accept final numeric class IDs only. Class
names and prefixes such as `orange` or `id:6` are rejected. The final IDs are:

```text
0=12, 1=20, 2=6, 3=8,
4=apple, 5=banana, 6=orange, 7=pineapple
```

For example, use `target_classes:=2` for the geometric class named `6`, and
use `target_classes:=6` for orange.

Useful parameters are `secondary_confidence`, `secondary_imgsz`, and
`min_bbox_area_ratio`. The integrated autonomous launch exposes them as
`secondary_yolo_confidence`, `secondary_yolo_imgsz`, and
`yolo_min_bbox_area_ratio`.

In `object_pickup_mission.launch.py`, these correction settings and `imgsz=800`
apply only to camera1. The optional camera2 YOLO node explicitly disables image
correction and uses `second_imgsz=640`.

## Run with direct OpenCV camera

This mode is still available, but the `v4l2_camera` launch above is preferred when you need exact V4L2 controls.

```bash
ros2 launch ros2_yolo_detector yolo_camera.launch.py \
  input_mode:=camera \
  camera_index:=0
```

## Run with ROS camera topic

Use this if another camera driver is already publishing images:

```bash
ros2 launch ros2_yolo_detector yolo_camera.launch.py \
  input_mode:=topic \
  image_topic:=/camera/image_raw
```

## Published topics

- `/yolo/detections`: `std_msgs/msg/String`, JSON detection result
- `/target_object`: `geometry_msgs/msg/PointStamped`, normalized bounding-box center x/y
- `/target_label`: `std_msgs/msg/String`, selected target class name
- `/avoid_object`: `geometry_msgs/msg/PointStamped`, closest selected avoid object
- `/avoid_objects`: `std_msgs/msg/String`, JSON list of all selected avoid objects for VFH-lite avoidance
- `/yolo/annotated_image`: `sensor_msgs/msg/Image`, image with boxes and normalized x/y coordinates drawn
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

Mission nodes can subscribe to `/target_object` and `/avoid_objects`.
`/avoid_objects` includes bbox-center `x`/`y`, the compatibility field
`center_y`, box-bottom `bottom_y`, confidence, and bounding-box payload for
each avoid object.
Lower-level consumers can subscribe to `/yolo/detections` for the full class,
confidence, and bounding-box payload.

The detector does not run ByteTrack or assign object IDs. The target converter
does retain a spatial target lock across short gaps, however: IoU and normalized
center distance keep control attached to the same physical box and prevent an
instant jump to another same-class object. Configure this with the
`target_lock_*` launch parameters.

## Annotated Normalized Coordinates

When `publish_annotated:=true`, every YOLO box includes a yellow marker and a
label such as:

```text
x=-0.352  rl_y=0.620  c_y=0.440
```

The values explicitly separate the RL closeness input from calibration:

```text
x = (bbox center x - image center x) / image center x   [-1, 1]
rl_y = bbox bottom / image height                       [0, 1]
c_y = bbox center y / image height                      [0, 1]
```

The yellow cross and `c_y` identify the bbox center used only by the CSV GUI
mapper. `/target_object.point.y` and `/avoid_objects.y` use `rl_y`; no calibrated
world coordinate is fed into the policy. These values are normalized camera
observations, not physical meters.

```bash
ros2 launch ros2_yolo_detector v4l2_yolo_camera.launch.py \
  publish_annotated:=true
ros2 run rqt_image_view rqt_image_view /yolo/annotated_image
```

`detections_to_target_node` removes avoid candidates whose bounding boxes
overlap target candidates by `avoid_target_iou_threshold` or more. The default
threshold is `0.35`, which helps prevent the same close object from being
published as both target and avoid.
