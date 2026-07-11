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
  correction_chroma_gain:=1.3
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

Gamma lookup data and the CLAHE object are created once when the node starts and
reused for every frame. Disable all correction without changing the remaining
settings by passing `correction_enabled:=false` to a launch file.

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
- `/target_object`: `geometry_msgs/msg/PointStamped`, normalized target center x and box-bottom y closeness
- `/target_label`: `std_msgs/msg/String`, selected target class name
- `/avoid_object`: `geometry_msgs/msg/PointStamped`, closest selected avoid object
- `/avoid_objects`: `std_msgs/msg/String`, JSON list of all selected avoid objects for VFH-lite avoidance
- `/yolo/annotated_image`: `sensor_msgs/msg/Image`, image with boxes drawn
- `/camera/image_raw`: optional raw camera image when `publish_raw:=true`

Detection JSON example:

```json
{
  "stamp": {"sec": 123, "nanosec": 456},
  "frame_id": "camera",
  "detections": [
    {
      "stable_track_id": 3,
      "track_id": 7,
      "class_id": 0,
      "class_name": "target",
      "confidence": 0.91,
      "bbox_xyxy": {"x1": 120.0, "y1": 80.0, "x2": 240.0, "y2": 210.0}
    }
  ]
}
```

Mission nodes can subscribe to `/target_object` and `/avoid_objects`.
`/avoid_objects` includes `x`, box-bottom `y`, bbox-center `center_y`,
confidence, tracking ID, and bounding-box payload for each avoid object.
Lower-level consumers can subscribe to `/yolo/detections` for the full class,
confidence, tracking ID, and bounding-box payload.

ByteTrack is enabled by default in `yolo_camera_node` through Ultralytics:

```text
tracker_enabled: true
tracker_config: bytetrack.yaml
tracker_persist: true
stable_tracking_enabled: true
stable_track_timeout_s: 1.0
stable_track_iou_threshold: 0.15
stable_track_center_ratio: 0.75
```

`track_id` is the raw ByteTrack ID. It can change when detection briefly drops
or the box moves abruptly. `stable_track_id` is an extra package-level ID that
matches by raw track ID first, then by box overlap and center distance. Mission
target locking uses this stable ID when available.

When multiple target objects are visible, `detections_to_target_node` keeps a
short target lock so the published `/target_object` does not switch left and
right every frame. If a `stable_track_id` or ByteTrack `track_id` is available,
that ID is used first. Otherwise, the selected target is kept while its box still
overlaps the previous target or stays near the previous normalized x/y position.
A new target can take over only when it is clearly closer or has a much better
center-weighted score.

Target lock defaults:

```text
target_lock_enabled: true
target_lock_timeout_s: 0.7
target_lock_iou_threshold: 0.20
target_lock_x_margin: 0.30
target_lock_y_margin: 0.20
target_switch_y_margin: 0.12
target_switch_score_margin: 0.25
target_center_weight: 0.25
```

`detections_to_target_node` removes avoid candidates whose bounding boxes
overlap target candidates by `avoid_target_iou_threshold` or more. The default
threshold is `0.35`, which helps prevent the same close object from being
published as both target and avoid.
