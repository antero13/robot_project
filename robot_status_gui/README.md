# Robot Status GUI

PyQt5 operator GUI for real-robot tests. It shows:

- robot position and heading in the 4 m x 4 m arena
- RL/search/grab state and the currently stored object classes
- full-mission phase, remaining match time, onboard capacity, and delivered count
- YOLO object positions estimated from `/odom` and measured bbox calibration
- the active odometry waypoint while returning to Storage Zone
- a base-motion pause/resume button

The map uses the competition drawing's lower-left origin. Internally, `/odom`
uses an arena-centered frame, so the GUI adds `(2.0, 2.0)`. The default start
pose `(1.8, -1.8)` is therefore displayed at `(3.8, 0.2)`.

## Install and build

```bash
sudo apt update
sudo apt install -y python3-pyqt5

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rl_model_policy robot_status_gui
source ~/ros2_ws/install/setup.bash
```

## Run separately

Start the autonomous runtime first, then open the GUI on the Jetson desktop or
through NoMachine:

```bash
ros2 launch robot_status_gui robot_status_gui.launch.py
```

## Run with the autonomous launch

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  launch_status_gui:=true \
  publish_annotated:=true \
  auto_start:=false
```

The GUI is optional and defaults to off so the autonomous launch still works
over a headless SSH session.

During the full mission the sidebar shows `collected/7`, `onboard/4`, and
`delivered/7`. Storage return reasons distinguish capacity, seventh-object,
30-second, and manual returns. The yellow `W` marker is the next centered-frame
waypoint selected by the storage return controller.

## Object position calibration

`/rl_estimated_objects` is not depth-camera ground truth. The mapper compares
each YOLO bounding-box center with the measured samples in
`robot_status_gui/config/distance_normalized_points.csv`. It interpolates the
camera-relative lateral and forward distance, rotates that vector with `/odom`,
and publishes a continuous arena coordinate. It never snaps a marker to one of
the 42 placement points. A point is shown after two confirmations, retained for
the match, and removed when a nearby target is reported as stored.

The GUI also has a display-only fallback. If `/rl_estimated_objects` stops,
it subscribes to `/yolo/detections` and performs the same interpolation inside
the GUI process. This data is used only for map markers and never changes RL
observations, mission state, or `/cmd_vel`.

```text
calibrated forward range: 0.3 to 1.8 m
track association radius: 0.30 m
position smoothing alpha: 0.35
```

Use another calibration file or tune continuous tracking from the integrated
launch:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  object_calibration_path:=/home/airobot/calibration/new_points.csv \
  object_association_radius_m:=0.30 \
  object_position_smoothing_alpha:=0.35 \
  launch_status_gui:=true
```

Position accuracy depends on the calibration setup matching the actual camera
mount and on `/odom` accuracy. Detections outside the measured image region are
rejected instead of extrapolated to a misleading map point.

The GUI connection header names whichever input is missing. Its object input is
connected when either `/rl_estimated_objects` or `/yolo/detections` is current.
If it still shows `ROS data 2/3` and `object position waiting`, inspect both:

```bash
ros2 node list | grep rl_object_world_mapper
ros2 topic hz /rl_estimated_objects
ros2 topic echo --once /rl_estimated_objects
ros2 topic hz /yolo/detections
```

The diagnostics contain `mapper_status`, `pose_fresh`, `detection_count`, and
`mapped_count`. `detections_outside_calibration` means YOLO is publishing but
the bbox center is outside the measured CSV range.

## Pause semantics

The button publishes `pause_motion` or `resume_motion` to
`/rl_model_policy_control`. Pause keeps perception, policy inference, mapping,
and status topics alive while forcing base velocity to zero. A grab sequence is
held at its current logical step until resume. A servo command already sent to
the controller cannot be physically cancelled mid-command.
