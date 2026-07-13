# Robot Status GUI

PyQt5 operator GUI for real-robot tests. It shows:

- robot position and heading in the 4 m x 4 m arena
- RL/search/grab state and the currently stored object classes
- YOLO object positions estimated from `/odom` and camera geometry
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

## Object position calibration

`/rl_estimated_objects` is not depth-camera ground truth. The mapper compares
each YOLO bounding-box center with the expected image position of the 42 legal
object points and chooses the best matching point. A point is shown after two
confirmations, retained for the match, and removed when a nearby target is
reported as stored. The defaults are:

```text
horizontal FOV: 80 deg
vertical FOV: 50 deg
camera optical-center height: 0.18 m
camera downward pitch: 15 deg
object center height: 0.04 m
```

Measure the real optical-center height and camera pitch, then override them:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  camera_height_m:=0.21 \
  camera_pitch_deg:=18.0 \
  camera_vertical_fov_deg:=52.0 \
  launch_status_gui:=true
```

The object height rule is 8 cm, so its center defaults to 4 cm. Position
accuracy depends directly on `/odom` accuracy and these camera parameters.

## Pause semantics

The button publishes `pause_motion` or `resume_motion` to
`/rl_model_policy_control`. Pause keeps perception, policy inference, mapping,
and status topics alive while forcing base velocity to zero. A grab sequence is
held at its current logical step until resume. A servo command already sent to
the controller cannot be physically cancelled mid-command.
