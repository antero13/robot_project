# RL Model Policy Run Guide

The bundled `mission_manager/models/rl_avoid_search_best.pt` checkpoint uses
10 YOLO-derived observations with no pose or IMU input. The ROS runner detects
the checkpoint input width automatically and also keeps compatibility with the
legacy 18-observation contract documented in
`rl_model_policy/MODEL_CHECKPOINT_README.md`.

## Data Flow

```text
camera + YOLO -> /target_object, /avoid_objects
robot_pose_tracker -> /odom (started, but ignored by the policy by default)
rl_model_policy -> /cmd_vel
cmd_vel_to_motor -> motor controller
```

The integrated launch starts all of these nodes. Do not run `mission_manager`,
keyboard teleoperation, another pose tracker, or another `/cmd_vel` publisher at
the same time.

## Jetson Build

```bash
cd ~/ros2_ws/src/robot_project
git pull --ff-only origin main

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  ros2_yolo_detector cmd_vel_to_motor robot_pose_tracker \
  mission_manager rl_model_policy
source ~/ros2_ws/install/setup.bash
```

## Dry Run

The arena coordinate frame is centered at `(0, 0)`. The lower-right start zone
is approximately `(1.8, -1.8)`. Set `initial_yaw_deg` to the robot's real start
heading.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  initial_x:=1.8 \
  initial_y:=-1.8 \
  initial_yaw_deg:=90.0 \
  speed_scale:=0.25 \
  dry_run:=true \
  auto_start:=false
```

Start the policy in another sourced terminal:

```bash
ros2 topic pub --once /rl_model_policy_control \
  std_msgs/msg/String "{data: start}"
```

Check the complete observation vector:

```bash
ros2 topic echo /rl_model_policy_state
```

Expected state:

```text
model_loaded: true
observation_dim: 10
obs: 10 values
pose_observation_enabled: false
```

The bundled model reports `observation_dim: 10` and never receives pose values.
A legacy model reports `observation_dim: 18`; with pose disabled its final 8
values are zeros.

The policy keeps the last YOLO target for `target_timeout_s=0.8` seconds so a
one- or two-frame detection gap does not immediately switch into search mode.
Because pose input is disabled by default, the held target x/y remains the last
camera observation rather than being adjusted from odometry.

The camera horizontal field of view defaults to `80.0` degrees to match the
training environment. Override these settings only when testing a calibrated
alternative:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  target_timeout_s:=0.8 \
  pose_observation_enabled:=false \
  camera_horizontal_fov_deg:=80.0
```

## Annotated YOLO Coordinates

Enable the annotated image to see the exact normalized observation coordinates
next to every YOLO box:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  publish_annotated:=true
ros2 run rqt_image_view rqt_image_view /yolo/annotated_image
```

The yellow label uses `x=-1..1` from the bounding-box center and `y=0..1` from
the bounding-box bottom edge. These are normalized policy inputs, not meters.

## Low-Speed Robot Test

After the dry run succeeds, stop the launch and run it again with
`dry_run:=false`. Keep `speed_scale:=0.25` for the first physical test.

Emergency stop:

```bash
ros2 topic pub --once /rl_model_policy_control \
  std_msgs/msg/String "{data: stop}"
```

The integrated launch loads the model installed at
`mission_manager/models/rl_avoid_search_best.pt`, whose first layer is
`[128, 10]`. The runner also supports legacy `[128, 18]` checkpoints. Any other
input width is rejected.
