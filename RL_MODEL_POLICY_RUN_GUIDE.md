# RL Model Policy Run Guide

The bundled `mission_manager/models/rl_avoid_search_best.pt` checkpoint uses
10 YOLO-derived observations with no pose or IMU input. The ROS runner detects
the checkpoint input width automatically and also keeps compatibility with the
legacy 18-observation contract documented in
`rl_model_policy/MODEL_CHECKPOINT_README.md`.

## Data Flow

```text
camera + YOLO -> /target_object, /avoid_objects
robot_pose_tracker -> /odom (coverage route; not part of the 10-input network)
rl_object_world_mapper -> /rl_estimated_objects
rl_model_policy -> /cmd_vel
cmd_vel_to_motor -> motor controller
robot_status_gui <- /odom, /rl_model_policy_state, /rl_estimated_objects
```

The integrated launch starts the camera, controller, pose tracker, motor
converter, policy, and object mapper. The status GUI can be enabled with
`launch_status_gui:=true`. Do not run `mission_manager`, keyboard
teleoperation, another pose tracker, or another `/cmd_vel` publisher at the
same time.

## Jetson Build

```bash
cd ~/ros2_ws/src/robot_project
git pull --ff-only origin main

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  ros2_yolo_detector cmd_vel_to_motor robot_pose_tracker \
  mission_manager robot_status_gui rl_model_policy
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
  launch_pose_tracker:=true \
  pose_observation_enabled:=false \
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
control_mode: COVERAGE_SEARCH or TRACK_TARGET
pose_fresh: true
```

The bundled model reports `observation_dim: 10` and never receives pose values,
even if `pose_observation_enabled:=true` is passed accidentally.
A legacy model reports `observation_dim: 18`; with pose disabled its final 8
values are zeros.

The policy keeps the last YOLO target for `target_timeout_s=0.8` seconds so a
one- or two-frame detection gap does not immediately switch into search mode.
Because pose input is disabled by default, the held target x/y remains the last
camera observation rather than being adjusted from odometry.

When no target has ever been seen, the node immediately enters
`COVERAGE_SEARCH`. After losing a target it first enters `LOCAL_REACQUIRE` for
0.8 seconds, then resumes coverage. A new target immediately switches control
back to `TRACK_TARGET`. Coverage uses `/odom` independently of the network's
10-value observation contract; stale odometry produces `WAITING_FOR_POSE` and
a zero velocity command.

The default coverage route scans `x = 1.25, 0.25, -0.75, -1.75 m` from the
lower main road at `y = -1.3343 m` up to `y = 1.0 m`. Tune it from the launch
command when the real arena alignment differs:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  coverage_min_x:=-1.75 \
  coverage_max_x:=1.25 \
  coverage_main_road_y:=-1.3343 \
  coverage_scan_end_y:=1.0 \
  coverage_lane_spacing:=1.0
```

To run a legacy 18-input checkpoint with calibrated odometry, explicitly pass
both `launch_pose_tracker:=true` and `pose_observation_enabled:=true`.

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

The yellow label uses `x=-1..1` and `y=0..1` from the bounding-box center.
These are the calibration-aligned normalized policy inputs, not meters.

## Low-Speed Robot Test

After the dry run succeeds, stop the launch and run it again with
`dry_run:=false`. Keep `speed_scale:=0.25` for the first physical test.

Emergency stop:

```bash
ros2 topic pub --once /rl_model_policy_control \
  std_msgs/msg/String "{data: stop}"
```

Operator pause keeps inference and status updates alive while stopping only
the base command:

```bash
ros2 topic pub --once /rl_model_policy_control \
  std_msgs/msg/String "{data: pause_motion}"

ros2 topic pub --once /rl_model_policy_control \
  std_msgs/msg/String "{data: resume_motion}"
```

Open the GUI in the integrated launch:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  launch_status_gui:=true \
  auto_start:=false
```

The object positions shown on the map are estimates snapped to the 42 legal
placement points. Tune `camera_height_m`, `camera_pitch_deg`, and
`camera_vertical_fov_deg` to the real camera installation before relying on
their distance.

The integrated launch loads the model installed at
`mission_manager/models/rl_avoid_search_best.pt`, whose first layer is
`[128, 10]`. The runner also supports legacy `[128, 18]` checkpoints. Any other
input width is rejected.
