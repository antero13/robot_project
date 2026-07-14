# rl_model_policy

ROS 2 node that runs the trained Isaac Lab/skrl avoid/search policy.

## Checkpoint contract

Before changing this package, read:

```text
rl_model_policy/MODEL_CHECKPOINT_README.md
```

The currently bundled model at
`mission_manager/models/rl_avoid_search_best.pt` is a 10-observation-input
checkpoint trained only from YOLO-derived values:

```text
policy.net_container.0.weight: (128, 10)
state_preprocessor.running_mean: (10,)
state_preprocessor.running_variance: (10,)
```

New training runs use only the first 10 YOLO-derived observations. The runner
detects the checkpoint input width and supports both contracts:

```text
10 inputs: YOLO target/avoidance values only (new training)
18 inputs: the same 10 values plus 8 legacy pose/IMU values
```

Do not resume an 18-input checkpoint in the 10-observation Isaac Lab
environment. The bundled model and new training environment use the same
10-input contract. The ROS runner still accepts legacy 18-input checkpoints
when one is passed explicitly through `model_path`.

## Inputs

- `/target_object` (`geometry_msgs/PointStamped`)
  - `point.x`: normalized target x in `[-1, 1]`
  - `point.y`: normalized bounding-box bottom y in `[0, 1]`
  - CSV map calibration separately reads the bbox center from `/yolo/detections`
- `/avoid_objects` (`std_msgs/String`)
  - JSON from `ros2_yolo_detector`
- `/odom` (`nav_msgs/Odometry`)
  - arena-center pose used by the no-target coverage controller
  - never included in the bundled 10-input model observation
  - optionally included in a legacy 18-input model with `pose_observation_enabled:=true`

For a new 10-input checkpoint, pose data is never sent to the network. For a
legacy 18-input checkpoint, disabling pose observation supplies zeros for its
last 8 values. Set `pose_observation_enabled:=true` only when deliberately
testing a calibrated legacy pose model.

## Output

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/ros_robot_controller/bus_servo/set_state` for the default bus-servo gripper
- `/rl_estimated_objects` (`std_msgs/String`) for GUI map markers

## Run

Build and source:

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
colcon build --packages-select rl_model_policy
.\install\setup.ps1
```

Start the policy node:

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py
```

Start motion:

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

Stop motion:

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

Monitor:

```powershell
ros2 topic echo /rl_model_policy_state
ros2 topic echo /cmd_vel
```

Use a different checkpoint:

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py model_path:="C:\path\to\best_agent.pt"
```

Use a slower command scale:

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py speed_scale:=0.25
```

## Notes

The robot machine must have PyTorch available in the Python environment used by ROS 2.

## One-command Jetson launch

Build the integrated runtime packages once after pulling the repository:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mission_manager robot_pose_tracker rl_model_policy --symlink-install
source ~/ros2_ws/install/setup.bash
```

Start the controller, camera/YOLO, pose tracker, motor converter, and RL policy
in one terminal. The pose tracker drives coverage waypoints but its values stay
outside the bundled 10-input policy observation:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  launch_pose_tracker:=true \
  pose_observation_enabled:=false
```

The safe default waits for a separate start command:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

For a true one-command start, add `auto_start:=true`. The robot starts moving after an
8 second delay so the camera and YOLO model have time to initialize:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  auto_start:=true
```

Press `Ctrl+C` in the launch terminal to stop all processes started by this launch file.
Do not run `mission_manager`, keyboard teleoperation, or another `/cmd_vel` publisher at
the same time.

`camera_horizontal_fov_deg` defaults to `80.0`, matching the training
environment. With `target_bearing_prediction_enabled:=true`, odometry yaw
projects the last target back into image x during short detection gaps. This
works with the current 10-input policy and does not add pose values to its
observation vector.

`target_timeout_s` defaults to `1.0`. The last target x/y is kept during a
short YOLO detection gap instead of immediately changing to search behavior.
Pickup still requires a raw YOLO detection newer than
`grab_detection_timeout_s` (default `0.25` seconds).

## No-target coverage search

The runtime is a hybrid controller. The learned policy handles a visible
target; deterministic coverage handles the case that the target is absent:

```text
TRACK_TARGET -> LOCAL_REACQUIRE (two-way sweep) -> COVERAGE_SEARCH
                                      |
target detected ----------------------+--> TRACK_TARGET
```

Local reacquisition stops forward motion and searches for three seconds. It
turns toward the last visible target side for 1.5 seconds, then reverses the
turn for another 1.5 seconds. Coverage search starts only if both sweeps fail.

Coverage starts on the lower main road, scans four vertical lanes northward,
reverses down each cleared lane, and shifts to the next lane on the lower road.
If `/odom` is missing or stale, the mode becomes `WAITING_FOR_POSE` and the
robot publishes a stop command. The default lane settings are:

```text
x lanes: 1.25, 0.25, -0.75, -1.75 m
main road y: -1.3343 m
scan end y: 1.0 m
scan speed: 0.14 m/s
main-road speed: 0.18 m/s
cleared-lane reverse speed: 0.20 m/s
```

Inspect the current mode, waypoint, pose, and route leg with:

```bash
ros2 topic echo /rl_model_policy_state
```

Set `coverage_enabled:=false` only to reproduce the old behavior where the RL
policy also receives no-target observations.

The default real-robot configuration is explicit below:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  pose_observation_enabled:=false \
  target_timeout_s:=1.0 \
  target_bearing_prediction_enabled:=true \
  coverage_reacquire_duration_s:=3.0 \
  camera_horizontal_fov_deg:=80.0
```

## Automatic pickup

The policy keeps the gripper closed when a run starts. RL owns target search,
alignment, and the complete visual approach. The runtime takes control only
when the target is centered and its bbox bottom y reaches `grab_area_ratio`.
It then runs the pickup sequence:

```text
TRACKING -> OPENING -> FINAL_FORWARD -> CLOSING -> GRABBED
```

Default bus-servo and pickup settings:

```text
servo ID: 1
open position: 1000
closed position: 300
grab center tolerance: 0.18
grab area ratio: 0.70
final forward: 0.20 m/s for 1.0 s
stop after grab: false
```

Override values from the integrated launch when needed:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  gripper_open_position:=1000 \
  gripper_closed_position:=300 \
  grab_area_ratio:=0.70 \
  final_forward_linear_x:=0.20 \
  final_forward_duration_s:=1.0
```

Monitor the pickup state:

```bash
ros2 topic echo /rl_model_policy_state
```

Manual gripper commands use the same control topic:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: open}"
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: close}"
```

## Status GUI and motion pause

Pass `launch_status_gui:=true` to open the PyQt status window. The autonomous
launch also starts `rl_object_world_mapper`, which estimates each detected
object's continuous arena position. It interpolates the measured bbox-center
calibration in `config/distance_normalized_points.csv`; it does not snap object
markers to the 42 legal placement points.

The GUI also subscribes to `/yolo/detections` directly. If the standalone
`/rl_estimated_objects` mapper stream is missing for more than two seconds, the
GUI automatically performs the same CSV interpolation as a display-only
fallback. These fallback coordinates are never added to the policy observation
and never affect `/cmd_vel`. Consequently, `ROS data 3/3` means that odometry,
policy state, and either the mapper stream or raw YOLO detections are current.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  launch_status_gui:=true
```

Inspect all three object-display inputs on Jetson with:

```bash
ros2 topic hz /yolo/detections
ros2 topic hz /odom
ros2 topic hz /rl_estimated_objects
```

If `/rl_estimated_objects` is absent but the first two topics are active, the
GUI logs that it has switched to `gui_detection_fallback` and still draws
calibrated object markers.

The GUI pause button publishes `pause_motion`; perception, policy calculations,
and state topics continue while the published base velocity remains zero.

## Full competition mission

The RL target controller now runs below a deterministic mission coordinator:

```text
COLLECTING
  -> RETURN_MAIN_ROAD
  -> RETURN_STAGING
  -> ENTER_STORAGE
  -> DEPOSIT
  -> EXIT_STORAGE
  -> CLOSE_AFTER_DEPOSIT
  -> COLLECTING or COMPLETE
```

The default capacity is four objects and the mission target is seven objects.
After four pickups, or after collecting the seventh object, the robot follows
odometry waypoints to the lower-left Storage Zone. With 30 seconds remaining it
also returns whenever at least one object is onboard. An empty robot continues
searching until it picks an object or the 180 second match expires.

Inside storage the gripper opens, the robot reverses back to the staging point,
and the gripper closes before collection resumes. The default centered-frame
waypoints are:

```text
main road y: -1.3343 m
storage staging: (-1.75, -1.25) m
storage center:  (-1.75, -1.75) m
entry heading:   -90 deg
```

Tune these from the integrated launch if the real robot center does not land
inside the 40 cm Storage Zone:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  storage_staging_x:=-1.75 storage_staging_y:=-1.25 \
  storage_center_x:=-1.75 storage_center_y:=-1.75 \
  storage_entry_yaw_deg:=-90.0
```
