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
- `/wall/distance_angle` (`geometry_msgs/Vector3Stamped`)
  - `vector.x`: perpendicular wall distance from the two VL53L1X sensors
  - used for lane x alignment and storage-staging x/y alignment

For a new 10-input checkpoint, pose data is never sent to the network. For a
legacy 18-input checkpoint, disabling pose observation supplies zeros for its
last 8 values. Set `pose_observation_enabled:=true` only when deliberately
testing a calibrated legacy pose model.

## Output

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/ros_robot_controller/bus_servo/set_state` for the default bus-servo gripper
- `/rl_estimated_objects` (`std_msgs/String`) for GUI map markers
- `/robot_pose/correct_x` (`std_msgs/Float64`) after lane or storage x alignment
- `/robot_pose/correct_y` (`std_msgs/Float64`) after storage y alignment

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
colcon build --packages-up-to mission_manager wall_distance_sensor robot_pose_tracker rl_model_policy --symlink-install
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

Local reacquisition stops forward motion and searches for 1.5 seconds. It
turns toward the last visible target side for 0.75 seconds, then reverses the
turn for another 0.75 seconds. Coverage search starts only if both sweeps fail.

The seven legal object columns are spaced by 0.5 m. Coverage uses four lanes at
centered-frame x coordinates `1.25`, `0.25`, `-0.75`, and `-1.25 m`. In each
lane the robot drives north, turns 180 degrees in place at the top, and drives
south with its front camera facing the travel direction. It shifts to the next
lane only on the lower road. The initial route is lanes 1->2->3->4; after a
storage deposit the controller is rebuilt and searches the reverse route
4->3->2->1 from the main road.

Small heading errors are corrected while moving. When a non-target object gets
too close during a lane scan, the robot keeps 70% of its forward speed and adds
a steering correction toward the clearer side. In-place rotation is reserved
for large direction changes such as the 180-degree turn at a lane end.

If `/odom` is missing or stale, the mode becomes `WAITING_FOR_POSE` and the
robot publishes a stop command. The default lane settings are:

```text
x corridors: 1.25, 0.25, -0.75, -1.25 m (lanes 1, 2, 3, 4)
main road y: -1.3343 m
scan end y: 1.0 m
upward scan speed: 0.24 m/s
downward scan speed: 0.24 m/s
lane-shift speed: 0.30 m/s
coverage angular limit: 1.00 rad/s
waypoint tolerance: 0.10 m
curve-avoid forward scale: 0.70
```

The integrated launch starts `wall_distance_sensor` by default. After returning
south on a lane, `SHIFT_TO_NEXT_LANE` first drives to the next lane center
with the existing odometry waypoint controller. ToF is not required during this
move. Only after odometry enters the waypoint tolerance does the robot turn
toward the wall selected from the lane-shift direction. Normal lane order
(1 -> 2 -> 3 -> 4) uses the west wall at x=-2.0; after storage, reverse lane
order (4 -> 3 -> 2 -> 1) uses the east wall at x=2.0. A fresh VL53L1X range is
then used to measure and correct the remaining x error to the configured
tolerance (3 cm by default) before the next scan leg begins. Coverage and
storage waypoints, `robot_x/y`, and pose correction topics all use the
geometric body center as their coordinate reference. When alignment completes,
the policy publishes the target body-center x to `/robot_pose/correct_x`. The
pose tracker changes x only; y, IMU yaw, and accumulated travel counters are
preserved. If the ToF message is stale after waypoint arrival, the robot stops
with phase `WAITING_FOR_LANE_TOF`.

The wall coordinate and sensor offset can be overridden as follows:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  lane_tof_left_wall_x_m:=-2.0 \
  lane_tof_right_wall_x_m:=2.0 \
  lane_tof_sensor_forward_offset_m:=0.09
```

`lane_tof_sensor_forward_offset_m` is the distance from the body center to the
ToF sensor plane. For the 40 cm chassis, the body center is at 20 cm and the
sensor is 11 cm behind the front edge, so the sensor position is 29 cm and the
offset is 29 - 20 = 9 cm. The rotation center at y=16 cm affects the physical
turning motion, but it is not used as the map-coordinate origin. Disable only
this correction with
`lane_tof_correction_enabled:=false`; disable launching the hardware node with
`launch_wall_distance_sensor:=false`.

The integrated sensor launch uses VL53L1X mode `3` (long range) by default.
Override it only when needed with `wall_ranging_mode:=1` or `2`.

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
  coverage_reacquire_duration_s:=1.5 \
  coverage_min_x:=-1.25 coverage_max_x:=1.25 \
  coverage_avoid_linear_scale:=0.70 \
  coverage_rejoin_speed:=0.20 \
  coverage_turn_in_place_threshold:=0.65 \
  target_activation_center_y_min:=0.30 \
  camera_horizontal_fov_deg:=80.0
```

A newly detected target can interrupt coverage only after its normalized bbox
center y reaches `target_activation_center_y_min`. The default `0.30` is about
0.6 m in the measured camera calibration. Once RL tracking has started, this
gate is not reapplied, so detection jitter cannot bounce the controller back
into coverage.

After a successful pickup, a scan leg first enters `ALIGN_REJOIN_LANE`, rotates
in place toward `(lane_x, current_robot_y)`, and then drives straight in
`REJOIN_LANE`. It resumes the interrupted up/down scan only after reaching that
same-y lane point.

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
final forward: 0.20 m/s for 1.2 s
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
  final_forward_duration_s:=1.2
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
  -> REJOIN_STORAGE_LANE
  -> CORRECT_STORAGE_Y
  -> ALIGN_STORAGE_ENTRY
  -> OPEN_STORAGE_ENTRY
  -> CORRECT_STORAGE_X
  -> EXIT_STORAGE
  -> CLOSE_STORAGE_EXIT
  -> RETURN_FROM_STORAGE
  -> COLLECTING (lanes 4 -> 3 -> 2 -> 1) or COMPLETE
```

The default capacity is four objects and the mission target is seven objects.
After four pickups, or after collecting the seventh object, the robot follows
the existing lane-rejoin motion before starting the storage route. With 30
seconds remaining it also returns whenever at least one object is onboard. An
empty robot continues searching until it picks an object or the 180 second
match expires.

The rejoin motion returns to the active search lane x at the pickup y. The robot
then faces south on that lane and bottom-wall ToF drives the body center directly
to `y=-1.75 m`; it does not first visit main-road `y=-1.3343 m` or staging
`x=-1.25 m`. With the 9 cm sensor offset the expected wall distance is 16 cm. A
stale ToF measurement holds the robot instead of entering storage.

After Y alignment the robot turns right to face west, opens the gripper, and
waits for the servo motion. West-wall ToF then drives to body-center
`x=-1.75 m` at a fixed `0.25 m/s` without approach slowdown. The gripper stays
open while the robot reverses east to `x=-1.25 m`. Only after the reverse exit
finishes does the robot stop, close the gripper, and wait for the servo motion.
It then turns right to face north and returns to the main road at `y=-1.3343 m`
before collection can resume.

The initial coverage route includes lanes 1, 2, 3, and 4. After a storage trip,
a new reverse controller starts at lane 4 and scans x positions `-1.25`,
`-0.75`, `0.25`, and `1.25 m` in that order. The default body-center
waypoints are:

```text
main road y:       -1.3343 m
storage Y target:  (active lane x, -1.75) m
storage center:    (-1.75, -1.75) m
storage X exit:    (-1.25, -1.75) m
Y approach yaw:    -90 deg (south)
X entry yaw:       180 deg (west)
road return yaw:    90 deg (north)
```

Tune these from the integrated launch if the real robot center does not land
inside the 40 cm Storage Zone:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  storage_staging_y:=-1.75 \
  storage_center_x:=-1.75 storage_center_y:=-1.75 \
  storage_exit_x:=-1.25 \
  storage_entry_yaw_deg:=-90.0 \
  storage_tof_left_wall_x_m:=-2.0 \
  storage_tof_bottom_wall_y_m:=-2.0 \
  storage_tof_sensor_forward_offset_m:=0.09
```

Set `storage_tof_correction_enabled:=false` to use odometry fallback from the
same active-lane x for the Y approach and for the X entry.
`storage_tof_xy_tolerance_m` defaults to `0.03` m.
