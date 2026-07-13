# rl_model_policy

ROS 2 node that runs the trained Isaac Lab/skrl avoid/search policy.

## Checkpoint contract

Before changing this package, read:

```text
rl_model_policy/MODEL_CHECKPOINT_README.md
```

The currently pushed model at
`mission_manager/models/rl_avoid_search_best.pt` is an 18-observation-input
checkpoint:

```text
policy.net_container.0.weight: (128, 18)
state_preprocessor.running_mean: (18,)
state_preprocessor.running_variance: (18,)
```

The runner must build the same 18-value observation vector described in
`MODEL_CHECKPOINT_README.md`. A 10-value observation runner is incompatible with
the current checkpoint.

## Inputs

- `/target_object` (`geometry_msgs/PointStamped`)
  - `point.x`: normalized target x in `[-1, 1]`
  - `point.y`: normalized target bottom-y/closeness in `[0, 1]`
- `/avoid_objects` (`std_msgs/String`)
  - JSON from `ros2_yolo_detector`
- `/odom` (`nav_msgs/Odometry`)
  - arena-center pose and calibrated/fallback yaw rate from `robot_pose_tracker`
  - stale or missing odometry sets `pose_valid=0` and zeroes observation 11-17
  - positions outside the 4 m arena plus `pose_bounds_tolerance_m` are also invalid

## Output

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/ros_robot_controller/bus_servo/set_state` for the default bus-servo gripper

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

Build the policy, pose tracker, and model package once after pulling the repository:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mission_manager robot_pose_tracker rl_model_policy --symlink-install
source ~/ros2_ws/install/setup.bash
```

Start the controller, camera/YOLO, motor converter, pose tracker, and RL policy
in one terminal. The default start pose is `(1.8, -1.8, 90 deg)` in the
arena-center coordinate frame used by training:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  initial_x:=1.8 \
  initial_y:=-1.8 \
  initial_yaw_deg:=90.0
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

`camera_horizontal_fov_deg` defaults to `90.0` and is used to convert target
image x into the last target world bearing. Replace it with the calibrated
horizontal field of view of the driving camera when that value is available.

## Automatic pickup

The policy keeps the gripper closed when a run starts. When the target is centered and
its bottom edge reaches `grab_area_ratio`, the policy temporarily pauses RL control and
runs this sequence:

```text
TRACKING -> OPENING -> FINAL_FORWARD -> CLOSING -> GRABBED
```

Default bus-servo and pickup settings:

```text
servo ID: 1
open position: 1000
closed position: 250
grab center tolerance: 0.12
grab area ratio: 0.50
final forward: 0.06 m/s for 1.6 s
stop after grab: true
```

Override values from the integrated launch when needed:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  gripper_open_position:=1000 \
  gripper_closed_position:=250 \
  final_forward_duration_s:=1.6
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
