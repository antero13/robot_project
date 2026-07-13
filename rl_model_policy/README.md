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
  - `point.y`: normalized target bottom-y/closeness in `[0, 1]`
- `/avoid_objects` (`std_msgs/String`)
  - JSON from `ros2_yolo_detector`
- `/odom` (`nav_msgs/Odometry`)
  - optional arena-center pose and yaw rate for legacy 18-input checkpoints
  - ignored by the bundled 10-input model
  - also disabled by default with `pose_observation_enabled:=false`

For a new 10-input checkpoint, pose data is never sent to the network. For a
legacy 18-input checkpoint, disabling pose observation supplies zeros for its
last 8 values. Set `pose_observation_enabled:=true` only when deliberately
testing a calibrated legacy pose model.

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

Build the integrated runtime packages once after pulling the repository:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mission_manager robot_pose_tracker rl_model_policy --symlink-install
source ~/ros2_ws/install/setup.bash
```

Start the controller, camera/YOLO, motor converter, and RL policy in one
terminal. The bundled 10-input model does not start or use the pose tracker:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  launch_pose_tracker:=false \
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
environment. It is used for yaw-based target prediction only when
running a legacy 18-input model with both `launch_pose_tracker:=true` and
`pose_observation_enabled:=true`.

`target_timeout_s` defaults to `0.8`. The last target x/y is kept during a
short YOLO detection gap instead of immediately changing to search behavior.

The default real-robot configuration is explicit below:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  pose_observation_enabled:=false \
  target_timeout_s:=0.8 \
  camera_horizontal_fov_deg:=80.0
```

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
