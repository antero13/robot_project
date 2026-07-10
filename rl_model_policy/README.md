# rl_model_policy

ROS 2 node that runs the trained Isaac Lab/skrl avoid/search policy.

## Inputs

- `/target_object` (`geometry_msgs/PointStamped`)
  - `point.x`: normalized target x in `[-1, 1]`
  - `point.y`: normalized target bottom-y/closeness in `[0, 1]`
- `/avoid_objects` (`std_msgs/String`)
  - JSON from `ros2_yolo_detector`

## Output

- `/cmd_vel` (`geometry_msgs/Twist`)

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

Build the policy and install its model file once after pulling the repository:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mission_manager rl_model_policy --symlink-install
source ~/ros2_ws/install/setup.bash
```

Start the controller, camera/YOLO, motor converter, and RL policy in one terminal:

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25
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
