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
