# RL checkpoint contract

This document is for Codex or a developer who needs to read
`mission_manager/models/rl_avoid_search_best.pt` and update the ROS files that
run the policy.

## Current checkpoint

Model file:

```text
mission_manager/models/rl_avoid_search_best.pt
```

The bundled checkpoint was copied from this Isaac Lab training run:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\logs\skrl\robot_avoid_search_2d\2026-07-13_14-22-55_ppo_torch\checkpoints\best_agent.pt
```

Important: this checkpoint is a **10 observation input** model. It receives
only the YOLO-derived values at indices 0 through 9. Pose and IMU values are not
part of this policy input.

## How to inspect the checkpoint

Run this from the ROS repository root:

```powershell
python -c "import torch; p=r'mission_manager\models\rl_avoid_search_best.pt'; ckpt=torch.load(p,map_location='cpu'); print(ckpt.keys()); [print(k,'dict',len(v),list(v.keys())[:10]) for k,v in ckpt.items() if isinstance(v,dict)]; [print('policy',k,tuple(v.shape)) for k,v in ckpt['policy'].items() if hasattr(v,'shape')]; [print('state_preprocessor',k,tuple(v.shape) if hasattr(v,'shape') else v) for k,v in ckpt['state_preprocessor'].items()]"
```

Expected top-level keys:

```text
policy
value
optimizer
state_preprocessor
value_preprocessor
```

## Network architecture

The policy is a skrl Gaussian policy. For real robot inference, use the policy
mean action. Do not sample random actions on the robot.

Architecture:

```text
obs_dim = 10
action_dim = 2

net_container:
  Linear(10, 128)
  ELU
  Linear(128, 128)
  ELU
  Linear(128, 64)
  ELU

policy_layer:
  Linear(64, 2)

value_layer:
  Linear(64, 1)

log_std_parameter:
  shape (2,)
```

Checkpoint tensor shapes:

```text
policy.log_std_parameter        (2,)
policy.net_container.0.weight   (128, 10)
policy.net_container.0.bias     (128,)
policy.net_container.2.weight   (128, 128)
policy.net_container.2.bias     (128,)
policy.net_container.4.weight   (64, 128)
policy.net_container.4.bias     (64,)
policy.policy_layer.weight      (2, 64)
policy.policy_layer.bias        (2,)
policy.value_layer.weight       (1, 64)
policy.value_layer.bias         (1,)

state_preprocessor.running_mean      (10,)
state_preprocessor.running_variance  (10,)
state_preprocessor.current_count     scalar
```

For inference:

```python
obs_scaled = (obs - running_mean) / sqrt(running_variance + epsilon)
action = policy(obs_scaled)
action = clamp(action, -1.0, 1.0)
```

Use `epsilon = 1e-8` unless the launch parameter overrides it.

## Observation order

The current model's 10 input values must be built in exactly this order:

```text
0  target_visible
1  target_x
2  target_y
3  time_since_target_seen_norm
4  last_target_direction
5  avoid_left
6  avoid_center
7  avoid_right
8  nearest_avoid_x
9  nearest_avoid_y
```

Legacy 18-input checkpoints append these values after index 9:

```text
10 pose_valid
11 robot_x_norm
12 robot_y_norm
13 sin(yaw)
14 cos(yaw)
15 imu_yaw_rate_norm
16 last_target_bearing_sin
17 last_target_bearing_cos
```

Meaning:

```text
target_visible:
  1.0 if target_object is fresh, else 0.0

target_x:
  normalized image x in [-1, 1], 0 when target is not visible

target_y:
  normalized image y/closeness in [0, 1], 0 when target is not visible

time_since_target_seen_norm:
  time_since_target_seen / episode_length_s, clamped to [0, 1]

last_target_direction:
  -1.0 if the last visible target was left of center,
   1.0 if the last visible target was right of center

avoid_left, avoid_center, avoid_right:
  obstacle danger bins from YOLO avoid objects, each in [0, 1]

nearest_avoid_x, nearest_avoid_y:
  x/y of the closest avoid object in normalized camera coordinates,
  or 0.0/0.0 when no avoid object is fresh

Legacy-only pose values:

pose_valid:
  1.0 when pose estimate is fresh and usable, else 0.0

robot_x_norm:
  robot arena x / 2.0, clamped to [-1, 1], 0.0 if pose is invalid

robot_y_norm:
  robot arena y / 2.0, clamped to [-1, 1], 0.0 if pose is invalid

sin(yaw), cos(yaw):
  robot heading from pose/IMU, 0.0 if pose is invalid

imu_yaw_rate_norm:
  yaw_rate / max_angular_speed, clamped to [-1, 1]

last_target_bearing_sin, last_target_bearing_cos:
  relative bearing from the current robot heading to the last known target
  world direction: last_target_world_bearing - current_yaw.
  If exact world target bearing is unavailable, compute an approximate value
  from the latest target_x and current yaw, or keep the previous bearing.
```

## Action order

The policy output has two values:

```text
action[0] = linear command in [-1, 1]
action[1] = angular command in [-1, 1]
```

The ROS runner converts it as:

```python
if action[0] >= 0:
    linear_x = action[0] * max_forward_speed
else:
    linear_x = action[0] * max_reverse_speed

angular_z = action[1] * max_angular_speed

linear_x *= speed_scale
angular_z *= speed_scale
```

Default values from training:

```text
max_forward_speed = 0.20
max_reverse_speed = 0.05
max_angular_speed = 0.80
action_filter_alpha = 0.60
max_linear_action_delta = 0.25
max_angular_action_delta = 0.16
episode_length_s = 18.0
```

## ROS files to update when checkpoint shape changes

Main runner:

```text
rl_model_policy/rl_model_policy/rl_model_policy_node.py
```

For the current 10-input checkpoint, this file must satisfy:

```text
PolicyNetwork.net_container.0 = torch.nn.Linear(10, 128)
the first 10 values from make_observation(...) are sent to the policy
obs_mean and obs_variance loaded from checkpoint['state_preprocessor'] are length 10
```

The ROS runner detects 10- and 18-input checkpoints, builds the matching first
layer, and rejects any other policy or state-preprocessor shape.

Launch files that may need parameters for pose input:

```text
rl_model_policy/launch/rl_model_policy.launch.py
rl_model_policy/launch/rl_autonomous_drive.launch.py
```

Related pose package:

```text
robot_pose_tracker/
```

The current 10-input model does not consume `/odom`. For a legacy 18-input
checkpoint, the ROS runner can subscribe to `/odom` from `robot_pose_tracker`
and use pose/yaw-rate values at observation indices 10 through 17. With pose
observation disabled, those eight legacy inputs are zero.

## Isaac Lab source of truth

The training observation is defined in:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\source\robot_avoid_search\robot_avoid_search\tasks\direct\robot_avoid_search\robot_avoid_search_env.py
```

Look for:

```text
_get_observations()
```

The training constants are defined in:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\source\robot_avoid_search\robot_avoid_search\tasks\direct\robot_avoid_search\robot_avoid_search_env_cfg.py
```

Look for:

```text
observation_space = 10
max_forward_speed
max_reverse_speed
max_angular_speed
front_capture_width_m = 0.209522
front_penalty_extra_each_side_m = 0.010
```

## Quick compatibility checklist

Before running the robot:

```text
1. torch.load(model)['policy']['net_container.0.weight'].shape == (128, 10)
2. the runner detects observation_dim == 10
3. only observation values 0 through 9 are sent to the current policy
4. state_preprocessor running_mean/running_variance are applied before inference
5. action uses policy mean, not Gaussian sampling
6. /cmd_vel is published by only one node
7. Test with dry_run:=true, then speed_scale:=0.25
```
