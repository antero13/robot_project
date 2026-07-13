# RL checkpoint contract

This document is for Codex or a developer who needs to read
`mission_manager/models/rl_avoid_search_best.pt` and update the ROS files that
run the policy.

## Current checkpoint

Model file:

```text
mission_manager/models/rl_avoid_search_best.pt
```

The currently pushed checkpoint was copied from Isaac Lab training output:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\logs\skrl\robot_avoid_search_2d\2026-07-10_16-57-34_ppo_torch\checkpoints\best_agent.pt
```

Important: this checkpoint is an **18 observation input** model. Any ROS runner
that still builds a 10-value observation or uses `torch.nn.Linear(10, 128)` will
not match this checkpoint.

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
obs_dim = 18
action_dim = 2

net_container:
  Linear(18, 128)
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
policy.net_container.0.weight   (128, 18)
policy.net_container.0.bias     (128,)
policy.net_container.2.weight   (128, 128)
policy.net_container.2.bias     (128,)
policy.net_container.4.weight   (64, 128)
policy.net_container.4.bias     (64,)
policy.policy_layer.weight      (2, 64)
policy.policy_layer.bias        (2,)
policy.value_layer.weight       (1, 64)
policy.value_layer.bias         (1,)

state_preprocessor.running_mean      (18,)
state_preprocessor.running_variance  (18,)
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

The 18 input values must be built in exactly this order:

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

For the current 18-input checkpoint, this file must satisfy:

```text
PolicyNetwork.net_container.0 = torch.nn.Linear(18, 128)
make_observation(...) returns exactly 18 floats
obs_mean and obs_variance loaded from checkpoint['state_preprocessor'] are length 18
```

The ROS runner now implements this 18-value contract and rejects checkpoints
whose first policy layer or state preprocessor has a different shape.

Launch files that may need parameters for pose input:

```text
rl_model_policy/launch/rl_model_policy.launch.py
rl_model_policy/launch/rl_autonomous_drive.launch.py
```

Related pose package:

```text
robot_pose_tracker/
```

The ROS runner subscribes to `/odom` from `robot_pose_tracker`. This supplies
pose and the tracker-selected yaw rate for observation indices 10 through 17.
If odometry is stale or unavailable, it sets `pose_valid = 0.0` and zeroes all
eight pose inputs; the policy was trained with pose dropout, but it performs
best with usable pose/IMU data.

## Isaac Lab source of truth

The training observation is defined in:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\source\robot_avoid_search\robot_avoid_search\tasks\direct\robot_avoid_search\robot_avoid_search_env.py
```

Look for:

```text
_get_observations()
_pose_observation()
```

The training constants are defined in:

```text
C:\Users\user\Documents\robot_avoid_search\robot_avoid_search\source\robot_avoid_search\robot_avoid_search\tasks\direct\robot_avoid_search\robot_avoid_search_env_cfg.py
```

Look for:

```text
observation_space = 18
max_forward_speed
max_reverse_speed
max_angular_speed
pose_xy_noise_std_m
pose_yaw_noise_std_rad
pose_dropout_prob
```

## Quick compatibility checklist

Before running the robot:

```text
1. torch.load(model)['policy']['net_container.0.weight'].shape == (128, 18)
2. PolicyNetwork first layer is Linear(18, 128)
3. make_observation returns 18 values
4. state_preprocessor running_mean/running_variance are applied before inference
5. action uses policy mean, not Gaussian sampling
6. /cmd_vel is published by only one node
7. Test with dry_run:=true, then speed_scale:=0.25
```
