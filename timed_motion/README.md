# timed_motion

Open-loop timed motion helper for robots without encoder/odometry feedback.

It subscribes to distance and angle commands, then publishes `/cmd_vel` at a
stable rate for the calculated duration.

## Run

```bash
ros2 launch timed_motion timed_motion.launch.py
```

## Drive Distance

Forward 1 m:

```bash
ros2 topic pub -1 /drive_distance std_msgs/msg/Float32 "{data: 1.0}"
```

Backward 0.5 m:

```bash
ros2 topic pub -1 /drive_distance std_msgs/msg/Float32 "{data: -0.5}"
```

## Turn Angle

Turn one full rotation counterclockwise:

```bash
ros2 topic pub -1 /turn_angle std_msgs/msg/Float32 "{data: 6.28318}"
```

Turn one full rotation clockwise:

```bash
ros2 topic pub -1 /turn_angle std_msgs/msg/Float32 "{data: -6.28318}"
```

## Calibration

If a 1 m command actually moves 0.95 m, set:

```text
distance_scale = 1.0 / 0.95 = 1.0526
```

If a 360 degree command actually turns 330 degrees, set:

```text
angle_scale = 360.0 / 330.0 = 1.0909
```
