# target_centering

Centers a detected camera target by publishing `/cmd_vel` angular commands.
The controller is PD:

```text
angular.z = -(angular_kp * error + angular_kd * d_error/dt)
```

It assumes another node, such as `yolo_target_detector`, publishes:

```text
/target_object geometry_msgs/msg/PointStamped
point.x = normalized horizontal error, -1.0 left to +1.0 right
point.y = normalized bounding-box bottom y, 0.0 top to 1.0 bottom
point.z = confidence

/avoid_object geometry_msgs/msg/PointStamped
same format, but for an unwanted object
```

This node only rotates the robot. It does not drive forward. If an avoid object
is close enough and near the camera center, avoidance has priority over target
centering.

## Control parameters

Edit `launch/target_centering.launch.py`:

```text
center_tolerance: error range where the robot stops
angular_kp: proportional gain; larger means stronger turning
angular_kd: derivative gain; larger means more damping
max_angular_z: maximum turn speed
min_angular_z: minimum turn speed outside the tolerance
target_timeout_s: target becomes stale if it is not updated within this time
filter_window_size: average this many recent detections
min_consecutive_detections: wait for this many detections before moving
lost_hold_s: after target_timeout_s, briefly keep the last command when detections flicker
avoid_enabled: turn avoid behavior on/off
avoid_area_ratio: avoid only if unwanted object's box bottom is at least this low
avoid_center_band: avoid only if unwanted object is near the center
avoid_angular_z: turn speed used while avoiding
avoid_only_if_closer_than_target: skip avoidance if the target appears closer
avoid_closer_ratio: avoid must appear this much lower than target to count as closer
```

## Build

```bash
cp -r target_centering ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select target_centering
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.xml
```

Terminal 2:

```bash
ros2 launch cmd_vel_to_motor cmd_vel_to_motor.launch.py
```

Terminal 3:

```bash
ros2 launch target_centering target_centering.launch.py
```

## Fake target tests

Pretend the object is on the right side of the image:

```bash
ros2 topic pub -1 /target_object geometry_msgs/msg/PointStamped "{point: {x: 0.5, y: 0.70, z: 0.9}}"
```

Pretend the object is on the left side:

```bash
ros2 topic pub -1 /target_object geometry_msgs/msg/PointStamped "{point: {x: -0.5, y: 0.70, z: 0.9}}"
```

Pretend the object is centered:

```bash
ros2 topic pub -1 /target_object geometry_msgs/msg/PointStamped "{point: {x: 0.0, y: 0.70, z: 0.9}}"
```

Watch the generated command:

```bash
ros2 topic echo /cmd_vel
```

## Fake avoid tests

Pretend an unwanted object is on the right side:

```bash
ros2 topic pub -1 /avoid_object geometry_msgs/msg/PointStamped "{point: {x: 0.3, y: 0.75, z: 0.9}}"
```

Pretend an unwanted object is on the left side:

```bash
ros2 topic pub -1 /avoid_object geometry_msgs/msg/PointStamped "{point: {x: -0.3, y: 0.75, z: 0.9}}"
```
