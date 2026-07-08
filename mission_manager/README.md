# mission_manager

First mission state-machine package for the AI Robot Challenge robot.

This node publishes `/cmd_vel` from a mission sequence. It follows
`/target_object` detections, aligns before approaching, and can temporarily
avoid nearer objects reported on `/avoid_object`.

## Topics

Subscribed:

- `/mission_control` (`std_msgs/msg/String`)
- `/target_object` (`geometry_msgs/msg/PointStamped`)
- `/avoid_object` (`geometry_msgs/msg/PointStamped`)

Published:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/mission_state` (`std_msgs/msg/String`)
- `/ros_robot_controller/pwm_servo/set_state` (`ros_robot_controller_msgs/msg/SetPWMServoState`)

## Commands

```bash
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'start'}"
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'search'}"
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'reset'}"
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'open'}"
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'close'}"
```

`start` runs this first demo mission:

```text
LEAVE_START -> SEARCH -> ALIGN_TARGET -> APPROACH_TARGET -> FINAL_FORWARD -> GRAB_OBJECT -> BACK_OUT -> DONE
```

If a close obstacle is detected, the manager inserts:

```text
AVOID_TURN -> AVOID_FORWARD -> REACQUIRE_TARGET
```

`/target_object` format:

```text
point.x = normalized horizontal error, -1.0 left to +1.0 right
point.y = normalized bounding-box bottom y, 0.0 top to 1.0 bottom
point.z = detection confidence
```

`/avoid_object` uses the same point format. During `ALIGN_TARGET`, the robot
turns until `point.x` is near 0. During `APPROACH_TARGET`, it moves forward
while centered and transitions to `FINAL_FORWARD` when `point.y` reaches
`grab_area_ratio`. Despite the legacy parameter name, this threshold is now a
camera y-position closeness score. `FINAL_FORWARD` drives straight briefly so
the object reaches the gripper before `GRAB_OBJECT` closes the servo.

## Build

Copy these packages into the Jetson workspace:

```bash
cp -r ros2_yolo_detector ~/ros2_ws/src/
cp -r cmd_vel_to_motor ~/ros2_ws/src/
cp -r mission_manager ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros2_yolo_detector cmd_vel_to_motor mission_manager
source install/setup.bash
```

The workspace must already contain `ros_robot_controller_msgs`.

## Run

Use three terminals at first:

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
ros2 launch mission_manager mission_manager.launch.py
```

Or run the camera detector, target conversion, mission manager, and motor
bridge together:

```bash
ros2 launch mission_manager object_pickup_mission.launch.py target_classes:=apple avoid_classes:=person
```

Start the demo:

```bash
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'start'}"
```

Emergency stop:

```bash
ros2 topic pub -1 /mission_control std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Watch state

```bash
ros2 topic echo /mission_state
```

## Tuning

Tune these values in `launch/mission_manager.launch.py` or
`launch/camera_mission.launch.py`:

```text
center_tolerance: how centered the target must be before moving forward
grab_area_ratio: how low the box bottom must be before grabbing, 0.0 top to 1.0 bottom
final_forward_linear_x: straight driving speed after visual approach
final_forward_duration_s: straight driving time before closing the gripper
approach_angular_gain: how strongly the robot turns toward the target
approach_max_linear_x: maximum approach speed
avoid_area_ratio: minimum obstacle box-bottom y before avoidance can trigger
avoid_center_band: how close to the camera center an obstacle must be
avoid_closer_ratio: how much lower the obstacle must appear than the target
```
