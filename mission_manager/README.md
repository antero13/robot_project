# mission_manager

First mission state-machine package for the AI Robot Challenge robot.

This node publishes `/cmd_vel` from a mission sequence. It follows
`/target_object` detections, aligns before approaching, and can temporarily
avoid nearer objects reported on `/avoid_object` or the multi-object
`/avoid_objects` VFH-lite input.

## Topics

Subscribed:

- `/mission_control` (`std_msgs/msg/String`)
- `/target_object` (`geometry_msgs/msg/PointStamped`)
- `/avoid_object` (`geometry_msgs/msg/PointStamped`)
- `/avoid_objects` (`std_msgs/msg/String`, JSON list of avoid objects)

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
LEAVE_START -> SEARCH -> ALIGN_TARGET -> APPROACH_TARGET -> OPEN_GRIPPER -> FINAL_FORWARD -> GRAB_OBJECT -> BACK_OUT -> DONE
```

At `start`, the gripper is commanded closed and stays closed while the robot
searches, aligns, and approaches the target.

If close obstacles are detected, the manager builds a small 5-bin horizontal
histogram from `/avoid_objects`, compares the left/right danger cost, then
inserts:

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
while centered and transitions to `OPEN_GRIPPER` when `point.y` reaches
`grab_area_ratio`. Despite the legacy parameter name, this threshold is now a
camera y-position closeness score. `OPEN_GRIPPER` opens the servo, then
`FINAL_FORWARD` drives straight briefly so the object enters the gripper before
`GRAB_OBJECT` closes the servo.

`/avoid_objects` is preferred when available. It is a JSON message:

```json
{
  "objects": [
    {"class_name": "person", "x": 0.25, "y": 0.62, "confidence": 0.88}
  ]
}
```

The manager treats `y` as closeness, squares it into a danger score, adds the
danger into five screen sectors, and chooses the lower-cost side. The previous
`/avoid_object` single-point topic still works as a fallback.

When the target is close and centered, avoid detections at nearly the same
screen position are treated as duplicate detections of the target and ignored.
This prevents the robot from suddenly avoiding the object it is about to grab.

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
grab_area_ratio: how low the box bottom must be before grabbing, 0.0 top to 1.0 bottom, default 0.50
final_forward_linear_x: straight driving speed after visual approach
final_forward_duration_s: straight driving time before closing the gripper
approach_angular_gain: how strongly the robot turns toward the target
approach_max_linear_x: maximum approach speed
avoid_area_ratio: obstacle box-bottom y where avoidance can trigger, default 0.45
avoid_center_band: widest horizontal band where obstacles can count, default 0.75
avoid_center_corridor: center gripper path that always counts for normal avoidance, default 0.30
avoid_path_margin: extra width around the current target path where obstacles count, default 0.30
avoid_emergency_ratio: y threshold that can trigger avoidance even if the target is also close, default 0.75
avoid_closer_ratio: how much lower the obstacle must appear than the target, default 1.00
avoid_turn_duration_s: first turn-only avoidance duration, default 0.45
avoid_forward_duration_s: curved forward avoidance duration, default 0.75
avoid_vfh_center_weight: extra danger for obstacles near the gripper center line, default 1.5
avoid_vfh_target_weight: small bias toward the target side when both avoid sides are similar, default 0.60
avoid_vfh_switch_penalty: penalty for rapidly switching avoid direction, default 0.25
avoid_direction_hold_s: seconds to prefer the previous avoid direction, default 0.8
avoid_ignore_near_target_enabled: ignore avoid detections that look like the close locked target, default true
avoid_ignore_target_min_y: target y where duplicate-avoid ignoring can start, default 0.35
avoid_ignore_target_center_band: target must be this close to center before duplicate-avoid ignoring, default 0.25
avoid_ignore_target_x_margin: max x gap between close target and duplicate avoid, default 0.25
avoid_ignore_target_y_margin: max y gap between close target and duplicate avoid, default 0.20
```

Lower `avoid_area_ratio` to avoid earlier. Raise `avoid_center_band`,
`avoid_center_corridor`, or `avoid_path_margin` to give the gripper more side
clearance. Lower them if the robot avoids objects that are not on the way to
the target. Raise `avoid_vfh_switch_penalty` or `avoid_direction_hold_s` if the
robot oscillates between left and right.
