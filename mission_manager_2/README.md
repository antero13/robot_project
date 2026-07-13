# mission_manager_2

Deterministic ROS 2 mission for the 4 m x 4 m arena. It does not use the RL
policy or the old `mission_manager` node.

## Route

Arena coordinates use the bottom-left corner as `(0, 0)` and meters as units.
The pose tracker starts on the main road at `(3.25, 0.6657, 90 deg)`, facing
north at the first search-lane entrance.

1. Check the north heading with IMU and a plausible upper-wall ToF reading.
2. Search north along `x=[3.25, 2.25, 1.25, 0.25] m`.
3. At each lane end, reverse to the main road and shift west to the next lane.
4. Split each frame into a 3x3 grid. When a sufficiently large target box has
   its center in the left-middle or right-middle cell in at least three of the
   latest five frames on the same side, rotate to center it and perform pickup.
5. After four pickup commands, return to the main road and stop. The storage
   route is intentionally deferred.

The state machine consumes raw per-frame `/yolo/detections`. It does not use
tracking IDs or require objects on both sides of a lane.

## Build and run

```bash
cd ~/robot_project
colcon build --symlink-install \
  --packages-select mission_manager_2 robot_pose_tracker wall_distance_sensor \
  ros2_yolo_detector cmd_vel_to_motor
source install/setup.bash
ros2 launch mission_manager_2 mission_manager_2.launch.py
```

Keep the robot still for the pose tracker's first two seconds of gyro
calibration. Then start the mission:

```bash
ros2 topic pub --once /mission2/control std_msgs/msg/String "{data: start}"
```

Stop and reset:

```bash
ros2 topic pub --once /mission2/control std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /mission2/control std_msgs/msg/String "{data: reset}"
```

`reset` assumes the robot has physically been returned to the start pose. It
also requests `/robot_pose/reset`.

Monitor the state and detailed JSON status:

```bash
ros2 topic echo /mission2/state
ros2 topic echo /mission2/status
```

Do not run the old mission manager, RL autonomous-drive node, keyboard teleop,
or another `/cmd_vel` publisher at the same time.

## Parameters to calibrate first

Edit `config/mission_manager_2.yaml` before a full-speed run.

| Parameter | Initial value | Purpose |
| --- | ---: | --- |
| `front_sensor_offset_m` | `0.15` | Robot-center to ToF sensor plane |
| `main_road_y_m` | `0.6657` | Robot-center main-road coordinate |
| `target_trigger_area_ratio` | `0.008` | Ignore very small/far YOLO boxes |
| `target_trigger_height_ratio` | `0.10` | Additional far-box rejection |
| `target_history_frames` | `5` | Number of actual YOLO frames used for voting |
| `target_required_frames` | `3` | Required votes in the same left/right cell |
| `pickup_bottom_y_ratio` | `0.70` | Start the final 10 cm pickup motion |
| `final_grab_forward_distance_m` | `0.10` | Distance travelled with gripper open |

`pickup_bottom_y_ratio` uses the lower edge of the box (`y2/image_height`), not
the box center. This matches the existing detection converter and is more
stable when an object grows in the lower part of the image.

Search and target-handling speeds remain conservative. Main-road shifts and
returns use `0.20 m/s`.

Set `target_classes` to a comma-separated allow-list if the YOLO model also
detects objects that must not be picked. An empty value treats every class as a
pickup target.

## Sensor policy

Odometry and IMU provide the route pose. A ToF reading is treated as a wall only
when it agrees with the wall distance predicted from odometry within
`wall_consistency_tolerance_m`. This prevents an object in front of one sensor
from being mistaken for the arena wall.

At a lane entrance, the manager waits up to two seconds for a plausible upper
wall measurement and aligns both front ranges. If the wall is out of range or
occluded, it keeps the IMU heading and starts the lane. The same consistency
check is applied to the 1 m upper-wall limit.

## Known limitations

- A closed-gripper command is counted as a successful pickup because no object
  presence sensor is available.
- Storage navigation and unloading are not implemented. The mission stops on
  the main road after the fourth pickup attempt.
- A 40 cm robot between two 8 cm objects on 50 cm centers has only 1 cm of
  clearance on each side. IMU plus command integration alone cannot guarantee
  that clearance; verify the true outer width and tune `linear_scale` at low
  speed before a competition run.
