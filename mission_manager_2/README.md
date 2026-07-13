# mission_manager_2

Deterministic ROS 2 mission for the 4 m x 4 m arena. It does not use the RL
policy or the old `mission_manager` node.

## Route

Arena coordinates use the bottom-left corner as `(0, 0)` and meters as units.
The pose tracker starts at `(3.8, 0.2, 90 deg)`.

1. Leave the start box with a forward-left arc.
2. Move to the bottom main road at `y=0.4*sqrt(2)+0.1=0.6657 m`.
3. Search north along `x=[3.25, 2.25, 1.25, 0.25] m`.
4. At each lane end, reverse to the main road and shift west to the next lane.
5. Split each frame into a 3x3 grid. When a sufficiently large target box has
   its center in the left-middle or right-middle cell in at least three of the
   latest five frames on the same side, rotate to center it and perform pickup.
6. After four pickup commands, return to the main road and drive west.
7. Stop at the storage-wall distance, rotate counterclockwise 45 degrees, open
   the gripper, reverse 40 cm, close the gripper, and finish.

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
| `leave_start_linear_x` | `0.08` | Start arc linear speed, m/s |
| `leave_start_angular_z` | `0.22` | Start arc left turn, rad/s |
| `leave_start_duration_s` | `3.0` | Time before in-place turns are allowed |
| `front_sensor_offset_m` | `0.15` | Robot-center to ToF sensor plane |
| `main_road_y_m` | `0.6657` | Robot-center main-road coordinate |
| `target_trigger_area_ratio` | `0.008` | Ignore very small/far YOLO boxes |
| `target_trigger_height_ratio` | `0.10` | Additional far-box rejection |
| `target_history_frames` | `5` | Number of actual YOLO frames used for voting |
| `target_required_frames` | `3` | Required votes in the same left/right cell |
| `pickup_bottom_y_ratio` | `0.70` | Start the final 10 cm pickup motion |
| `final_grab_forward_distance_m` | `0.10` | Distance travelled with gripper open |
| `storage_wall_distance_m` | `0.3828` | Direct ToF reading used near storage |

`pickup_bottom_y_ratio` uses the lower edge of the box (`y2/image_height`), not
the box center. This matches the existing detection converter and is more
stable when an object grows in the lower part of the image.

Search and target-handling speeds remain conservative. Non-search travel uses
`0.20 m/s` for main-road shifts and returns and `0.16 m/s` toward storage. Four
complete lane traversals still take about 155 seconds before target approaches,
turns, storage travel, and gripper dwell time, so a worst-case four-lane run may
still exceed three minutes.

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
check is applied to the 1 m upper-wall limit and the storage wall.

## Known limitations

- A closed-gripper command is counted as a successful pickup because no object
  presence sensor is available.
- This first version stops after depositing the first load of four. A second
  trip for a seven-object goal needs a defined route from storage back to the
  saved search lane.
- A 40 cm robot between two 8 cm objects on 50 cm centers has only 1 cm of
  clearance on each side. IMU plus command integration alone cannot guarantee
  that clearance; verify the true outer width and tune `linear_scale` at low
  speed before a competition run.
- `storage_wall_distance_m` and `front_sensor_offset_m` refer to different
  physical points. Measure both on the assembled robot instead of relying on
  the drawing alone.
