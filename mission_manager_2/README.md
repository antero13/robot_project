# mission_manager_2

Deterministic ROS 2 mission for the 4 m x 4 m arena. It does not use the RL
policy or the old `mission_manager` node.

## Route

Arena coordinates use the bottom-left corner as `(0, 0)` and meters as units.
The pose tracker starts on the main road at `(3.25, 0.6657, 90 deg)`, facing
north at the first search-lane entrance.

1. Orient north using the IMU heading.
2. Search north along `x=[3.25, 2.25, 1.25, 0.25] m`.
3. At each lane end, reverse to the main road, turn west using IMU heading,
   and use odometry to reach the next lane.
4. Split each frame into a 3x3 grid. When a sufficiently large target box has
   its center in any middle-row cell or either lower corner in at least three
   of the latest five frames, rotate to center it and perform pickup. Votes may
   come from different allowed cells.

```text
not used | not used | not used
allowed  | allowed  | allowed
allowed  | not used | allowed
```
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

On the Jetson Orin Nano, the default sensor mapping is bus 7 for the left
sensor on header pins 3/5 and bus 1 for the right sensor on pins 27/28. Both
sensors retain the default `0x29` address because they use separate buses.

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
| `wall_correction_enabled` | `false` | Disable ToF route correction |
| `front_sensor_offset_m` | `0.15` | Robot-center to ToF sensor plane |
| `main_road_y_m` | `0.6657` | Robot-center main-road coordinate |
| `yaw_tolerance_deg` | `1.0` | IMU turn-completion tolerance |
| `position_tolerance_m` | `0.01` | Route arrival tolerance (1 cm) |
| `target_classes` | `0,4` | Collectible YOLO class IDs |
| `target_trigger_area_ratio` | `0.008` | Ignore very small/far YOLO boxes |
| `target_trigger_height_ratio` | `0.10` | Additional far-box rejection |
| `target_history_frames` | `5` | Number of actual YOLO frames used for voting |
| `target_required_frames` | `3` | Required votes across all allowed cells |
| `pickup_bottom_y_ratio` | `0.70` | Start the final 20 cm pickup motion |
| `final_grab_forward_distance_m` | `0.20` | Distance travelled with gripper open |
| `gripper_open_position` | `1000` | Servo position used for an open gripper |
| `gripper_closed_position` | `300` | Servo position used for a closed gripper |

`pickup_bottom_y_ratio` uses the lower edge of the box (`y2/image_height`), not
the box center. This matches the existing detection converter and is more
stable when an object grows in the lower part of the image.

Search and target-handling speeds remain conservative. Main-road shifts and
returns use `0.20 m/s`.

The default `target_classes=0,4` collects only YOLO class IDs 0 and 4. Set a
different comma-separated allow-list when the model labels change. An empty
value treats every class as a pickup target.

## Runtime target and speed tuning

The target allow-list and motion speeds can be changed while the node is
running. A rebuild and restart are not required:

```bash
ros2 param set /mission_manager_2 target_classes "apple,banana,orange,pineapple"
ros2 param set /mission_manager_2 search_linear_x 0.08
ros2 param set /mission_manager_2 navigation_linear_x 0.16
ros2 param set /mission_manager_2 return_linear_x -0.16
ros2 param set /mission_manager_2 target_return_linear_x -0.10
ros2 param set /mission_manager_2 target_approach_max_linear_x 0.06
ros2 param set /mission_manager_2 final_grab_forward_linear_x 0.04
```

Use an empty string to make every detected class collectible:

```bash
ros2 param set /mission_manager_2 target_classes "''"
```

Forward and angular speed parameters must be positive. `return_linear_x` and
`target_return_linear_x` must remain negative. Minimum speeds cannot exceed
their corresponding maximum. The current values are reported under
`runtime_settings` in `/mission2/status` and can also be queried directly:

```bash
ros2 param get /mission_manager_2 target_classes
ros2 param get /mission_manager_2 search_linear_x
ros2 topic echo /mission2/status
```

Runtime changes last until the node stops. For persistent changes without a
rebuild, copy `config/mission_manager_2.yaml` outside the workspace, edit it,
and pass its absolute path with `mission_config:=...` when launching.

## Sensor policy

Route correction uses IMU heading and command-integrated odometry only.
`wall_correction_enabled` and `launch_wall_sensor` are both `false` by default.
Lane and main-road wall-alignment states therefore pass immediately, main-road
distance and steering use odometry, and the upper search limit uses pose `y`.

The ToF implementation remains available for isolated sensor testing. Enabling
route correction requires both `wall_correction_enabled: true` in the mission
configuration and `launch_wall_sensor:=true` on the launch command.

## Known limitations

- A closed-gripper command is counted as a successful pickup because no object
  presence sensor is available.
- Storage navigation and unloading are not implemented. The mission stops on
  the main road after the fourth pickup attempt.
- A 40 cm robot between two 8 cm objects on 50 cm centers has only 1 cm of
  clearance on each side. IMU plus command integration alone cannot guarantee
  that clearance; verify the true outer width and tune `linear_scale` at low
  speed before a competition run.
