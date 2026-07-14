# bbox_zone_controller

This package drives the robot from only the largest YOLO bounding box in each
current frame. It does not use the RL policy, object tracking, odometry, or the
existing mission manager.

## Zone geometry

Bounding-box centers use normalized image coordinates:

```text
x = -1 at the left edge, 0 at the image center, 1 at the right edge
y =  0 at the top edge, 1 at the bottom edge
```

The left boundary is the line through points 1 and 2. The right boundary is the
line through points 3 and 4. Both segments are extended as lines across the
frame, and `x=0` is the center boundary.

```text
point 1 = (-0.8600, 0.9900)
point 2 = (-0.7600, 0.8333)
point 3 = ( 0.7375, 0.9933)
point 4 = ( 0.5825, 0.7367)
```

For a non-target largest box:

| Zone | Command |
| --- | --- |
| outer left | straight |
| inner left | forward-right |
| inner right | forward-left |
| outer right | straight |

For a target largest box, the robot rotates in place until the bbox center is
within `target_center_tolerance` of `x=0`, then drives straight toward it.

## Build and run

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to bbox_zone_controller
source install/setup.bash
ros2 launch bbox_zone_controller bbox_zone_drive.launch.py
```

Movement is disabled at startup. Inspect the camera and status before starting:

```bash
ros2 topic echo /bbox_zone_controller/status
ros2 topic pub --once /bbox_zone_controller/control std_msgs/msg/String "{data: start}"
```

Stop immediately with:

```bash
ros2 topic pub --once /bbox_zone_controller/control std_msgs/msg/String "{data: stop}"
```

For a motion-free test:

```bash
ros2 launch bbox_zone_controller bbox_zone_drive.launch.py dry_run:=true
```

The integrated launch starts only the robot controller, camera 1 with YOLO,
`cmd_vel_to_motor`, and this node. Set any `launch_*` argument to `false` when
that component is already running.

## Runtime parameters

Target classes can use class IDs, class names, or both:

```bash
ros2 launch bbox_zone_controller bbox_zone_drive.launch.py \
  target_classes:=apple,banana,orange,pineapple
```

Speeds can be changed without editing code:

```bash
ros2 launch bbox_zone_controller bbox_zone_drive.launch.py \
  straight_linear_x:=0.12 \
  avoid_turn_linear_x:=0.07 \
  avoid_turn_angular_z:=0.50 \
  target_forward_linear_x:=0.09
```

The four zone points, target centering tolerance, alignment gain, minimum turn
speed, and maximum turn speed are also launch arguments. `active_on_start` is
`false` by default; enable it only after dry-run validation.
