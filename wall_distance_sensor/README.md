# wall_distance_sensor

ROS 2 Python node for two front-facing VL53L1X ToF sensors.

It publishes the left/right range readings and a combined wall measurement:

- `/wall/left_range` (`sensor_msgs/msg/Range`)
- `/wall/right_range` (`sensor_msgs/msg/Range`)
- `/wall/distance_angle` (`geometry_msgs/msg/Vector3Stamped`)
- `/wall/measurement_json` (`std_msgs/msg/String`)

`/wall/distance_angle` uses:

- `vector.x`: perpendicular wall distance in meters
- `vector.y`: signed wall angle in radians
- `vector.z`: minimum of the two raw sensor distances in meters

Angle convention:

- `0.0` means both sensors measure the same distance, so the robot faces the wall perpendicularly.
- Positive angle means the left sensor is closer than the right sensor.
- Negative angle means the right sensor is closer than the left sensor.

## Geometry

Let:

```text
dL = left distance
dR = right distance
B  = distance between the two sensor centers
```

The node computes:

```text
wall_angle = atan2(dR - dL, B)
wall_distance = ((dL + dR) / 2) * cos(wall_angle)
```

## Jetson Orin Nano wiring

For Jetson Orin Nano, use two separate I2C buses so the two VL53L1X sensors can
both keep their default `0x29` address. This avoids SDA/SCL branching and does
not require XSHUT wiring.

Left sensor:

```text
VCC -> Pin 1 or Pin 17, 3.3V
GND -> Pin 6, 9, 14, 20, 25, 30, 34, or 39
SDA -> Pin 3, I2C1_SDA
SCL -> Pin 5, I2C1_SCL
```

Right sensor:

```text
VCC -> Pin 1 or Pin 17, 3.3V
GND -> Pin 6, 9, 14, 20, 25, 30, 34, or 39
SDA -> Pin 27, I2C0_SDA
SCL -> Pin 28, I2C0_SCL
```

Default launch for this wiring:

```bash
ros2 launch wall_distance_sensor wall_distance_angle.launch.py \
  left_i2c_bus:=7 \
  right_i2c_bus:=1 \
  left_address:=0x29 \
  right_address:=0x29 \
  sensor_separation_m:=0.29 \
  safe_distance_m:=0.15
```

Check which Linux I2C bus numbers are exposed on your board:

```bash
ls /dev/i2c-*
sudo i2cdetect -r -y 7
sudo i2cdetect -r -y 1
```

Each bus should show one sensor at `0x29`. The header labels `I2C1` and `I2C0`
do not match the Linux bus numbers on the Jetson Orin Nano: pins 3/5 normally
appear as `/dev/i2c-7`, and pins 27/28 normally appear as `/dev/i2c-1`. If your
board exposes different bus numbers, pass those values as `left_i2c_bus` and
`right_i2c_bus`.

The package can run standalone. The `mission_manager_2` launch file starts this
node automatically with the same bus defaults.

## Standalone wall-alignment test

Use this test without the mission manager, teleop, or another `/cmd_vel`
publisher. Place the robot about 0.5-1.0 m from a flat wall with enough space
for an in-place turn. If the robot-controller driver is not already running,
start it in a separate terminal:

```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.xml
```

Then launch the test. Launching it does not move the robot:

```bash
ros2 launch wall_distance_sensor wall_alignment_test.launch.py
```

In another sourced terminal, monitor the sensor and test state:

```bash
ros2 topic echo /wall/measurement_json
ros2 topic echo /wall_align/status
```

Start one alignment attempt only after both sensors report valid distances:

```bash
ros2 topic pub --once /wall_align/control std_msgs/msg/String "{data: align}"
```

Stop immediately if the robot rotates toward the wall or the absolute angle
increases:

```bash
ros2 topic pub --once /wall_align/control std_msgs/msg/String "{data: stop}"
```

Run another attempt with `reset` followed by `align`:

```bash
ros2 topic pub --once /wall_align/control std_msgs/msg/String "{data: reset}"
ros2 topic pub --once /wall_align/control std_msgs/msg/String "{data: align}"
```

The result is successful when `/wall_align/state` becomes `ALIGNED` and
`wall_angle_deg` remains within 2 degrees for five control ticks. The test
refuses to rotate when the closest sensor is under 0.35 m, the wall is over
2.0 m away, measurements are stale, or the 10-second timeout expires.

The launch file starts `cmd_vel_to_motor` by default. If that bridge is already
running, avoid a duplicate by launching with:

```bash
ros2 launch wall_distance_sensor wall_alignment_test.launch.py \
  launch_motor_bridge:=false
```

## Test without hardware

```bash
ros2 launch wall_distance_sensor wall_distance_angle.launch.py \
  driver_backend:=mock \
  mock_left_distance_m:=0.12 \
  mock_right_distance_m:=0.18
```

Then inspect:

```bash
ros2 topic echo /wall/measurement_json
```

## Runtime dependency

The node imports the `VL53L1X` Python driver only at runtime. On Jetson, install a
VL53L1X Python driver compatible with your board. If you use XSHUT pins, install
`Jetson.GPIO` as well.
