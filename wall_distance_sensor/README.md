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

## Jetson Nano wiring notes

VL53L1X modules normally boot at I2C address `0x29`. If you connect two modules
on the same I2C bus, use each module's XSHUT pin so the node can power them up
one at a time and assign new addresses.

Typical launch with XSHUT pins:

```bash
ros2 launch wall_distance_sensor wall_distance_angle.launch.py \
  i2c_bus:=1 \
  left_xshut_pin:=15 \
  right_xshut_pin:=16 \
  xshut_pin_mode:=BOARD \
  sensor_separation_m:=0.29 \
  safe_distance_m:=0.15
```

If the two sensors already have different addresses or are behind an I2C
multiplexer, leave the XSHUT pins disabled and set the addresses:

```bash
ros2 launch wall_distance_sensor wall_distance_angle.launch.py \
  left_address:=0x2A \
  right_address:=0x2B
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
