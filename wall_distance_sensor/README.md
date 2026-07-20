# wall_distance_sensor

ROS 2 package for two front-facing VL53L1X ToF sensors.

The launch starts three OS processes:

- `left_wall_tof`: reads exactly one VL53L1X on Linux I2C bus 7
- `right_wall_tof`: reads exactly one VL53L1X on Linux I2C bus 1
- `wall_distance_aggregator`: combines the two `Range` topics

Keeping each native `VL53L1X` driver in its own process avoids the driver's
global I2C callback state being shared between two Linux I2C buses.

The sensor process also declares the native `ctypes` ABI that is missing from
Pimoroni VL53L1X 0.0.5. This is required on 64-bit Jetson systems; without it,
the device pointer returned by the C library can be truncated and the process
can exit with `-11` (SIGSEGV).

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

Left sensor (exposed as Linux I2C bus 7 on the target Jetson):

```text
VCC -> Pin 1 or Pin 17, 3.3V
GND -> Pin 6, 9, 14, 20, 25, 30, 34, or 39
SDA -> Pin 3, I2C1_SDA
SCL -> Pin 5, I2C1_SCL
```

Right sensor (exposed as Linux I2C bus 1 on the target Jetson):

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
  left_address:=41 \
  right_address:=41 \
  ranging_mode:=3 \
  sensor_separation_m:=0.29 \
  safe_distance_m:=0.15
```

I2C addresses are ROS integer parameters. `41` decimal is `0x29`. Ranging
modes are `1=short`, `2=medium`, and `3=long`; long mode is the default.

Check which Linux I2C bus numbers are exposed on your board:

```bash
ls /dev/i2c-1 /dev/i2c-7
sudo i2cdetect -r -y 1
sudo i2cdetect -r -y 7
```

Each bus should show one sensor at `0x29`. If your board exposes different bus
numbers, pass those values as `left_i2c_bus` and `right_i2c_bus`.

The integrated `rl_autonomous_drive.launch.py` starts this three-process launch
automatically unless `launch_wall_distance_sensor:=false` is passed.

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

Each sensor process imports one `VL53L1X` Python driver only at runtime. On
Jetson, install a VL53L1X Python driver compatible with the board. Separate
buses allow both sensors to retain integer address `41` (`0x29`) without XSHUT.
