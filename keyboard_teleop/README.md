# keyboard_teleop

SSH terminal keyboard controller for the robot.

It publishes `/cmd_vel` (`geometry_msgs/msg/Twist`) from WASD or arrow key
input. It can also publish bus servo commands to
`/ros_robot_controller/bus_servo/set_state`. Use it together with
`cmd_vel_to_motor` and `ros_robot_controller`.

## Build

Copy this package into the Jetson workspace:

```bash
cp -r keyboard_teleop ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select keyboard_teleop
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
ros2 run keyboard_teleop keyboard_teleop
```

## Keys

```text
w / up arrow       forward
s / down arrow     backward
a / left arrow     turn left
d / right arrow    turn right
q                  forward + left
e                  forward + right
z                  backward + left
c                  backward + right

i / k              increase/decrease linear speed
o / l              increase/decrease angular speed

r / f              increase/decrease bus servo position
t                  bus servo center position
y                  bus servo minimum position
u                  bus servo maximum position
v                  stop bus servo

space or x         stop
Ctrl+C             quit
```

The default `key_timeout_s` is `0.0`, so movement continues until another
movement key, space, x, or Ctrl+C is pressed. Set `key_timeout_s` to a positive
value if you want dead-man-switch behavior.

## Bus servo settings

Edit `launch/keyboard_teleop.launch.py` if the servo ID or range is different.

```python
'bus_servo_id': 1,
'bus_servo_min_position': 0,
'bus_servo_max_position': 1000,
'bus_servo_center_position': 500,
'bus_servo_step': 50,
'bus_servo_duration_s': 0.4,
```

You can test the bus servo directly without the keyboard node:

```bash
ros2 topic pub -1 /ros_robot_controller/bus_servo/set_state ros_robot_controller_msgs/msg/SetBusServoState "{duration: 0.5, state: [{present_id: [1, 1], position: [1, 500]}]}"
```
