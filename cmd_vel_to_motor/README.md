# cmd_vel_to_motor

Converts `/cmd_vel` (`geometry_msgs/msg/Twist`) into
`/ros_robot_controller/set_motor` (`ros_robot_controller_msgs/msg/MotorsState`).

## Build

Copy this package into the Jetson ROS2 workspace:

```bash
cp -r cmd_vel_to_motor ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select cmd_vel_to_motor
source install/setup.bash
```

The workspace must also contain and build `ros_robot_controller_msgs`.

## Run

```bash
ros2 launch cmd_vel_to_motor cmd_vel_to_motor.launch.py
```

## Test

Forward:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Turn:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Stop:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

If the robot drives backward or spins the wrong way, change the motor sign
parameters in `launch/cmd_vel_to_motor.launch.py`.
