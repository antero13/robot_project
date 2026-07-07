import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, SetBusServoState


HELP_TEXT = """
Keyboard teleop
----------------
Move:
  w / up arrow       forward
  s / down arrow     backward
  a / left arrow     turn left
  d / right arrow    turn right
  q                  forward + left
  e                  forward + right
  z                  backward + left
  c                  backward + right

Speed:
  i                  increase linear speed
  k                  decrease linear speed
  o                  increase angular speed
  l                  decrease angular speed

Bus servo:
  r                  increase bus servo position
  f                  decrease bus servo position
  t                  move bus servo to center position
  y                  move bus servo to minimum position
  u                  move bus servo to maximum position
  v                  stop bus servo

Stop / quit:
  space              stop
  x                  stop
  Ctrl+C             quit

Note:
  Movement continues until another movement key, space, x, or Ctrl+C is pressed.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('linear_step', 0.02)
        self.declare_parameter('angular_step', 0.10)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('key_timeout_s', 0.0)
        self.declare_parameter('bus_servo_topic', '/ros_robot_controller/bus_servo/set_state')
        self.declare_parameter('bus_servo_id', 1)
        self.declare_parameter('bus_servo_min_position', 0)
        self.declare_parameter('bus_servo_max_position', 1000)
        self.declare_parameter('bus_servo_center_position', 500)
        self.declare_parameter('bus_servo_step', 50)
        self.declare_parameter('bus_servo_duration_s', 0.4)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.key_timeout_s = float(self.get_parameter('key_timeout_s').value)
        self.bus_servo_topic = self.get_parameter('bus_servo_topic').value
        self.bus_servo_id = int(self.get_parameter('bus_servo_id').value)
        self.bus_servo_min_position = int(self.get_parameter('bus_servo_min_position').value)
        self.bus_servo_max_position = int(self.get_parameter('bus_servo_max_position').value)
        self.bus_servo_position = int(self.get_parameter('bus_servo_center_position').value)
        self.bus_servo_step = int(self.get_parameter('bus_servo_step').value)
        self.bus_servo_duration_s = float(self.get_parameter('bus_servo_duration_s').value)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_key_time = self.get_clock().now()

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bus_servo_pub = self.create_publisher(SetBusServoState, self.bus_servo_topic, 10)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_cmd_vel)

        print(HELP_TEXT)
        self.print_speed()
        self.print_bus_servo()
        self.get_logger().info(f'Publishing keyboard commands to {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing bus servo commands to {self.bus_servo_topic}')

    def handle_key(self, key):
        if key in ('w', '\x1b[A'):
            self.set_motion(1.0, 0.0)
        elif key in ('s', '\x1b[B'):
            self.set_motion(-1.0, 0.0)
        elif key in ('a', '\x1b[D'):
            self.set_motion(0.0, 1.0)
        elif key in ('d', '\x1b[C'):
            self.set_motion(0.0, -1.0)
        elif key == 'q':
            self.set_motion(1.0, 1.0)
        elif key == 'e':
            self.set_motion(1.0, -1.0)
        elif key == 'z':
            self.set_motion(-1.0, 1.0)
        elif key == 'c':
            self.set_motion(-1.0, -1.0)
        elif key in (' ', 'x'):
            self.stop()
        elif key == 'i':
            self.linear_speed += self.linear_step
            self.print_speed()
        elif key == 'k':
            self.linear_speed = max(0.0, self.linear_speed - self.linear_step)
            self.print_speed()
        elif key == 'o':
            self.angular_speed += self.angular_step
            self.print_speed()
        elif key == 'l':
            self.angular_speed = max(0.0, self.angular_speed - self.angular_step)
            self.print_speed()
        elif key == 'r':
            self.move_bus_servo(self.bus_servo_position + self.bus_servo_step)
        elif key == 'f':
            self.move_bus_servo(self.bus_servo_position - self.bus_servo_step)
        elif key == 't':
            self.move_bus_servo(int(self.get_parameter('bus_servo_center_position').value))
        elif key == 'y':
            self.move_bus_servo(self.bus_servo_min_position)
        elif key == 'u':
            self.move_bus_servo(self.bus_servo_max_position)
        elif key == 'v':
            self.stop_bus_servo()

    def set_motion(self, linear_scale, angular_scale):
        self.target_linear = self.linear_speed * linear_scale
        self.target_angular = self.angular_speed * angular_scale
        self.last_key_time = self.get_clock().now()
        print(f'cmd_vel linear.x={self.target_linear:.2f}, angular.z={self.target_angular:.2f}')

    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_key_time = self.get_clock().now()
        self.publish_cmd_vel()
        print('cmd_vel stop')

    def publish_cmd_vel(self):
        if self.key_timed_out():
            self.target_linear = 0.0
            self.target_angular = 0.0

        msg = Twist()
        msg.linear.x = float(self.target_linear)
        msg.angular.z = float(self.target_angular)
        self.publisher.publish(msg)

    def key_timed_out(self):
        if self.key_timeout_s <= 0.0:
            return False
        elapsed = self.get_clock().now() - self.last_key_time
        return elapsed.nanoseconds / 1_000_000_000.0 > self.key_timeout_s

    def print_speed(self):
        print(f'linear_speed={self.linear_speed:.2f} m/s, angular_speed={self.angular_speed:.2f} rad/s')

    def move_bus_servo(self, position):
        position = int(max(self.bus_servo_min_position, min(self.bus_servo_max_position, position)))
        self.bus_servo_position = position

        state = BusServoState()
        state.present_id = [1, self.bus_servo_id]
        state.position = [1, position]

        msg = SetBusServoState()
        msg.duration = self.bus_servo_duration_s
        msg.state = [state]
        self.bus_servo_pub.publish(msg)
        self.print_bus_servo()

    def stop_bus_servo(self):
        state = BusServoState()
        state.present_id = [1, self.bus_servo_id]
        state.stop = [1, 1]

        msg = SetBusServoState()
        msg.duration = 0.0
        msg.state = [state]
        self.bus_servo_pub.publish(msg)
        print(f'bus_servo id={self.bus_servo_id} stop')

    def print_bus_servo(self):
        print(
            f'bus_servo id={self.bus_servo_id}, position={self.bus_servo_position}, '
            f'range={self.bus_servo_min_position}-{self.bus_servo_max_position}'
        )


class TerminalKeyboard:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def __enter__(self):
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_s=0.02):
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None

        key = sys.stdin.read(1)
        if key == '\x1b':
            ready, _, _ = select.select([sys.stdin], [], [], 0.001)
            if ready:
                key += sys.stdin.read(1)
                ready, _, _ = select.select([sys.stdin], [], [], 0.001)
                if ready:
                    key += sys.stdin.read(1)
        return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        with TerminalKeyboard() as keyboard:
            while rclpy.ok():
                key = keyboard.read_key()
                if key == '\x03':
                    break
                if key is not None:
                    node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
