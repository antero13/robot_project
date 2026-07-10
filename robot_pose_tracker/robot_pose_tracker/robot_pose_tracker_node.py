import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_pose_tracker.pose_estimator import PoseEstimator


class RobotPoseTracker(Node):
    def __init__(self):
        super().__init__('robot_pose_tracker')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('status_topic', '/robot_pose/status')
        self.declare_parameter('reset_service', '/robot_pose/reset')
        self.declare_parameter('recalibrate_service', '/robot_pose/recalibrate_gyro')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('command_angular_scale', 1.0)
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.declare_parameter('gyro_deadband_rad_s', 0.01)
        self.declare_parameter('gyro_calibration_duration_s', 2.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('imu_timeout_s', 0.3)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.initial_x = self.get_float('initial_x')
        self.initial_y = self.get_float('initial_y')
        self.initial_yaw = math.radians(self.get_float('initial_yaw_deg'))
        self.linear_scale = self.get_float('linear_scale')
        self.command_angular_scale = self.get_float('command_angular_scale')
        self.imu_yaw_sign = self.get_float('imu_yaw_sign')
        self.gyro_deadband = self.get_float('gyro_deadband_rad_s')
        self.gyro_calibration_duration = self.get_float('gyro_calibration_duration_s')
        self.cmd_timeout = self.get_float('cmd_timeout_s')
        self.imu_timeout = self.get_float('imu_timeout_s')
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        publish_rate_hz = self.get_float('publish_rate_hz')
        self.validate_parameters(publish_rate_hz)

        self.estimator = PoseEstimator(self.initial_x, self.initial_y, self.initial_yaw)
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_gyro_z = 0.0
        self.latest_imu_time = None
        self.last_update_time = self.get_clock().now()

        self.gyro_calibrated = self.gyro_calibration_duration <= 0.0
        self.gyro_calibration_started_at = None
        self.gyro_bias_sum = 0.0
        self.gyro_bias_samples = 0
        self.gyro_bias_z = 0.0
        self.last_status = None

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.reset_srv = self.create_service(
            Trigger,
            self.get_parameter('reset_service').value,
            self.reset_callback,
        )
        self.recalibrate_srv = self.create_service(
            Trigger,
            self.get_parameter('recalibrate_service').value,
            self.recalibrate_callback,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.update)

        self.get_logger().info(
            f'Tracking pose from cmd_vel={self.cmd_vel_topic} and imu={self.imu_topic}; '
            f'publishing pose={self.pose_topic}, odom={self.odom_topic}'
        )
        self.get_logger().info(
            f'Initial pose: x={self.initial_x:.2f} m, y={self.initial_y:.2f} m, '
            f'yaw={math.degrees(self.initial_yaw):.1f} deg'
        )
        if not self.gyro_calibrated:
            self.get_logger().info(
                f'Keep the robot still for {self.gyro_calibration_duration:.1f} s '
                'while the gyro bias is measured.'
            )

    def validate_parameters(self, publish_rate_hz):
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than 0')
        if self.cmd_timeout <= 0.0:
            raise ValueError('cmd_timeout_s must be greater than 0')
        if self.imu_timeout <= 0.0:
            raise ValueError('imu_timeout_s must be greater than 0')
        if self.gyro_calibration_duration < 0.0:
            raise ValueError('gyro_calibration_duration_s cannot be negative')
        if self.imu_yaw_sign not in (-1.0, 1.0):
            raise ValueError('imu_yaw_sign must be 1.0 or -1.0')

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def imu_callback(self, msg):
        now = self.get_clock().now()
        self.latest_gyro_z = float(msg.angular_velocity.z)
        self.latest_imu_time = now

        if self.gyro_calibrated:
            return

        if self.gyro_calibration_started_at is None:
            self.gyro_calibration_started_at = now

        self.gyro_bias_sum += self.latest_gyro_z
        self.gyro_bias_samples += 1
        elapsed = self.seconds_between(now, self.gyro_calibration_started_at)

        if elapsed >= self.gyro_calibration_duration and self.gyro_bias_samples > 0:
            self.gyro_bias_z = self.gyro_bias_sum / self.gyro_bias_samples
            self.gyro_calibrated = True
            self.get_logger().info(
                f'Gyro calibration complete: z bias={self.gyro_bias_z:.6f} rad/s '
                f'from {self.gyro_bias_samples} samples.'
            )

    def update(self):
        now = self.get_clock().now()
        dt = self.seconds_between(now, self.last_update_time)
        self.last_update_time = now

        if dt <= 0.0 or dt > 0.5:
            self.publish_pose(now, 0.0, 0.0)
            return

        linear_velocity, command_angular_velocity = self.current_command(now)
        angular_velocity, status = self.current_angular_velocity(
            now,
            command_angular_velocity,
        )

        self.estimator.step(dt, linear_velocity, angular_velocity)
        self.publish_pose(now, linear_velocity, angular_velocity)
        self.publish_status(status)

    def current_command(self, now):
        if self.latest_cmd_time is None:
            return 0.0, 0.0

        age = self.seconds_between(now, self.latest_cmd_time)
        if age > self.cmd_timeout:
            return 0.0, 0.0

        return (
            float(self.latest_cmd.linear.x) * self.linear_scale,
            float(self.latest_cmd.angular.z) * self.command_angular_scale,
        )

    def current_angular_velocity(self, now, command_angular_velocity):
        if self.latest_imu_time is None:
            return command_angular_velocity, 'WAITING_FOR_IMU_CMD_FALLBACK'

        imu_age = self.seconds_between(now, self.latest_imu_time)
        if imu_age > self.imu_timeout:
            return command_angular_velocity, 'IMU_TIMEOUT_CMD_FALLBACK'

        if not self.gyro_calibrated:
            return command_angular_velocity, 'CALIBRATING_GYRO_CMD_FALLBACK'

        gyro_z = (self.latest_gyro_z - self.gyro_bias_z) * self.imu_yaw_sign
        if abs(gyro_z) < self.gyro_deadband:
            gyro_z = 0.0
        return gyro_z, 'TRACKING_WITH_IMU'

    def publish_pose(self, now, linear_velocity, angular_velocity):
        stamp = now.to_msg()
        qz = math.sin(self.estimator.yaw * 0.5)
        qw = math.cos(self.estimator.yaw * 0.5)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = self.estimator.x
        pose_msg.pose.position.y = self.estimator.y
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        self.set_covariance(odom_msg)
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = pose_msg.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.estimator.x
            transform.transform.translation.y = self.estimator.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def set_covariance(self, odom_msg):
        # Uncertainty grows because translation is inferred from commanded speed, not encoders.
        position_sigma = 0.03 + 0.15 * self.estimator.distance_travelled
        heading_sigma = math.radians(2.0) + 0.05 * self.estimator.rotation_travelled

        odom_msg.pose.covariance[0] = position_sigma ** 2
        odom_msg.pose.covariance[7] = position_sigma ** 2
        odom_msg.pose.covariance[14] = 1_000_000.0
        odom_msg.pose.covariance[21] = 1_000_000.0
        odom_msg.pose.covariance[28] = 1_000_000.0
        odom_msg.pose.covariance[35] = heading_sigma ** 2

        odom_msg.twist.covariance[0] = 0.05 ** 2
        odom_msg.twist.covariance[7] = 0.05 ** 2
        odom_msg.twist.covariance[14] = 1_000_000.0
        odom_msg.twist.covariance[21] = 1_000_000.0
        odom_msg.twist.covariance[28] = 1_000_000.0
        odom_msg.twist.covariance[35] = math.radians(5.0) ** 2

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

        if status != self.last_status:
            self.get_logger().info(f'Pose tracker status: {status}')
            self.last_status = status

    def reset_callback(self, _request, response):
        self.estimator.reset(self.initial_x, self.initial_y, self.initial_yaw)
        self.last_update_time = self.get_clock().now()
        response.success = True
        response.message = (
            f'Pose reset to x={self.initial_x:.2f}, y={self.initial_y:.2f}, '
            f'yaw={math.degrees(self.initial_yaw):.1f} deg.'
        )
        self.get_logger().info(response.message)
        return response

    def recalibrate_callback(self, _request, response):
        self.gyro_calibrated = self.gyro_calibration_duration <= 0.0
        self.gyro_calibration_started_at = None
        self.gyro_bias_sum = 0.0
        self.gyro_bias_samples = 0
        self.gyro_bias_z = 0.0
        response.success = True
        response.message = (
            'Gyro recalibration started. Keep the robot still.'
            if not self.gyro_calibrated
            else 'Gyro calibration is disabled by parameter.'
        )
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def seconds_between(newer, older):
        return (newer - older).nanoseconds / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
