import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class ArenaVisualizer(Node):
    def __init__(self):
        super().__init__('arena_visualizer')

        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('path_topic', '/robot_path')
        self.declare_parameter('arena_markers_topic', '/arena_markers')
        self.declare_parameter('robot_markers_topic', '/robot_pose/markers')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('arena_width_m', 4.0)
        self.declare_parameter('arena_height_m', 4.0)
        self.declare_parameter('zone_size_m', 0.4)
        self.declare_parameter('robot_width_m', 0.4)
        self.declare_parameter('robot_length_m', 0.4)
        self.declare_parameter('path_min_distance_m', 0.02)
        self.declare_parameter('path_min_yaw_deg', 2.0)
        self.declare_parameter('path_reset_jump_m', 0.75)
        self.declare_parameter('max_path_poses', 5000)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.arena_markers_topic = self.get_parameter('arena_markers_topic').value
        self.robot_markers_topic = self.get_parameter('robot_markers_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.arena_width = self.get_float('arena_width_m')
        self.arena_height = self.get_float('arena_height_m')
        self.zone_size = self.get_float('zone_size_m')
        self.robot_width = self.get_float('robot_width_m')
        self.robot_length = self.get_float('robot_length_m')
        self.path_min_distance = self.get_float('path_min_distance_m')
        self.path_min_yaw = math.radians(self.get_float('path_min_yaw_deg'))
        self.path_reset_jump = self.get_float('path_reset_jump_m')
        self.max_path_poses = int(self.get_parameter('max_path_poses').value)

        self.validate_parameters()

        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.arena_pub = self.create_publisher(
            MarkerArray,
            self.arena_markers_topic,
            static_qos,
        )
        self.robot_pub = self.create_publisher(
            MarkerArray,
            self.robot_markers_topic,
            10,
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_path_pose = None
        self.static_timer = self.create_timer(1.0, self.publish_arena)
        self.publish_arena()

        self.get_logger().info(
            f'Visualizing arena from {self.pose_topic}; publishing path={self.path_topic}'
        )

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    def validate_parameters(self):
        if self.arena_width <= 0.0 or self.arena_height <= 0.0:
            raise ValueError('arena dimensions must be greater than 0')
        if self.zone_size <= 0.0:
            raise ValueError('zone_size_m must be greater than 0')
        if self.robot_width <= 0.0 or self.robot_length <= 0.0:
            raise ValueError('robot dimensions must be greater than 0')
        if self.max_path_poses <= 0:
            raise ValueError('max_path_poses must be greater than 0')

    def pose_callback(self, msg):
        self.publish_robot(msg)
        self.update_path(msg)

    def publish_arena(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers = [
            self.make_floor_marker(stamp),
            self.make_wall_marker(stamp),
            self.make_zone_marker(
                stamp,
                marker_id=0,
                namespace='storage_zone',
                center_x=self.zone_size / 2.0,
                center_y=self.zone_size / 2.0,
                red=0.15,
                green=0.80,
                blue=0.30,
            ),
            self.make_zone_marker(
                stamp,
                marker_id=0,
                namespace='start_zone',
                center_x=self.arena_width - self.zone_size / 2.0,
                center_y=self.zone_size / 2.0,
                red=0.20,
                green=0.45,
                blue=1.00,
            ),
            self.make_grid_points_marker(stamp),
            self.make_text_marker(
                stamp,
                marker_id=0,
                text='STORAGE / FLAG',
                x=0.55,
                y=0.20,
                red=0.20,
                green=1.00,
                blue=0.35,
            ),
            self.make_text_marker(
                stamp,
                marker_id=1,
                text='START',
                x=self.arena_width - 0.55,
                y=0.20,
                red=0.35,
                green=0.65,
                blue=1.00,
            ),
        ]
        self.arena_pub.publish(markers)

    def make_floor_marker(self, stamp):
        marker = self.new_marker(stamp, 'arena', 0, Marker.CUBE)
        marker.pose.position.x = self.arena_width / 2.0
        marker.pose.position.y = self.arena_height / 2.0
        marker.pose.position.z = -0.03
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.arena_width
        marker.scale.y = self.arena_height
        marker.scale.z = 0.04
        self.set_color(marker, 0.46, 0.35, 0.23, 0.50)
        return marker

    def make_wall_marker(self, stamp):
        marker = self.new_marker(stamp, 'arena', 1, Marker.LINE_STRIP)
        marker.scale.x = 0.05
        marker.points = [
            self.point(0.0, 0.0, 0.03),
            self.point(self.arena_width, 0.0, 0.03),
            self.point(self.arena_width, self.arena_height, 0.03),
            self.point(0.0, self.arena_height, 0.03),
            self.point(0.0, 0.0, 0.03),
        ]
        self.set_color(marker, 0.95, 0.95, 0.95, 1.0)
        return marker

    def make_zone_marker(
        self,
        stamp,
        marker_id,
        namespace,
        center_x,
        center_y,
        red,
        green,
        blue,
    ):
        marker = self.new_marker(stamp, namespace, marker_id, Marker.CUBE)
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.01
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.zone_size
        marker.scale.y = self.zone_size
        marker.scale.z = 0.02
        self.set_color(marker, red, green, blue, 0.60)
        return marker

    def make_grid_points_marker(self, stamp):
        marker = self.new_marker(stamp, 'object_candidate_points', 0, Marker.SPHERE_LIST)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.04
        marker.points = [
            self.point(0.5 + column * 0.5, 1.0 + row * 0.5, 0.04)
            for row in range(6)
            for column in range(7)
        ]
        self.set_color(marker, 1.00, 0.72, 0.12, 0.90)
        return marker

    def make_text_marker(self, stamp, marker_id, text, x, y, red, green, blue):
        marker = self.new_marker(stamp, 'labels', marker_id, Marker.TEXT_VIEW_FACING)
        marker.text = text
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.14
        self.set_color(marker, red, green, blue, 1.0)
        return marker

    def publish_robot(self, pose_msg):
        stamp = pose_msg.header.stamp
        markers = MarkerArray()
        yaw = self.pose_yaw(pose_msg)
        heading_length = self.robot_length * 0.95
        heading_start = self.point(
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            0.22,
        )
        heading_end = self.point(
            pose_msg.pose.position.x + heading_length * math.cos(yaw),
            pose_msg.pose.position.y + heading_length * math.sin(yaw),
            0.22,
        )

        body = self.new_marker(stamp, 'robot', 0, Marker.CUBE)
        body.pose = deepcopy(pose_msg.pose)
        body.pose.position.z = 0.07
        body.scale.x = self.robot_length
        body.scale.y = self.robot_width
        body.scale.z = 0.12
        self.set_color(body, 0.05, 0.75, 0.95, 0.80)

        heading = self.new_marker(stamp, 'robot', 1, Marker.ARROW)
        heading.points = [heading_start, heading_end]
        heading.scale.x = 0.06
        heading.scale.y = 0.14
        heading.scale.z = 0.18
        self.set_color(heading, 1.00, 0.20, 0.15, 1.0)

        nose = self.new_marker(stamp, 'robot', 2, Marker.SPHERE)
        nose.pose.position.x = heading_end.x
        nose.pose.position.y = heading_end.y
        nose.pose.position.z = 0.22
        nose.pose.orientation.w = 1.0
        nose.scale.x = 0.14
        nose.scale.y = 0.14
        nose.scale.z = 0.08
        self.set_color(nose, 1.00, 0.75, 0.05, 1.0)

        markers.markers = [body, heading, nose]
        self.robot_pub.publish(markers)

    def update_path(self, pose_msg):
        if self.last_path_pose is not None:
            distance = self.pose_distance(pose_msg, self.last_path_pose)
            yaw_change = abs(
                self.normalize_angle(
                    self.pose_yaw(pose_msg) - self.pose_yaw(self.last_path_pose)
                )
            )

            if distance > self.path_reset_jump:
                self.path.poses = []
                self.get_logger().info('Large pose jump detected; clearing visualized path.')
            elif distance < self.path_min_distance and yaw_change < self.path_min_yaw:
                return

        self.path.header.stamp = pose_msg.header.stamp
        self.path.poses.append(pose_msg)
        if len(self.path.poses) > self.max_path_poses:
            self.path.poses = self.path.poses[-self.max_path_poses:]
        self.last_path_pose = pose_msg
        self.path_pub.publish(self.path)

    def new_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    @staticmethod
    def set_color(marker, red, green, blue, alpha):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    @staticmethod
    def point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def pose_distance(first, second):
        dx = first.pose.position.x - second.pose.position.x
        dy = first.pose.position.y - second.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def pose_yaw(pose_msg):
        orientation = pose_msg.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def normalize_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ArenaVisualizer()
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
