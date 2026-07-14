import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from robot_status_gui.status_model import (
    centered_pose_to_map,
    mission_progress_label,
    mission_time_label,
    mode_label,
    parse_json_message,
    quaternion_to_yaw,
    return_reason_label,
    stored_object_label,
)

try:
    from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
    from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
    from PyQt5.QtWidgets import (
        QApplication,
        QFrame,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QSizePolicy,
        QStyle,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise RuntimeError(
        "PyQt5 is required. Install it with: sudo apt install python3-pyqt5"
    ) from exc


class RobotStatusGuiNode(Node):
    def __init__(self):
        super().__init__("robot_status_gui")

        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("policy_state_topic", "/rl_model_policy_state")
        self.declare_parameter("estimated_objects_topic", "/rl_estimated_objects")
        self.declare_parameter("control_topic", "/rl_model_policy_control")
        self.declare_parameter("pose_offset_x", 2.0)
        self.declare_parameter("pose_offset_y", 2.0)
        self.declare_parameter("connection_timeout_s", 1.0)

        self.pose_offset_x = float(self.get_parameter("pose_offset_x").value)
        self.pose_offset_y = float(self.get_parameter("pose_offset_y").value)
        self.connection_timeout_s = float(
            self.get_parameter("connection_timeout_s").value
        )
        self.pose = None
        self.path = []
        self.policy_state = {}
        self.object_state = {}
        self.pose_received_at = None
        self.policy_received_at = None
        self.objects_received_at = None

        self.control_pub = self.create_publisher(
            String,
            self.get_parameter("control_topic").value,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self.odometry_callback,
            10,
        )
        self.policy_sub = self.create_subscription(
            String,
            self.get_parameter("policy_state_topic").value,
            self.policy_callback,
            10,
        )
        self.objects_sub = self.create_subscription(
            String,
            self.get_parameter("estimated_objects_topic").value,
            self.objects_callback,
            10,
        )

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        map_x, map_y = centered_pose_to_map(
            position.x,
            position.y,
            self.pose_offset_x,
            self.pose_offset_y,
        )
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose = {
            "arena_x": float(position.x),
            "arena_y": float(position.y),
            "map_x": map_x,
            "map_y": map_y,
            "yaw": yaw,
        }
        if not self.path or math.hypot(
            map_x - self.path[-1][0],
            map_y - self.path[-1][1],
        ) >= 0.02:
            self.path.append((map_x, map_y))
            self.path = self.path[-1000:]
        self.pose_received_at = time.monotonic()

    def policy_callback(self, msg):
        self.policy_state = parse_json_message(msg.data)
        self.policy_received_at = time.monotonic()

    def objects_callback(self, msg):
        self.object_state = parse_json_message(msg.data)
        self.objects_received_at = time.monotonic()

    def request_motion_pause(self, paused):
        message = String()
        message.data = "pause_motion" if paused else "resume_motion"
        self.control_pub.publish(message)

    def topic_is_fresh(self, received_at):
        return (
            received_at is not None
            and time.monotonic() - received_at <= self.connection_timeout_s
        )

    def snapshot(self):
        return {
            "pose": self.pose,
            "path": list(self.path),
            "policy": dict(self.policy_state),
            "objects": list(self.object_state.get("objects", [])),
            "object_diagnostics": dict(self.object_state),
            "pose_connected": self.topic_is_fresh(self.pose_received_at),
            "policy_connected": self.topic_is_fresh(self.policy_received_at),
            "objects_connected": self.topic_is_fresh(self.objects_received_at),
        }


class ArenaMapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.snapshot = {}
        self.setMinimumSize(610, 610)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#11161a"))

        margin = 58.0
        side = max(100.0, min(self.width(), self.height()) - margin * 2.0)
        left = (self.width() - side) * 0.5
        top = (self.height() - side) * 0.5
        arena = QRectF(left, top, side, side)

        painter.fillRect(arena, QColor("#c4aa7c"))
        self.draw_grid(painter, arena)
        self.draw_zones(painter, arena)
        self.draw_candidate_points(painter, arena)
        self.draw_path(painter, arena)
        self.draw_mission_waypoint(painter, arena)
        self.draw_objects(painter, arena)
        self.draw_robot(painter, arena)

        painter.setPen(QPen(QColor("#f4f6f7"), 3.0))
        painter.drawRect(arena)
        self.draw_axes(painter, arena)
        self.draw_legend(painter, arena)

    def draw_grid(self, painter, arena):
        painter.setPen(QPen(QColor(255, 255, 255, 42), 1.0))
        for index in range(1, 8):
            coordinate = index * 0.5
            x, _ = self.to_pixel(coordinate, 0.0, arena)
            _, y = self.to_pixel(0.0, coordinate, arena)
            painter.drawLine(QPointF(x, arena.top()), QPointF(x, arena.bottom()))
            painter.drawLine(QPointF(arena.left(), y), QPointF(arena.right(), y))

    def draw_zones(self, painter, arena):
        storage_top_left = self.to_pixel(0.0, 0.4, arena)
        storage_bottom_right = self.to_pixel(0.4, 0.0, arena)
        start_top_left = self.to_pixel(3.6, 0.4, arena)
        start_bottom_right = self.to_pixel(4.0, 0.0, arena)

        painter.fillRect(
            QRectF(QPointF(*storage_top_left), QPointF(*storage_bottom_right)),
            QColor(39, 174, 96, 175),
        )
        painter.fillRect(
            QRectF(QPointF(*start_top_left), QPointF(*start_bottom_right)),
            QColor(52, 125, 219, 175),
        )
        painter.setFont(QFont("Sans", 8, QFont.DemiBold))
        painter.setPen(QColor("#effff5"))
        painter.drawText(
            QRectF(QPointF(*storage_top_left), QPointF(*storage_bottom_right)),
            Qt.AlignCenter,
            "STORAGE",
        )
        painter.setPen(QColor("#f3f8ff"))
        painter.drawText(
            QRectF(QPointF(*start_top_left), QPointF(*start_bottom_right)),
            Qt.AlignCenter,
            "START",
        )

    def draw_candidate_points(self, painter, arena):
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(22, 93, 122, 155))
        radius = max(3.0, arena.width() * 0.010)
        for row in range(6):
            for column in range(7):
                x, y = self.to_pixel(0.5 + column * 0.5, 1.0 + row * 0.5, arena)
                painter.drawEllipse(QPointF(x, y), radius, radius)

    def draw_path(self, painter, arena):
        path_points = self.snapshot.get("path", [])
        if len(path_points) < 2:
            return
        path = QPainterPath()
        first = self.to_pixel(path_points[0][0], path_points[0][1], arena)
        path.moveTo(QPointF(*first))
        for x, y in path_points[1:]:
            path.lineTo(QPointF(*self.to_pixel(x, y, arena)))
        painter.setPen(QPen(QColor(46, 215, 190, 115), 2.0))
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

    def draw_mission_waypoint(self, painter, arena):
        policy = self.snapshot.get("policy", {})
        mission = policy.get("mission", {}) if isinstance(policy, dict) else {}
        waypoint = mission.get("waypoint") if isinstance(mission, dict) else None
        if not isinstance(waypoint, dict):
            return
        try:
            waypoint_x = float(waypoint["x"]) + 2.0
            waypoint_y = float(waypoint["y"]) + 2.0
        except (KeyError, TypeError, ValueError):
            return

        pixel_x, pixel_y = self.to_pixel(waypoint_x, waypoint_y, arena)
        pose = self.snapshot.get("pose")
        if pose:
            robot_x, robot_y = self.to_pixel(pose["map_x"], pose["map_y"], arena)
            painter.setPen(QPen(QColor("#f5df4d"), 2.0, Qt.DashLine))
            painter.drawLine(QPointF(robot_x, robot_y), QPointF(pixel_x, pixel_y))
        painter.setPen(QPen(QColor("#11161a"), 2.0))
        painter.setBrush(QColor("#f5df4d"))
        painter.drawEllipse(QPointF(pixel_x, pixel_y), 9.0, 9.0)
        painter.setPen(QColor("#11161a"))
        painter.setFont(QFont("Sans", 7, QFont.Bold))
        painter.drawText(
            QRectF(pixel_x - 9.0, pixel_y - 8.0, 18.0, 16.0),
            Qt.AlignCenter,
            "W",
        )

    def draw_objects(self, painter, arena):
        for item in self.snapshot.get("objects", []):
            try:
                x = float(item["map_x"])
                y = float(item["map_y"])
            except (KeyError, TypeError, ValueError):
                continue
            pixel_x, pixel_y = self.to_pixel(x, y, arena)
            role = item.get("role", "avoid")
            colors = {
                "target": QColor("#f2c94c"),
                "avoid": QColor("#e76f51"),
                "other": QColor("#aab7bd"),
            }
            color = colors.get(role, colors["other"])
            painter.setPen(QPen(QColor("#11161a"), 2.0))
            painter.setBrush(color)
            painter.drawEllipse(QPointF(pixel_x, pixel_y), 10.0, 10.0)
            painter.setPen(QColor("#11161a"))
            painter.setFont(QFont("Sans", 7, QFont.Bold))
            painter.drawText(
                QRectF(pixel_x - 9.0, pixel_y - 8.0, 18.0, 16.0),
                Qt.AlignCenter,
                "T" if role == "target" else ("X" if role == "avoid" else "O"),
            )

    def draw_robot(self, painter, arena):
        pose = self.snapshot.get("pose")
        if not pose:
            return
        x, y = self.to_pixel(pose["map_x"], pose["map_y"], arena)
        robot_size = arena.width() * 0.10

        painter.save()
        painter.translate(x, y)
        painter.rotate(-math.degrees(pose["yaw"]))
        painter.setPen(QPen(QColor("#d9fbff"), 2.0))
        painter.setBrush(QColor("#168da1"))
        painter.drawRect(
            QRectF(-robot_size * 0.5, -robot_size * 0.5, robot_size, robot_size)
        )
        heading = QPolygonF(
            [
                QPointF(robot_size * 0.72, 0.0),
                QPointF(robot_size * 0.30, -robot_size * 0.22),
                QPointF(robot_size * 0.30, robot_size * 0.22),
            ]
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#f5df4d"))
        painter.drawPolygon(heading)
        painter.restore()

    def draw_axes(self, painter, arena):
        painter.setFont(QFont("Sans", 8))
        painter.setPen(QColor("#aeb9bf"))
        for index in range(9):
            value = index * 0.5
            x, _ = self.to_pixel(value, 0.0, arena)
            _, y = self.to_pixel(0.0, value, arena)
            painter.drawText(
                QRectF(x - 18, arena.bottom() + 7, 36, 18),
                Qt.AlignCenter,
                f"{value:g}",
            )
            painter.drawText(
                QRectF(arena.left() - 42, y - 9, 34, 18),
                Qt.AlignRight,
                f"{value:g}",
            )
        painter.setFont(QFont("Sans", 9, QFont.DemiBold))
        painter.drawText(
            QRectF(arena.left(), arena.bottom() + 28, arena.width(), 20),
            Qt.AlignCenter,
            "경기장 X (m)",
        )
        painter.save()
        painter.translate(arena.left() - 49, arena.center().y())
        painter.rotate(-90)
        painter.drawText(
            QRectF(-arena.height() / 2, -12, arena.height(), 20),
            Qt.AlignCenter,
            "경기장 Y (m)",
        )
        painter.restore()

    def draw_legend(self, painter, arena):
        entries = [
            (QColor("#f2c94c"), "목표"),
            (QColor("#e76f51"), "회피"),
            (QColor("#168da1"), "로봇"),
        ]
        x = arena.left() + 10
        y = arena.top() + 10
        painter.setFont(QFont("Sans", 8))
        for color, label in entries:
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(QPointF(x + 5, y + 5), 5, 5)
            painter.setPen(QColor("#20272b"))
            painter.drawText(QRectF(x + 15, y - 4, 48, 18), Qt.AlignVCenter, label)
            x += 62

    @staticmethod
    def to_pixel(map_x, map_y, arena):
        x = arena.left() + float(map_x) / 4.0 * arena.width()
        y = arena.bottom() - float(map_y) / 4.0 * arena.height()
        return x, y


class RobotStatusWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("AI Robot Challenge · Robot Status")
        self.resize(1180, 780)
        self.setMinimumSize(1040, 680)
        self.build_ui()

    def build_ui(self):
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 18, 20, 20)
        outer.setSpacing(14)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("AI ROBOT CHALLENGE")
        title.setObjectName("title")
        subtitle = QLabel("실시간 주행 상태 · 4 m × 4 m 경기장")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()
        self.connection_summary = QLabel("ROS 연결 확인 중")
        self.connection_summary.setObjectName("connectionSummary")
        header.addWidget(self.connection_summary)
        outer.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(18)
        self.map_widget = ArenaMapWidget()
        body.addWidget(self.map_widget, 1)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(350)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(20, 20, 20, 20)
        side.setSpacing(12)

        self.mode_label = QLabel("상태 대기 중")
        self.mode_label.setObjectName("mode")
        self.mode_label.setWordWrap(True)
        side.addWidget(self.section_label("현재 상태"))
        side.addWidget(self.mode_label)

        self.progress_label = self.value_label("미션 진행", "-")
        self.time_label = self.value_label("남은 시간", "03:00")
        self.return_reason_label = self.value_label("복귀 사유", "-")
        self.target_label = self.value_label("감지 목표", "-")
        self.stored_label = self.value_label("로봇 내부", "없음")
        self.coverage_label = self.value_label("탐색 단계", "-")
        side.addWidget(self.progress_label)
        side.addWidget(self.time_label)
        side.addWidget(self.return_reason_label)
        side.addWidget(self.target_label)
        side.addWidget(self.stored_label)
        side.addWidget(self.coverage_label)
        side.addWidget(self.separator())

        side.addWidget(self.section_label("로봇 자세"))
        self.position_label = self.value_label("위치", "-")
        self.heading_label = self.value_label("방향", "-")
        self.velocity_label = self.value_label("출력 속도", "-")
        side.addWidget(self.position_label)
        side.addWidget(self.heading_label)
        side.addWidget(self.velocity_label)
        side.addWidget(self.separator())

        side.addWidget(self.section_label("인지 상태"))
        self.object_count_label = self.value_label("지도 객체", "0")
        self.pose_connection = self.connection_row("위치 추적")
        self.policy_connection = self.connection_row("RL 상태")
        self.objects_connection = self.connection_row("객체 위치")
        side.addWidget(self.object_count_label)
        side.addLayout(self.pose_connection[0])
        side.addLayout(self.policy_connection[0])
        side.addLayout(self.objects_connection[0])
        note = QLabel("객체 마커는 카메라 중심점과 로봇 자세를 이용한 후보점 추정값입니다.")
        note.setObjectName("note")
        note.setWordWrap(True)
        side.addWidget(note)
        side.addStretch()

        self.pause_button = QPushButton("주행 일시정지")
        self.pause_button.setObjectName("pauseButton")
        self.pause_button.setMinimumHeight(52)
        self.pause_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        self.pause_button.setToolTip("인지와 상태 갱신은 유지하고 바퀴 명령만 0으로 만듭니다")
        self.pause_button.clicked.connect(self.toggle_pause)
        side.addWidget(self.pause_button)
        body.addWidget(sidebar)
        outer.addLayout(body, 1)

        self.setStyleSheet(
            """
            QWidget#root { background: #11161a; color: #eef2f3; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #f4f6f7; }
            QLabel#subtitle { font-size: 12px; color: #91a0a8; }
            QLabel#connectionSummary { color: #9aa8af; font-weight: 600; }
            QFrame#sidebar {
                background: #1a2227;
                border: 1px solid #2b373d;
                border-radius: 6px;
            }
            QLabel#section { color: #79d1dc; font-size: 11px; font-weight: 700; }
            QLabel#mode { color: #ffffff; font-size: 20px; font-weight: 700; padding-bottom: 6px; }
            QLabel#value { color: #dbe2e5; font-size: 13px; padding: 3px 0; }
            QLabel#note { color: #85959d; font-size: 11px; }
            QPushButton#pauseButton {
                background: #b84343;
                color: white;
                border: 0;
                border-radius: 5px;
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton#pauseButton:hover { background: #cf5454; }
            QPushButton#pauseButton[paused="true"] { background: #168da1; }
            QPushButton#pauseButton[paused="true"]:hover { background: #20a4b8; }
            """
        )

    def refresh(self):
        snapshot = self.node.snapshot()
        policy = snapshot["policy"]
        pose = snapshot["pose"]
        objects = snapshot["objects"]
        paused = bool(policy.get("motion_paused", False))

        self.map_widget.set_snapshot(snapshot)
        self.mode_label.setText(mode_label(policy))
        self.progress_label.setText(
            f"미션 진행   {mission_progress_label(policy)}"
        )
        self.time_label.setText(
            f"남은 시간   {mission_time_label(policy)}"
        )
        self.return_reason_label.setText(
            f"복귀 사유   {return_reason_label(policy)}"
        )
        self.target_label.setText(
            f"감지 목표   {policy.get('target_label') or '-'}"
        )
        self.stored_label.setText(
            f"로봇 내부   {stored_object_label(policy)}"
        )
        coverage = policy.get("coverage", {})
        phase = coverage.get("phase") if isinstance(coverage, dict) else None
        self.coverage_label.setText(f"탐색 단계   {phase or '-'}")

        if pose:
            self.position_label.setText(
                f"위치   X {pose['map_x']:.2f} m · Y {pose['map_y']:.2f} m"
            )
            heading = math.degrees(pose["yaw"]) % 360.0
            self.heading_label.setText(f"방향   {heading:.1f}°")
        else:
            self.position_label.setText("위치   -")
            self.heading_label.setText("방향   -")

        linear = float(policy.get("linear_x", 0.0))
        angular = float(policy.get("angular_z", 0.0))
        self.velocity_label.setText(
            f"출력 속도   {linear:.2f} m/s · {angular:.2f} rad/s"
        )
        target_count = sum(item.get("role") == "target" for item in objects)
        avoid_count = sum(item.get("role") == "avoid" for item in objects)
        self.object_count_label.setText(
            f"지도 객체   {len(objects)} · 목표 {target_count} · 회피 {avoid_count}"
        )

        self.set_connection(self.pose_connection, snapshot["pose_connected"])
        self.set_connection(self.policy_connection, snapshot["policy_connected"])
        self.set_connection(self.objects_connection, snapshot["objects_connected"])
        connected_count = sum(
            bool(snapshot[key])
            for key in ("pose_connected", "policy_connected", "objects_connected")
        )
        self.connection_summary.setText(f"ROS 데이터 {connected_count}/3 연결")

        self.pause_button.setProperty("paused", paused)
        self.pause_button.style().unpolish(self.pause_button)
        self.pause_button.style().polish(self.pause_button)
        if paused:
            self.pause_button.setText("주행 재개")
            self.pause_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPlay)
            )
        else:
            self.pause_button.setText("주행 일시정지")
            self.pause_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPause)
            )
        self.pause_button.setEnabled(bool(policy.get("active", False)) or paused)

    def toggle_pause(self):
        paused = bool(self.node.policy_state.get("motion_paused", False))
        self.node.request_motion_pause(not paused)

    @staticmethod
    def section_label(text):
        label = QLabel(text)
        label.setObjectName("section")
        return label

    @staticmethod
    def value_label(name, value):
        label = QLabel(f"{name}   {value}")
        label.setObjectName("value")
        label.setWordWrap(True)
        return label

    @staticmethod
    def separator():
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: #334148;")
        return line

    @staticmethod
    def connection_row(name):
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 1, 0, 1)
        dot = QFrame()
        dot.setFixedSize(9, 9)
        label = QLabel(name)
        label.setObjectName("value")
        status = QLabel("대기")
        status.setObjectName("note")
        layout.addWidget(dot)
        layout.addWidget(label)
        layout.addStretch()
        layout.addWidget(status)
        return layout, dot, status

    @staticmethod
    def set_connection(connection, connected):
        _, dot, status = connection
        color = "#3ac47d" if connected else "#6f7b81"
        dot.setStyleSheet(f"background: {color}; border-radius: 4px;")
        status.setText("연결" if connected else "대기")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusGuiNode()
    app = QApplication([sys.argv[0]])
    window = RobotStatusWindow(node)
    window.show()

    timer = QTimer()

    def update():
        # Drain all high-rate pose and policy callbacks without blocking Qt.
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.0)
        window.refresh()

    timer.timeout.connect(update)
    timer.start(50)
    exit_code = app.exec_()

    timer.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
