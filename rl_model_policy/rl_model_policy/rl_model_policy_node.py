import json
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from std_msgs.msg import Bool, Float32, String

from rl_model_policy.coverage_controller import (
    CoverageController,
    generate_coverage_legs,
)
from rl_model_policy.mission_coordinator import (
    MissionCoordinator,
    MissionPhase,
    ReturnReason,
    reverse_exit_command,
    waypoint_command,
)
from rl_model_policy.observation import (
    OBSERVATION_DIM,
    OBSERVATION_NAMES,
    SUPPORTED_OBSERVATION_DIMS,
    estimate_target_image_x,
    estimate_target_world_bearing,
    make_pose_observation,
    model_uses_pose_observation,
    pose_is_usable,
    quaternion_to_yaw,
    validate_observation,
)
from rl_model_policy.pickup_trigger import pickup_is_ready
from rl_model_policy.leave_start import make_leave_start_command
from rl_model_policy.target_reacquisition import reacquire_angular_velocity
from rl_model_policy.target_confirmation import target_is_confirmed
from rl_model_policy.target_activation import target_is_eligible

try:
    import torch
except ImportError as exc:
    torch = None
    TORCH_IMPORT_ERROR = exc
else:
    TORCH_IMPORT_ERROR = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


if torch is not None:
    class PolicyNetwork(torch.nn.Module):
        def __init__(self, observation_dim):
            super().__init__()
            self.log_std_parameter = torch.nn.Parameter(torch.zeros(2))
            self.net_container = torch.nn.Sequential(
                torch.nn.Linear(observation_dim, 128),
                torch.nn.ELU(),
                torch.nn.Linear(128, 128),
                torch.nn.ELU(),
                torch.nn.Linear(128, 64),
                torch.nn.ELU(),
            )
            self.policy_layer = torch.nn.Linear(64, 2)
            self.value_layer = torch.nn.Linear(64, 1)

        def forward(self, obs):
            features = self.net_container(obs)
            return self.policy_layer(features)


class RLModelPolicyNode(Node):
    """Run the trained Isaac Lab/skrl policy from YOLO-derived observations."""

    MODE_IDLE = "IDLE"
    MODE_LEAVE_START = "LEAVE_START"
    MODE_TRACK_TARGET = "TRACK_TARGET"
    MODE_LOCAL_REACQUIRE = "LOCAL_REACQUIRE"
    MODE_COVERAGE_SEARCH = "COVERAGE_SEARCH"
    MODE_WAITING_FOR_POSE = "WAITING_FOR_POSE"
    MODE_GRAB_SEQUENCE = "GRAB_SEQUENCE"
    MODE_RETURN_TO_STORAGE = "RETURN_TO_STORAGE"
    MODE_ENTER_STORAGE = "ENTER_STORAGE"
    MODE_DEPOSIT = "DEPOSIT"
    MODE_EXIT_STORAGE = "EXIT_STORAGE"
    MODE_MISSION_COMPLETE = "MISSION_COMPLETE"
    MODE_MISSION_TIMEOUT = "MISSION_TIMEOUT"

    GRAB_TRACKING = "TRACKING"
    GRAB_OPENING = "OPENING"
    GRAB_FINAL_FORWARD = "FINAL_FORWARD"
    GRAB_CLOSING = "CLOSING"
    GRAB_COMPLETE = "GRABBED"

    def __init__(self):
        super().__init__("rl_model_policy")

        self.declare_parameter("model_path", "mission_manager/models/rl_avoid_search_best.pt")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_object_topic", "/target_object")
        self.declare_parameter("target_label_topic", "/target_label")
        self.declare_parameter("target_visibility_topic", "/target_visible")
        self.declare_parameter("target_center_y_topic", "/target_center_y")
        self.declare_parameter("avoid_object_topic", "/avoid_object")
        self.declare_parameter("avoid_objects_topic", "/avoid_objects")
        self.declare_parameter("control_topic", "/rl_model_policy_control")
        self.declare_parameter("state_topic", "/rl_model_policy_state")
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("pwm_servo_topic", "/ros_robot_controller/pwm_servo/set_state")
        self.declare_parameter("bus_servo_topic", "/ros_robot_controller/bus_servo/set_state")

        self.declare_parameter("active_on_start", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("timer_rate_hz", 20.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("target_confirmation_window", 5)
        self.declare_parameter("target_confirmation_min_detections", 3)
        self.declare_parameter("target_activation_center_y_min", 0.30)
        self.declare_parameter("target_tracking_center_y_min", 0.22)
        self.declare_parameter("target_bearing_prediction_enabled", True)
        self.declare_parameter("avoid_timeout_s", 0.25)
        self.declare_parameter("episode_length_s", 18.0)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("pose_observation_enabled", False)
        self.declare_parameter("arena_half_extent_m", 2.0)
        self.declare_parameter("pose_bounds_tolerance_m", 0.25)
        self.declare_parameter("camera_horizontal_fov_deg", 80.0)

        self.declare_parameter("leave_start_enabled", True)
        self.declare_parameter("leave_start_distance_m", 0.55)
        self.declare_parameter("leave_start_speed", 0.25)
        self.declare_parameter("leave_start_heading_gain", 1.5)
        self.declare_parameter("leave_start_max_angular_speed", 0.40)

        self.declare_parameter("coverage_enabled", True)
        self.declare_parameter("coverage_min_x", -0.75)
        self.declare_parameter("coverage_max_x", 1.25)
        self.declare_parameter("coverage_main_road_y", -1.3343)
        self.declare_parameter("coverage_scan_end_y", 1.0)
        self.declare_parameter("coverage_lane_spacing", 1.0)
        self.declare_parameter("coverage_scan_speed", 0.24)
        self.declare_parameter("coverage_transit_speed", 0.30)
        self.declare_parameter("coverage_return_speed", 0.24)
        self.declare_parameter("coverage_waypoint_tolerance", 0.10)
        self.declare_parameter("coverage_heading_tolerance", 0.08)
        self.declare_parameter("coverage_heading_gain", 2.4)
        self.declare_parameter("coverage_max_angular_speed", 1.00)
        self.declare_parameter("coverage_turn_in_place_threshold", 0.65)
        self.declare_parameter("coverage_avoid_danger_threshold", 0.20)
        self.declare_parameter("coverage_avoid_angular_speed", 0.45)
        self.declare_parameter("coverage_avoid_linear_scale", 0.70)
        self.declare_parameter("coverage_rejoin_speed", 0.20)
        self.declare_parameter("coverage_reacquire_duration_s", 1.5)
        self.declare_parameter("coverage_reacquire_reverse_after_s", 0.75)
        self.declare_parameter("coverage_reacquire_angular_z", 0.35)

        self.declare_parameter("avoid_area_ratio", 0.42)
        self.declare_parameter("avoid_center_band", 0.75)
        self.declare_parameter("avoid_center_corridor", 0.15)
        self.declare_parameter("avoid_vfh_center_weight", 0.5)
        self.declare_parameter("avoid_only_if_closer_than_target", False)
        self.declare_parameter("avoid_closer_ratio", 0.85)

        self.declare_parameter("max_forward_speed", 0.20)
        self.declare_parameter("max_reverse_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("speed_scale", 0.75)
        self.declare_parameter("max_linear_action_delta", 0.25)
        self.declare_parameter("max_angular_action_delta", 0.08)
        self.declare_parameter("action_filter_alpha", 0.55)
        self.declare_parameter("publish_stop_when_inactive", True)
        self.declare_parameter("state_preprocessor_epsilon", 1e-8)

        self.declare_parameter("full_mission_enabled", True)
        self.declare_parameter("mission_duration_s", 180.0)
        self.declare_parameter("force_return_remaining_s", 30.0)
        self.declare_parameter("storage_capacity", 4)
        self.declare_parameter("target_object_count", 7)
        self.declare_parameter("storage_main_road_y", -1.3343)
        self.declare_parameter("storage_staging_x", -1.75)
        self.declare_parameter("storage_staging_y", -1.25)
        self.declare_parameter("storage_exit_y", -1.0)
        self.declare_parameter("storage_center_x", -1.75)
        self.declare_parameter("storage_center_y", -1.75)
        self.declare_parameter("storage_entry_yaw_deg", -90.0)
        self.declare_parameter("storage_return_speed", 0.25)
        self.declare_parameter("storage_entry_speed", 0.12)
        self.declare_parameter("storage_exit_reverse_speed", 0.16)
        self.declare_parameter("storage_waypoint_tolerance", 0.10)
        self.declare_parameter("storage_entry_tolerance", 0.04)
        self.declare_parameter("storage_heading_tolerance", 0.14)
        self.declare_parameter("storage_final_yaw_tolerance", 0.12)
        self.declare_parameter("storage_heading_gain", 1.5)
        self.declare_parameter("storage_max_angular_speed", 0.60)
        self.declare_parameter("storage_avoid_danger_threshold", 0.20)

        self.declare_parameter("gripper_enabled", True)
        self.declare_parameter("gripper_type", "bus")
        self.declare_parameter("gripper_servo_id", 1)
        self.declare_parameter("gripper_open_position", 1000)
        self.declare_parameter("gripper_closed_position", 300)
        self.declare_parameter("gripper_move_duration_s", 0.5)
        self.declare_parameter("grab_center_tolerance", 0.18)
        self.declare_parameter("grab_area_ratio", 0.70)
        self.declare_parameter("grab_detection_timeout_s", 0.25)
        self.declare_parameter("final_forward_linear_x", 0.20)
        self.declare_parameter("final_forward_duration_s", 1.2)
        self.declare_parameter("grab_duration_s", 1.0)
        self.declare_parameter("stop_after_grab", False)

        self.active = bool(self.get_parameter("active_on_start").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_label = None
        self.latest_target_label_time = None
        self.latest_target_center_y = None
        self.latest_target_center_y_time = None
        self.target_confirmation_window = int(
            self.get_parameter("target_confirmation_window").value
        )
        self.target_confirmation_min_detections = int(
            self.get_parameter("target_confirmation_min_detections").value
        )
        target_is_confirmed(
            [],
            self.target_confirmation_window,
            self.target_confirmation_min_detections,
        )
        self.target_visibility_history = deque(
            maxlen=self.target_confirmation_window
        )
        self.latest_avoid = None
        self.latest_avoid_time = None
        self.latest_avoid_objects = []
        self.latest_avoid_objects_time = None
        self.latest_pose_time = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_yaw_rate = 0.0
        self.last_target_world_bearing = None
        self.time_since_target_seen = self.get_float("episode_length_s")
        self.last_target_direction = 1.0
        self.raw_action = [0.0, 0.0]
        self.filtered_action = [0.0, 0.0]
        self.last_cmd = (0.0, 0.0)
        self.grab_state = self.GRAB_TRACKING
        self.grab_state_started_at = self.get_clock().now()
        self.grab_state_elapsed_offset_s = 0.0
        self.motion_paused = False
        self.pickup_label = None
        self.control_mode = self.MODE_IDLE
        self.leave_start_active = (
            self.active and bool(self.get_parameter("leave_start_enabled").value)
        )
        self.leave_start_origin = None
        self.leave_start_yaw = None
        self.leave_start_traveled_m = 0.0
        if self.leave_start_active:
            self.control_mode = self.MODE_LEAVE_START
        self.had_visible_target = False
        self.target_lost_started_at = None
        self.coverage_command = None
        self.mission_waypoint = None
        self.return_lane_x = 0.0

        self.mission = MissionCoordinator(
            storage_capacity=int(self.get_parameter("storage_capacity").value),
            target_object_count=int(
                self.get_parameter("target_object_count").value
            ),
            mission_duration_s=self.get_float("mission_duration_s"),
            force_return_remaining_s=self.get_float(
                "force_return_remaining_s"
            ),
        )
        if self.active:
            self.mission.start(self.now_s())

        self.policy = None
        self.model_observation_dim = OBSERVATION_DIM
        self.obs_mean = None
        self.obs_variance = None
        self.model_path = None
        self.load_model()
        self.coverage_controller = self.create_coverage_controller()

        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.pwm_servo_pub = self.create_publisher(
            SetPWMServoState,
            self.get_parameter("pwm_servo_topic").value,
            10,
        )
        self.bus_servo_pub = self.create_publisher(
            SetBusServoState,
            self.get_parameter("bus_servo_topic").value,
            10,
        )
        self.target_sub = self.create_subscription(
            PointStamped,
            self.get_parameter("target_object_topic").value,
            self.target_callback,
            10,
        )
        self.target_label_sub = self.create_subscription(
            String,
            self.get_parameter("target_label_topic").value,
            self.target_label_callback,
            10,
        )
        self.target_visibility_sub = self.create_subscription(
            Bool,
            self.get_parameter("target_visibility_topic").value,
            self.target_visibility_callback,
            10,
        )
        self.target_center_y_sub = self.create_subscription(
            Float32,
            self.get_parameter("target_center_y_topic").value,
            self.target_center_y_callback,
            10,
        )
        self.avoid_sub = self.create_subscription(
            PointStamped,
            self.get_parameter("avoid_object_topic").value,
            self.avoid_callback,
            10,
        )
        self.avoid_objects_sub = self.create_subscription(
            String,
            self.get_parameter("avoid_objects_topic").value,
            self.avoid_objects_callback,
            10,
        )
        self.odometry_sub = None
        if (
            self.pose_observation_is_active()
            or self.coverage_is_enabled()
            or self.full_mission_is_enabled()
        ):
            self.odometry_sub = self.create_subscription(
                Odometry,
                self.get_parameter("odometry_topic").value,
                self.odometry_callback,
                10,
            )
        if self.coverage_is_enabled() and not self.pose_observation_is_active():
            self.get_logger().info(
                "Odometry is enabled for coverage search but remains excluded from RL observations"
            )
        elif self.model_observation_dim == OBSERVATION_DIM and self.odometry_sub is None:
            self.get_logger().info(
                "Legacy 18-input policy loaded with pose disabled; pose inputs remain zero"
            )
        elif self.odometry_sub is None:
            self.get_logger().info(
                "10-input YOLO-only policy loaded; odometry and pose correction are disabled"
            )
        self.control_sub = self.create_subscription(
            String,
            self.get_parameter("control_topic").value,
            self.control_callback,
            10,
        )

        timer_rate_hz = self.get_float("timer_rate_hz")
        self.timer = self.create_timer(1.0 / timer_rate_hz, self.tick)

        self.get_logger().info(
            f"RL model policy ready. active={self.active}, dry_run={self.dry_run}, model={self.model_path}"
        )
        self.get_logger().info(
            "Send start/pause_motion/resume_motion/stop on "
            f"{self.get_parameter('control_topic').value}; publishing cmd_vel on "
            f"{self.get_parameter('cmd_vel_topic').value}"
        )

    def load_model(self):
        if torch is None:
            self.get_logger().error(f"PyTorch is not installed: {TORCH_IMPORT_ERROR}")
            self.active = False
            return

        model_path = self.resolve_model_path(str(self.get_parameter("model_path").value))
        if model_path is None:
            self.get_logger().error(
                "Cannot find RL model. Pass -p model_path:=<path to rl_avoid_search_best.pt>."
            )
            self.active = False
            return

        checkpoint = torch.load(str(model_path), map_location="cpu")
        policy_state = checkpoint.get("policy", {})
        first_weight = policy_state.get("net_container.0.weight")
        if first_weight is None or len(first_weight.shape) != 2:
            shape = None if first_weight is None else tuple(first_weight.shape)
            raise RuntimeError(
                "RL checkpoint has no valid policy net_container.0.weight; "
                f"got {shape}"
            )
        observation_dim = int(first_weight.shape[1])
        if (
            tuple(first_weight.shape) != (128, observation_dim)
            or observation_dim not in SUPPORTED_OBSERVATION_DIMS
        ):
            raise RuntimeError(
                "RL checkpoint observation contract mismatch: supported input "
                f"dimensions are {SUPPORTED_OBSERVATION_DIMS}, got {tuple(first_weight.shape)}"
            )

        preprocessor = checkpoint.get("state_preprocessor", {})
        obs_mean = preprocessor.get("running_mean")
        obs_variance = preprocessor.get("running_variance")
        if obs_mean is None or tuple(obs_mean.shape) != (observation_dim,):
            raise RuntimeError(
                f"Checkpoint running_mean must have shape ({observation_dim},)"
            )
        if obs_variance is None or tuple(obs_variance.shape) != (observation_dim,):
            raise RuntimeError(
                f"Checkpoint running_variance must have shape ({observation_dim},)"
            )

        self.model_observation_dim = observation_dim
        self.policy = PolicyNetwork(observation_dim)
        self.policy.load_state_dict(policy_state, strict=True)
        self.policy.eval()

        self.obs_mean = obs_mean.float()
        self.obs_variance = obs_variance.float()
        self.model_path = str(model_path)
        self.get_logger().info(
            f"Loaded {observation_dim}-observation RL checkpoint: {self.model_path}"
        )

    def resolve_model_path(self, raw_path):
        candidates = []
        raw = Path(raw_path)
        candidates.append(raw)
        candidates.append(Path.cwd() / raw)

        default_name = "rl_avoid_search_best.pt"
        for root in [Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents]:
            candidates.append(root / "mission_manager" / "models" / default_name)
            candidates.append(root / "models" / default_name)

        if get_package_share_directory is not None:
            for package in ("mission_manager", "rl_model_policy"):
                try:
                    share = Path(get_package_share_directory(package))
                except Exception:
                    continue
                candidates.append(share / "models" / default_name)

        for candidate in candidates:
            try:
                if candidate.exists():
                    return candidate.resolve()
            except OSError:
                continue
        return None

    def target_callback(self, msg):
        self.latest_target = self.make_point(msg.point.x, msg.point.y, msg.point.z)
        self.latest_target_time = self.get_clock().now()
        if abs(self.latest_target.x) > 1e-6:
            self.last_target_direction = 1.0 if self.latest_target.x >= 0.0 else -1.0
        if self.is_fresh(self.latest_pose_time, "pose_timeout_s"):
            fov_rad = math.radians(self.get_float("camera_horizontal_fov_deg"))
            self.last_target_world_bearing = estimate_target_world_bearing(
                self.robot_yaw,
                self.latest_target.x,
                fov_rad,
            )

    def target_label_callback(self, msg):
        self.latest_target_label = msg.data.strip() or None
        self.latest_target_label_time = self.get_clock().now()

    def target_visibility_callback(self, msg):
        visible = bool(msg.data)
        self.target_visibility_history.append(visible)

    def target_center_y_callback(self, msg):
        self.latest_target_center_y = float(msg.data)
        self.latest_target_center_y_time = self.get_clock().now()

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_x = float(position.x)
        self.robot_y = float(position.y)
        self.robot_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.robot_yaw_rate = float(msg.twist.twist.angular.z)
        self.latest_pose_time = self.get_clock().now()
        if self.last_target_world_bearing is None:
            self.last_target_world_bearing = self.robot_yaw

    def avoid_callback(self, msg):
        self.latest_avoid = self.make_point(msg.point.x, msg.point.y, msg.point.z)
        self.latest_avoid_time = self.get_clock().now()

    def avoid_objects_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid avoid_objects JSON: {exc}")
            return

        raw_objects = payload.get("objects", payload) if isinstance(payload, dict) else payload
        if not isinstance(raw_objects, list):
            return

        objects = []
        for raw in raw_objects:
            if not isinstance(raw, dict):
                continue
            try:
                x = float(raw.get("x", raw.get("point_x", 0.0)))
                y = float(raw.get("y", raw.get("point_y", 0.0)))
                confidence = float(raw.get("confidence", raw.get("z", 1.0)))
            except (TypeError, ValueError):
                continue
            objects.append(self.make_point(x, y, confidence))

        self.latest_avoid_objects = objects
        self.latest_avoid_objects_time = self.get_clock().now()

    def control_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ("start", "run", "demo"):
            if self.policy is None:
                self.get_logger().error("Cannot start: model is not loaded")
                return
            self.mission.start(self.now_s())
            self.coverage_controller.reset()
            self.coverage_command = None
            self.mission_waypoint = None
            self.had_visible_target = False
            self.target_lost_started_at = None
            self.target_visibility_history.clear()
            self.motion_paused = False
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(open_gripper=False)
            self.active = True
            self.begin_leave_start()
            self.get_logger().info("RL model policy started")
        elif command in ("pause", "pause_motion"):
            self.set_motion_paused(True)
        elif command in ("resume", "resume_motion"):
            self.set_motion_paused(False)
        elif command == "stop":
            self.active = False
            self.mission.stop(self.now_s())
            self.motion_paused = False
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
            self.cancel_leave_start()
            self.set_control_mode(self.MODE_IDLE)
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("RL model policy stopped")
        elif command == "reset":
            self.active = False
            self.mission.reset()
            self.latest_target = None
            self.latest_target_label = None
            self.latest_target_label_time = None
            self.latest_avoid = None
            self.latest_avoid_objects = []
            self.last_target_world_bearing = None
            self.time_since_target_seen = self.get_float("episode_length_s")
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_controller.reset()
            self.coverage_command = None
            self.had_visible_target = False
            self.target_lost_started_at = None
            self.target_visibility_history.clear()
            self.motion_paused = False
            self.pickup_label = None
            self.mission_waypoint = None
            self.cancel_leave_start()
            self.set_control_mode(self.MODE_IDLE)
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(open_gripper=False)
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("RL model policy reset")
        elif command in ("return", "return_storage"):
            if self.mission.begin_return(ReturnReason.MANUAL, self.now_s()):
                self.prepare_storage_return()
                self.get_logger().info("Manual return to storage requested")
            else:
                self.get_logger().info(
                    "Ignoring storage return request because the robot is empty"
                )
        elif command == "open":
            self.command_gripper(open_gripper=True)
        elif command == "close":
            self.command_gripper(open_gripper=False)

    def tick(self):
        now_s = self.now_s()
        target = self.current_target()
        objects = self.current_avoid_objects(target)
        obs, bins, nearest = self.make_observation(target, objects)
        if self.active and self.full_mission_is_enabled():
            previous_phase = self.mission.phase
            self.mission.update_time(now_s)
            if (
                self.mission.is_storage_phase()
                and previous_phase not in MissionPhase.STORAGE_PHASES
            ):
                self.prepare_storage_return()
            if self.mission.phase == MissionPhase.TIMEOUT:
                self.active = False
                self.set_control_mode(self.MODE_MISSION_TIMEOUT)
                self.get_logger().warning("Mission time expired; stopping the robot")

        collecting = (
            not self.full_mission_is_enabled()
            or self.mission.phase == MissionPhase.COLLECTING
        )
        if collecting and not self.leave_start_active:
            self.update_target_history(target)
        tracking_target = self.control_mode == self.MODE_TRACK_TARGET
        target_control_ready = (
            target is not None
            and self.target_activation_is_met(tracking_active=tracking_target)
            and (
                tracking_target
                or self.target_confirmation_is_met()
            )
        )
        confirmed_target = target if target_control_ready else None
        grab_cmd = (
            self.update_grab_sequence(confirmed_target)
            if (
                self.active
                and collecting
                and not self.motion_paused
                and not self.coverage_controller.rejoin_active
                and not self.leave_start_active
            )
            else None
        )
        if self.active and self.motion_paused:
            if self.grab_state != self.GRAB_TRACKING:
                self.set_control_mode(self.MODE_GRAB_SEQUENCE)
            linear_x, angular_z = (0.0, 0.0)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif self.active and self.mission.is_storage_phase():
            linear_x, angular_z = self.storage_mission_command(bins, now_s)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif self.active and self.leave_start_active:
            linear_x, angular_z = self.leave_start_command()
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif grab_cmd is not None:
            self.set_control_mode(self.MODE_GRAB_SEQUENCE)
            linear_x, angular_z = grab_cmd
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif (
            self.active
            and collecting
            and self.coverage_controller.rejoin_active
        ):
            linear_x, angular_z = self.coverage_search_command(bins)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        elif (
            self.active
            and confirmed_target is not None
            and self.policy is not None
        ):
            self.set_control_mode(self.MODE_TRACK_TARGET)
            self.raw_action = self.infer_action(obs)
            self.filtered_action = self.filter_action(self.filtered_action, self.raw_action)
            linear_x, angular_z = self.action_to_cmd(self.filtered_action)
            self.coverage_command = None
        elif self.active and self.should_locally_reacquire():
            self.set_control_mode(self.MODE_LOCAL_REACQUIRE)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
            linear_x = 0.0
            angular_z = reacquire_angular_velocity(
                last_target_direction=self.last_target_direction,
                elapsed_s=self.target_lost_age_s(),
                reverse_after_s=self.get_float(
                    "coverage_reacquire_reverse_after_s"
                ),
                angular_speed=self.get_float("coverage_reacquire_angular_z"),
            )
        elif self.active and self.coverage_is_enabled():
            linear_x, angular_z = self.coverage_search_command(bins)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        elif self.active and self.policy is not None:
            self.set_control_mode(self.MODE_TRACK_TARGET)
            self.raw_action = self.infer_action(obs)
            self.filtered_action = self.filter_action(self.filtered_action, self.raw_action)
            linear_x, angular_z = self.action_to_cmd(self.filtered_action)
            self.coverage_command = None
        else:
            if self.mission.phase == MissionPhase.COMPLETE:
                self.set_control_mode(self.MODE_MISSION_COMPLETE)
            elif self.mission.phase == MissionPhase.TIMEOUT:
                self.set_control_mode(self.MODE_MISSION_TIMEOUT)
            else:
                self.set_control_mode(self.MODE_IDLE)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = self.filter_action(self.filtered_action, self.raw_action)
            linear_x, angular_z = (0.0, 0.0)
            self.coverage_command = None

        planned_linear_x = linear_x
        planned_angular_z = angular_z
        if self.motion_paused:
            linear_x, angular_z = (0.0, 0.0)

        if self.active or bool(self.get_parameter("publish_stop_when_inactive").value):
            self.publish_cmd(linear_x, angular_z)
        self.publish_state(
            obs,
            bins,
            nearest,
            linear_x,
            angular_z,
            planned_linear_x,
            planned_angular_z,
        )

    def set_motion_paused(self, paused):
        paused = bool(paused)
        if paused == self.motion_paused:
            return
        if paused:
            self.grab_state_elapsed_offset_s = self.grab_state_age_s()
            self.motion_paused = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                "Base motion paused; perception and policy updates remain active"
            )
            return

        self.motion_paused = False
        self.grab_state_started_at = self.get_clock().now()
        self.get_logger().info("Base motion resumed")

    def prepare_storage_return(self):
        self.cancel_leave_start()
        self.coverage_controller.cancel_rejoin()
        return_min_x = min(
            self.get_float("coverage_min_x"),
            self.get_float("storage_staging_x"),
        )
        return_max_x = max(
            self.get_float("coverage_max_x"),
            self.get_float("storage_staging_x"),
        )
        self.return_lane_x = self.clamp(
            self.robot_x,
            return_min_x,
            return_max_x,
        )
        self.mission_waypoint = (
            self.return_lane_x,
            self.get_float("storage_main_road_y"),
        )
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_center_y = None
        self.latest_target_center_y_time = None
        self.target_visibility_history.clear()
        self.had_visible_target = False
        self.target_lost_started_at = None
        self.filtered_action = [0.0, 0.0]
        self.get_logger().info(
            "Returning to storage: "
            f"reason={self.mission.return_reason}, "
            f"onboard={self.mission.onboard_count}, "
            f"delivered={self.mission.delivered_count}"
        )

    def storage_mission_command(self, bins, now_s):
        if not self.storage_pose_is_valid():
            self.mission_waypoint = None
            self.set_control_mode(self.MODE_WAITING_FOR_POSE)
            return (0.0, 0.0)

        phase = self.mission.phase
        entry_yaw = math.radians(self.get_float("storage_entry_yaw_deg"))

        if phase == MissionPhase.RETURN_MAIN_ROAD:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                self.return_lane_x,
                self.get_float("storage_main_road_y"),
                self.get_float("storage_return_speed"),
            )
            if command.reached:
                self.mission.set_phase(MissionPhase.RETURN_STAGING, now_s)
                self.mission_waypoint = (
                    self.get_float("storage_staging_x"),
                    self.get_float("storage_staging_y"),
                )
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.RETURN_STAGING:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                self.get_float("storage_staging_x"),
                self.get_float("storage_staging_y"),
                self.get_float("storage_return_speed"),
                final_yaw=entry_yaw,
            )
            if command.reached:
                self.mission.set_phase(MissionPhase.ENTER_STORAGE, now_s)
                self.mission_waypoint = (
                    self.get_float("storage_center_x"),
                    self.get_float("storage_center_y"),
                )
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.ENTER_STORAGE:
            self.set_control_mode(self.MODE_ENTER_STORAGE)
            command = self.storage_waypoint_command(
                (0.0, 0.0, 0.0),
                self.get_float("storage_center_x"),
                self.get_float("storage_center_y"),
                self.get_float("storage_entry_speed"),
                final_yaw=entry_yaw,
                waypoint_tolerance=self.get_float(
                    "storage_entry_tolerance"
                ),
            )
            if command.reached:
                self.publish_cmd(0.0, 0.0)
                self.command_gripper(open_gripper=True)
                self.mission.set_phase(MissionPhase.DEPOSIT, now_s)
                self.mission_waypoint = None
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.DEPOSIT:
            self.set_control_mode(self.MODE_DEPOSIT)
            self.mission_waypoint = None
            if self.mission.phase_age_s(now_s) >= self.get_float(
                "gripper_move_duration_s"
            ):
                deposited_count = self.mission.onboard_count
                self.mission.record_deposit(now_s)
                self.get_logger().info(
                    f"Deposited {deposited_count} object(s); "
                    f"total delivered={self.mission.delivered_count}"
                )
            return (0.0, 0.0)

        if phase == MissionPhase.EXIT_STORAGE:
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            self.mission_waypoint = (
                self.get_float("storage_staging_x"),
                self.get_float("storage_exit_y"),
            )
            command = reverse_exit_command(
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                exit_y=self.get_float("storage_exit_y"),
                desired_yaw=entry_yaw,
                reverse_speed=self.get_float("storage_exit_reverse_speed"),
                y_tolerance=self.get_float("storage_waypoint_tolerance"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_max_angular_speed"),
            )
            if command.reached:
                self.publish_cmd(0.0, 0.0)
                self.command_gripper(open_gripper=False)
                self.mission.set_phase(MissionPhase.CLOSE_AFTER_DEPOSIT, now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CLOSE_AFTER_DEPOSIT:
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            if self.mission.phase_age_s(now_s) < self.get_float(
                "gripper_move_duration_s"
            ):
                return (0.0, 0.0)
            next_phase = self.mission.finish_storage_exit(now_s)
            self.mission_waypoint = None
            if next_phase == MissionPhase.COMPLETE:
                self.active = False
                self.set_control_mode(self.MODE_MISSION_COMPLETE)
                self.get_logger().info(
                    f"Mission complete: {self.mission.delivered_count} objects delivered"
                )
                return (0.0, 0.0)
            self.resume_collection_after_storage()
            return (0.0, 0.0)

        return (0.0, 0.0)

    def storage_waypoint_command(
        self,
        bins,
        target_x,
        target_y,
        speed,
        final_yaw=None,
        waypoint_tolerance=None,
    ):
        self.mission_waypoint = (float(target_x), float(target_y))
        if float(bins[1]) >= self.get_float("storage_avoid_danger_threshold"):
            direction = 1.0 if float(bins[0]) <= float(bins[2]) else -1.0
            return SimpleNamespace(
                linear_x=0.0,
                angular_z=direction
                * self.get_float("coverage_avoid_angular_speed"),
                reached=False,
            )
        return waypoint_command(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            target_x=target_x,
            target_y=target_y,
            speed=speed,
            waypoint_tolerance=(
                self.get_float("storage_waypoint_tolerance")
                if waypoint_tolerance is None
                else float(waypoint_tolerance)
            ),
            heading_tolerance=self.get_float("storage_heading_tolerance"),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float("storage_max_angular_speed"),
            final_yaw=final_yaw,
            final_yaw_tolerance=self.get_float(
                "storage_final_yaw_tolerance"
            ),
        )

    def storage_pose_is_valid(self):
        return self.is_fresh(
            self.latest_pose_time,
            "pose_timeout_s",
        ) and pose_is_usable(
            self.robot_x,
            self.robot_y,
            self.get_float("arena_half_extent_m"),
            self.get_float("pose_bounds_tolerance_m"),
        )

    def resume_collection_after_storage(self):
        self.coverage_controller.cancel_avoidance()
        self.coverage_command = None
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_label = None
        self.latest_target_label_time = None
        self.had_visible_target = False
        self.target_lost_started_at = None
        self.change_grab_state(self.GRAB_TRACKING)
        self.coverage_controller.begin_rejoin(self.robot_y)
        self.set_control_mode(self.MODE_COVERAGE_SEARCH)
        self.get_logger().info(
            "Storage exit complete; resuming collection with "
            f"{self.mission.delivered_count}/{self.mission.target_object_count} delivered"
        )

    def create_coverage_controller(self):
        legs = generate_coverage_legs(
            min_x=self.get_float("coverage_min_x"),
            max_x=self.get_float("coverage_max_x"),
            main_road_y=self.get_float("coverage_main_road_y"),
            scan_end_y=self.get_float("coverage_scan_end_y"),
            lane_spacing=self.get_float("coverage_lane_spacing"),
            scan_speed=self.get_float("coverage_scan_speed"),
            transit_speed=self.get_float("coverage_transit_speed"),
            return_speed=self.get_float("coverage_return_speed"),
        )
        controller = CoverageController(
            legs=legs,
            waypoint_tolerance=self.get_float("coverage_waypoint_tolerance"),
            heading_tolerance=self.get_float("coverage_heading_tolerance"),
            heading_gain=self.get_float("coverage_heading_gain"),
            max_angular_speed=self.get_float("coverage_max_angular_speed"),
            turn_in_place_threshold=self.get_float(
                "coverage_turn_in_place_threshold"
            ),
            avoid_danger_threshold=self.get_float(
                "coverage_avoid_danger_threshold"
            ),
            avoid_angular_speed=self.get_float("coverage_avoid_angular_speed"),
            avoid_linear_scale=self.get_float(
                "coverage_avoid_linear_scale"
            ),
            rejoin_speed=self.get_float("coverage_rejoin_speed"),
        )
        scan_lane_count = sum(
            leg.phase.startswith("SCAN_LANE") for leg in legs
        )
        self.get_logger().info(
            f"Coverage search ready with {len(legs)} legs "
            f"({scan_lane_count} scan lanes)"
        )
        return controller

    def update_target_history(self, target):
        if target is not None:
            self.had_visible_target = True
            self.target_lost_started_at = None
            return
        if self.had_visible_target and self.target_lost_started_at is None:
            self.target_lost_started_at = self.get_clock().now()

    def should_locally_reacquire(self):
        if not self.had_visible_target or self.target_lost_started_at is None:
            return False
        elapsed_s = self.target_lost_age_s()
        if elapsed_s <= self.get_float("coverage_reacquire_duration_s"):
            return True
        self.had_visible_target = False
        self.target_lost_started_at = None
        return False

    def target_confirmation_is_met(self):
        return target_is_confirmed(
            self.target_visibility_history,
            self.target_confirmation_window,
            self.target_confirmation_min_detections,
        )

    def current_target_center_y(self):
        center_y = None
        if self.is_fresh(self.latest_target_center_y_time, "target_timeout_s"):
            center_y = self.latest_target_center_y
        return center_y

    def target_activation_is_met(self, tracking_active=None):
        if tracking_active is None:
            tracking_active = self.control_mode == self.MODE_TRACK_TARGET
        return target_is_eligible(
            self.current_target_center_y(),
            self.get_float("target_activation_center_y_min"),
            self.get_float("target_tracking_center_y_min"),
            tracking_active,
        )

    def begin_leave_start(self):
        self.leave_start_origin = None
        self.leave_start_yaw = None
        self.leave_start_traveled_m = 0.0
        self.leave_start_active = bool(
            self.get_parameter("leave_start_enabled").value
        )
        self.set_control_mode(
            self.MODE_LEAVE_START
            if self.leave_start_active
            else self.MODE_COVERAGE_SEARCH
        )

    def cancel_leave_start(self):
        self.leave_start_active = False
        self.leave_start_origin = None
        self.leave_start_yaw = None
        self.leave_start_traveled_m = 0.0

    def leave_start_command(self):
        if not self.storage_pose_is_valid():
            self.set_control_mode(self.MODE_WAITING_FOR_POSE)
            return (0.0, 0.0)

        if self.leave_start_origin is None:
            self.leave_start_origin = (self.robot_x, self.robot_y)
            self.leave_start_yaw = self.robot_yaw
            self.get_logger().info(
                "Leaving start zone by driving straight from the current heading"
            )

        command = make_leave_start_command(
            origin_x=self.leave_start_origin[0],
            origin_y=self.leave_start_origin[1],
            desired_yaw=self.leave_start_yaw,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            distance_m=self.get_float("leave_start_distance_m"),
            linear_speed=self.get_float("leave_start_speed"),
            heading_gain=self.get_float("leave_start_heading_gain"),
            max_angular_speed=self.get_float("leave_start_max_angular_speed"),
        )
        self.leave_start_traveled_m = command.traveled_m
        if command.complete:
            self.leave_start_active = False
            self.set_control_mode(self.MODE_COVERAGE_SEARCH)
            self.get_logger().info("Start zone exit complete; beginning coverage search")
        else:
            self.set_control_mode(self.MODE_LEAVE_START)
        return (command.linear_x, command.angular_z)

    def target_lost_age_s(self):
        if self.target_lost_started_at is None:
            return 0.0
        elapsed = self.get_clock().now() - self.target_lost_started_at
        return max(0.0, elapsed.nanoseconds / 1_000_000_000.0)

    def coverage_search_command(self, bins):
        pose_fresh = self.is_fresh(self.latest_pose_time, "pose_timeout_s")
        pose_valid = pose_fresh and pose_is_usable(
            self.robot_x,
            self.robot_y,
            self.get_float("arena_half_extent_m"),
            self.get_float("pose_bounds_tolerance_m"),
        )
        if not pose_valid:
            self.coverage_command = None
            self.set_control_mode(self.MODE_WAITING_FOR_POSE)
            return (0.0, 0.0)

        self.coverage_command = self.coverage_controller.command(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            avoid_left=bins[0],
            avoid_center=bins[1],
            avoid_right=bins[2],
        )
        self.set_control_mode(self.MODE_COVERAGE_SEARCH)
        return (
            self.coverage_command.linear_x,
            self.coverage_command.angular_z,
        )

    def set_control_mode(self, mode):
        if mode == self.control_mode:
            return
        previous_mode = self.control_mode
        self.control_mode = mode
        if mode == self.MODE_TRACK_TARGET:
            self.filtered_action = [0.0, 0.0]
        self.get_logger().info(f"Control mode: {previous_mode} -> {mode}")

    def current_target(self):
        if not self.is_fresh(self.latest_target_time, "target_timeout_s"):
            return None

        target = self.make_point(
            self.latest_target.x,
            self.latest_target.y,
            self.latest_target.z,
        )
        if (
            bool(self.get_parameter("target_bearing_prediction_enabled").value)
            and self.last_target_world_bearing is not None
            and self.is_fresh(self.latest_pose_time, "pose_timeout_s")
        ):
            fov_rad = math.radians(self.get_float("camera_horizontal_fov_deg"))
            target.x = estimate_target_image_x(
                self.robot_yaw,
                self.last_target_world_bearing,
                fov_rad,
            )
            if abs(target.x) > 1e-6:
                self.last_target_direction = 1.0 if target.x >= 0.0 else -1.0
        return target

    def make_observation(self, target, objects):
        visible = target is not None
        if visible:
            self.time_since_target_seen = 0.0
            target_x = target.x
            target_y = target.y
        else:
            dt = 1.0 / max(self.get_float("timer_rate_hz"), 1e-6)
            self.time_since_target_seen = min(
                self.get_float("episode_length_s"),
                self.time_since_target_seen + dt,
            )
            target_x = 0.0
            target_y = 0.0

        left, center, right, nearest = self.avoid_bins(objects)
        nearest_x = 0.0 if nearest is None else nearest.x
        nearest_y = 0.0 if nearest is None else nearest.y
        time_norm = self.clamp(
            self.time_since_target_seen / max(self.get_float("episode_length_s"), 1e-6),
            0.0,
            1.0,
        )

        pose_enabled = self.pose_observation_is_active()
        pose_fresh = pose_enabled and self.is_fresh(
            self.latest_pose_time,
            "pose_timeout_s",
        )
        pose_valid = pose_fresh and pose_is_usable(
            self.robot_x,
            self.robot_y,
            self.get_float("arena_half_extent_m"),
            self.get_float("pose_bounds_tolerance_m"),
        )
        pose_obs = make_pose_observation(
            pose_valid=pose_valid,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            yaw=self.robot_yaw,
            yaw_rate=self.robot_yaw_rate,
            last_target_world_bearing=self.last_target_world_bearing,
            arena_half_extent_m=self.get_float("arena_half_extent_m"),
            max_angular_speed=self.get_float("max_angular_speed"),
        )
        obs = [
            1.0 if visible else 0.0,
            target_x if visible else 0.0,
            target_y if visible else 0.0,
            time_norm,
            self.last_target_direction,
            left,
            center,
            right,
            nearest_x,
            nearest_y,
            *pose_obs,
        ]
        return validate_observation(obs), (left, center, right), nearest

    def infer_action(self, obs):
        validate_observation(obs)
        model_obs = obs[:self.model_observation_dim]
        with torch.no_grad():
            obs_tensor = torch.tensor(model_obs, dtype=torch.float32).unsqueeze(0)
            scaled = (obs_tensor - self.obs_mean) / torch.sqrt(
                self.obs_variance + self.get_float("state_preprocessor_epsilon")
            )
            action = self.policy(scaled).squeeze(0)
            action = torch.clamp(action, -1.0, 1.0)
        return [float(action[0].item()), float(action[1].item())]

    def filter_action(self, current, target):
        alpha = self.get_float("action_filter_alpha")
        max_delta = [self.get_float("max_linear_action_delta"), self.get_float("max_angular_action_delta")]
        out = []
        for old, new, limit in zip(current, target, max_delta):
            delta = self.clamp(new - old, -limit, limit)
            out.append(self.clamp(old + alpha * delta, -1.0, 1.0))
        return out

    def action_to_cmd(self, action):
        linear_action = action[0]
        angular_action = action[1]
        if linear_action >= 0.0:
            linear_x = linear_action * self.get_float("max_forward_speed")
        else:
            linear_x = linear_action * self.get_float("max_reverse_speed")
        angular_z = angular_action * self.get_float("max_angular_speed")
        scale = self.get_float("speed_scale")
        return linear_x * scale, angular_z * scale

    def update_grab_sequence(self, target):
        if not bool(self.get_parameter("gripper_enabled").value):
            return None

        if self.grab_state == self.GRAB_TRACKING:
            if target is None:
                return None
            if not self.is_fresh(
                self.latest_target_time,
                "grab_detection_timeout_s",
            ):
                return None
            if not pickup_is_ready(
                target_x=target.x,
                target_y=target.y,
                center_tolerance=self.get_float("grab_center_tolerance"),
                grab_area_ratio=self.get_float("grab_area_ratio"),
            ):
                return None

            self.pickup_label = self.current_target_label() or "unknown"
            self.publish_cmd(0.0, 0.0)
            self.command_gripper(open_gripper=True)
            self.change_grab_state(self.GRAB_OPENING)
            return (0.0, 0.0)

        if self.grab_state == self.GRAB_OPENING:
            fresh_label = self.current_target_label()
            if fresh_label is not None:
                self.pickup_label = fresh_label
            if self.grab_state_age_s() < self.get_float("gripper_move_duration_s"):
                return (0.0, 0.0)
            self.change_grab_state(self.GRAB_FINAL_FORWARD)

        if self.grab_state == self.GRAB_FINAL_FORWARD:
            if self.grab_state_age_s() < self.get_float("final_forward_duration_s"):
                return (self.get_float("final_forward_linear_x"), 0.0)
            self.command_gripper(open_gripper=False)
            self.change_grab_state(self.GRAB_CLOSING)
            return (0.0, 0.0)

        if self.grab_state == self.GRAB_CLOSING:
            if self.grab_state_age_s() < self.get_float("grab_duration_s"):
                return (0.0, 0.0)
            label = self.pickup_label or "unknown"
            if self.full_mission_is_enabled():
                return_reason = self.mission.record_pickup(label, self.now_s())
                if return_reason is not None:
                    self.prepare_storage_return()
                else:
                    self.clear_target_after_pickup()
                    self.coverage_controller.begin_rejoin(self.robot_y)
            else:
                self.mission.onboard_objects.append(label)
                self.clear_target_after_pickup()
                self.coverage_controller.begin_rejoin(self.robot_y)
            self.pickup_label = None
            self.change_grab_state(self.GRAB_COMPLETE)
            if (
                not self.full_mission_is_enabled()
                and bool(self.get_parameter("stop_after_grab").value)
            ):
                self.active = False
                self.get_logger().info("Object grabbed; RL drive stopped")
            else:
                self.change_grab_state(self.GRAB_TRACKING)
            return (0.0, 0.0)

        if self.grab_state == self.GRAB_COMPLETE:
            return (0.0, 0.0)

        return None

    def clear_target_after_pickup(self):
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_label = None
        self.latest_target_label_time = None
        self.latest_target_center_y = None
        self.latest_target_center_y_time = None
        self.target_visibility_history.clear()
        self.had_visible_target = False
        self.target_lost_started_at = None

    def current_avoid_objects(self, target):
        if self.is_fresh(self.latest_avoid_objects_time, "avoid_timeout_s"):
            objects = list(self.latest_avoid_objects)
        elif self.is_fresh(self.latest_avoid_time, "avoid_timeout_s") and self.latest_avoid is not None:
            objects = [self.latest_avoid]
        else:
            objects = []

        if bool(self.get_parameter("avoid_only_if_closer_than_target").value) and target is not None:
            threshold = target.y * self.get_float("avoid_closer_ratio")
            objects = [obj for obj in objects if obj.y >= threshold]
        return objects

    def avoid_bins(self, objects):
        left = 0.0
        center = 0.0
        right = 0.0
        nearest = None
        center_band = max(self.get_float("avoid_center_band"), 1e-6)
        center_corridor = self.get_float("avoid_center_corridor")
        center_weight = self.get_float("avoid_vfh_center_weight")
        avoid_area = self.get_float("avoid_area_ratio")

        for obj in objects:
            if nearest is None or obj.y > nearest.y:
                nearest = obj
            if obj.y < avoid_area:
                continue

            centered = self.clamp(1.0 - abs(obj.x) / center_band, 0.0, 1.0)
            danger = self.clamp(obj.y * obj.y * (1.0 + center_weight * centered), 0.0, 1.0)
            if obj.x < 0.0:
                left = max(left, danger)
            if obj.x > 0.0:
                right = max(right, danger)
            if abs(obj.x) <= center_corridor:
                center = max(center, danger)

        return left, center, right, nearest

    def publish_cmd(self, linear_x, angular_z):
        self.last_cmd = (float(linear_x), float(angular_z))
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def command_gripper(self, open_gripper):
        action = "open" if open_gripper else "close"
        if not bool(self.get_parameter("gripper_enabled").value):
            self.get_logger().info(f"Gripper {action} skipped because gripper_enabled is false")
            return
        if self.dry_run:
            self.get_logger().info(f"Gripper {action} skipped in dry-run mode")
            return

        position = int(
            self.get_parameter("gripper_open_position").value
            if open_gripper
            else self.get_parameter("gripper_closed_position").value
        )
        gripper_type = str(self.get_parameter("gripper_type").value).strip().lower()
        if gripper_type == "bus":
            state = BusServoState()
            state.present_id = [1, int(self.get_parameter("gripper_servo_id").value)]
            state.position = [1, position]

            msg = SetBusServoState()
            msg.duration = self.get_float("gripper_move_duration_s")
            msg.state = [state]
            self.bus_servo_pub.publish(msg)
        elif gripper_type == "pwm":
            state = PWMServoState()
            state.id = [int(self.get_parameter("gripper_servo_id").value)]
            state.position = [position]

            msg = SetPWMServoState()
            msg.duration = self.get_float("gripper_move_duration_s")
            msg.state = [state]
            self.pwm_servo_pub.publish(msg)
        else:
            self.get_logger().warning(f"Unknown gripper_type: {gripper_type}")
            return

        self.get_logger().info(f"Gripper command: {action} position={position}")

    def change_grab_state(self, state):
        if self.grab_state == state:
            self.grab_state_started_at = self.get_clock().now()
            self.grab_state_elapsed_offset_s = 0.0
            return
        self.grab_state = state
        self.grab_state_started_at = self.get_clock().now()
        self.grab_state_elapsed_offset_s = 0.0
        self.get_logger().info(f"Grab state -> {state}")

    def grab_state_age_s(self):
        elapsed = self.get_clock().now() - self.grab_state_started_at
        return (
            self.grab_state_elapsed_offset_s
            + elapsed.nanoseconds / 1_000_000_000.0
        )

    def current_target_label(self):
        if not self.is_fresh(self.latest_target_label_time, "target_timeout_s"):
            return None
        return self.latest_target_label

    def publish_state(
        self,
        obs,
        bins,
        nearest,
        linear_x,
        angular_z,
        planned_linear_x,
        planned_angular_z,
    ):
        pose_fresh = self.is_fresh(self.latest_pose_time, "pose_timeout_s")
        coverage = self.coverage_command
        now_s = self.now_s()
        mission_waypoint = self.mission_waypoint
        msg = String()
        msg.data = json.dumps(
            {
                "active": self.active,
                "motion_paused": self.motion_paused,
                "dry_run": self.dry_run,
                "model_loaded": self.policy is not None,
                "control_mode": self.control_mode,
                "observation_dim": self.model_observation_dim,
                "observation_names": OBSERVATION_NAMES[:self.model_observation_dim],
                "pose_observation_enabled": self.pose_observation_is_active(),
                "pose_observation_requested": bool(
                    self.get_parameter("pose_observation_enabled").value
                ),
                "grab_state": self.grab_state,
                "gripper_enabled": bool(self.get_parameter("gripper_enabled").value),
                "target_label": self.current_target_label(),
                "target_confirmation": {
                    "detections": sum(self.target_visibility_history),
                    "frames": len(self.target_visibility_history),
                    "window": self.target_confirmation_window,
                    "required": self.target_confirmation_min_detections,
                    "confirmed": self.target_confirmation_is_met(),
                },
                "target_activation": {
                    "center_y": self.current_target_center_y(),
                    "minimum_center_y": self.get_float(
                        "target_tracking_center_y_min"
                        if self.control_mode == self.MODE_TRACK_TARGET
                        else "target_activation_center_y_min"
                    ),
                    "eligible": self.target_activation_is_met(),
                    "entry_minimum_center_y": self.get_float(
                        "target_activation_center_y_min"
                    ),
                    "tracking_minimum_center_y": self.get_float(
                        "target_tracking_center_y_min"
                    ),
                    "tracking_hysteresis_active": (
                        self.control_mode == self.MODE_TRACK_TARGET
                    ),
                },
                "leave_start": {
                    "active": self.leave_start_active,
                    "distance_m": self.get_float("leave_start_distance_m"),
                    "traveled_m": round(self.leave_start_traveled_m, 3),
                },
                "pickup_label": self.pickup_label,
                "stored_objects": list(self.mission.onboard_objects),
                "delivered_objects": list(self.mission.delivered_objects),
                "mission_phase": self.mission.phase,
                "mission": {
                    "enabled": self.full_mission_is_enabled(),
                    "phase": self.mission.phase,
                    "return_reason": self.mission.return_reason,
                    "elapsed_s": round(self.mission.elapsed_s(now_s), 2),
                    "remaining_s": round(self.mission.remaining_s(now_s), 2),
                    "duration_s": self.mission.mission_duration_s,
                    "force_return_remaining_s": (
                        self.mission.force_return_remaining_s
                    ),
                    "storage_capacity": self.mission.storage_capacity,
                    "target_object_count": self.mission.target_object_count,
                    "onboard_count": self.mission.onboard_count,
                    "delivered_count": self.mission.delivered_count,
                    "total_collected_count": (
                        self.mission.total_collected_count
                    ),
                    "waypoint": None
                    if mission_waypoint is None
                    else {
                        "x": round(float(mission_waypoint[0]), 4),
                        "y": round(float(mission_waypoint[1]), 4),
                    },
                },
                "obs": [round(v, 4) for v in obs[:self.model_observation_dim]],
                "raw_action": [round(v, 4) for v in self.raw_action],
                "filtered_action": [round(v, 4) for v in self.filtered_action],
                "linear_x": round(float(linear_x), 4),
                "angular_z": round(float(angular_z), 4),
                "planned_linear_x": round(float(planned_linear_x), 4),
                "planned_angular_z": round(float(planned_angular_z), 4),
                "avoid_left": round(bins[0], 4),
                "avoid_center": round(bins[1], 4),
                "avoid_right": round(bins[2], 4),
                "nearest_avoid_x": None if nearest is None else round(nearest.x, 4),
                "nearest_avoid_y": None if nearest is None else round(nearest.y, 4),
                "pose_fresh": pose_fresh,
                "pose": None
                if not pose_fresh
                else {
                    "x": round(self.robot_x, 4),
                    "y": round(self.robot_y, 4),
                    "yaw": round(self.robot_yaw, 4),
                },
                "coverage": {
                    "enabled": self.coverage_is_enabled(),
                    "phase": None if coverage is None else coverage.phase,
                    "leg_index": None if coverage is None else coverage.leg_index,
                    "leg_count": len(self.coverage_controller.legs),
                    "waypoint_x": None
                    if coverage is None
                    else round(coverage.waypoint_x, 4),
                    "waypoint_y": None
                    if coverage is None
                    else round(coverage.waypoint_y, 4),
                    "cycle_count": self.coverage_controller.cycle_count,
                },
            },
            ensure_ascii=True,
        )
        self.state_pub.publish(msg)

    def coverage_is_enabled(self):
        return bool(self.get_parameter("coverage_enabled").value)

    def full_mission_is_enabled(self):
        return bool(self.get_parameter("full_mission_enabled").value)

    def pose_observation_is_active(self):
        return model_uses_pose_observation(
            self.model_observation_dim,
            self.get_parameter("pose_observation_enabled").value,
        )

    def is_fresh(self, stamp, timeout_param):
        if stamp is None:
            return False
        age = self.get_clock().now() - stamp
        return age.nanoseconds / 1_000_000_000.0 <= self.get_float(timeout_param)

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def make_point(x, y, z):
        return SimpleNamespace(
            x=max(-1.0, min(1.0, float(x))),
            y=max(0.0, min(1.0, float(y))),
            z=max(0.0, float(z)),
        )

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = RLModelPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
