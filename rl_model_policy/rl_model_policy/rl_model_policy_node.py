import json
import math
from pathlib import Path
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from std_msgs.msg import String

from rl_model_policy.observation import (
    OBSERVATION_DIM,
    OBSERVATION_NAMES,
    estimate_target_world_bearing,
    make_pose_observation,
    pose_is_usable,
    quaternion_to_yaw,
    validate_observation,
)

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
        def __init__(self):
            super().__init__()
            self.log_std_parameter = torch.nn.Parameter(torch.zeros(2))
            self.net_container = torch.nn.Sequential(
                torch.nn.Linear(OBSERVATION_DIM, 128),
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
        self.declare_parameter("target_timeout_s", 0.5)
        self.declare_parameter("avoid_timeout_s", 0.5)
        self.declare_parameter("episode_length_s", 18.0)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("arena_half_extent_m", 2.0)
        self.declare_parameter("pose_bounds_tolerance_m", 0.25)
        self.declare_parameter("camera_horizontal_fov_deg", 90.0)

        self.declare_parameter("avoid_area_ratio", 0.20)
        self.declare_parameter("avoid_center_band", 0.75)
        self.declare_parameter("avoid_center_corridor", 0.30)
        self.declare_parameter("avoid_vfh_center_weight", 2.0)
        self.declare_parameter("avoid_only_if_closer_than_target", False)
        self.declare_parameter("avoid_closer_ratio", 0.85)

        self.declare_parameter("max_forward_speed", 0.20)
        self.declare_parameter("max_reverse_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("speed_scale", 0.50)
        self.declare_parameter("max_linear_action_delta", 0.25)
        self.declare_parameter("max_angular_action_delta", 0.16)
        self.declare_parameter("action_filter_alpha", 0.60)
        self.declare_parameter("publish_stop_when_inactive", True)
        self.declare_parameter("state_preprocessor_epsilon", 1e-8)

        self.declare_parameter("gripper_enabled", True)
        self.declare_parameter("gripper_type", "bus")
        self.declare_parameter("gripper_servo_id", 1)
        self.declare_parameter("gripper_open_position", 1000)
        self.declare_parameter("gripper_closed_position", 250)
        self.declare_parameter("gripper_move_duration_s", 0.5)
        self.declare_parameter("grab_center_tolerance", 0.12)
        self.declare_parameter("grab_area_ratio", 0.50)
        self.declare_parameter("final_forward_linear_x", 0.06)
        self.declare_parameter("final_forward_duration_s", 1.6)
        self.declare_parameter("grab_duration_s", 1.0)
        self.declare_parameter("stop_after_grab", True)

        self.active = bool(self.get_parameter("active_on_start").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.latest_target = None
        self.latest_target_time = None
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

        self.policy = None
        self.obs_mean = None
        self.obs_variance = None
        self.model_path = None
        self.load_model()

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
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self.odometry_callback,
            10,
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
            f"Send start/stop on {self.get_parameter('control_topic').value}; publishing cmd_vel on "
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
        if first_weight is None or tuple(first_weight.shape) != (128, OBSERVATION_DIM):
            shape = None if first_weight is None else tuple(first_weight.shape)
            raise RuntimeError(
                "RL checkpoint observation contract mismatch: expected policy "
                f"net_container.0.weight shape (128, {OBSERVATION_DIM}), got {shape}"
            )

        preprocessor = checkpoint.get("state_preprocessor", {})
        obs_mean = preprocessor.get("running_mean")
        obs_variance = preprocessor.get("running_variance")
        if obs_mean is None or tuple(obs_mean.shape) != (OBSERVATION_DIM,):
            raise RuntimeError(
                f"Checkpoint running_mean must have shape ({OBSERVATION_DIM},)"
            )
        if obs_variance is None or tuple(obs_variance.shape) != (OBSERVATION_DIM,):
            raise RuntimeError(
                f"Checkpoint running_variance must have shape ({OBSERVATION_DIM},)"
            )

        self.policy = PolicyNetwork()
        self.policy.load_state_dict(policy_state, strict=True)
        self.policy.eval()

        self.obs_mean = obs_mean.float()
        self.obs_variance = obs_variance.float()
        self.model_path = str(model_path)
        self.get_logger().info(
            f"Loaded {OBSERVATION_DIM}-observation RL checkpoint: {self.model_path}"
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
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(open_gripper=False)
            self.active = True
            self.get_logger().info("RL model policy started")
        elif command in ("stop", "pause"):
            self.active = False
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("RL model policy stopped")
        elif command == "reset":
            self.active = False
            self.latest_target = None
            self.latest_avoid = None
            self.latest_avoid_objects = []
            self.last_target_world_bearing = None
            self.time_since_target_seen = self.get_float("episode_length_s")
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(open_gripper=False)
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("RL model policy reset")
        elif command == "open":
            self.command_gripper(open_gripper=True)
        elif command == "close":
            self.command_gripper(open_gripper=False)

    def tick(self):
        target = self.latest_target if self.is_fresh(self.latest_target_time, "target_timeout_s") else None
        objects = self.current_avoid_objects(target)
        obs, bins, nearest = self.make_observation(target, objects)
        grab_cmd = self.update_grab_sequence(target) if self.active else None

        if grab_cmd is not None:
            linear_x, angular_z = grab_cmd
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        elif self.active and self.policy is not None:
            self.raw_action = self.infer_action(obs)
            self.filtered_action = self.filter_action(self.filtered_action, self.raw_action)
            linear_x, angular_z = self.action_to_cmd(self.filtered_action)
        else:
            self.raw_action = [0.0, 0.0]
            self.filtered_action = self.filter_action(self.filtered_action, self.raw_action)
            linear_x, angular_z = (0.0, 0.0)

        if self.active or bool(self.get_parameter("publish_stop_when_inactive").value):
            self.publish_cmd(linear_x, angular_z)
        self.publish_state(obs, bins, nearest, linear_x, angular_z)

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

        pose_fresh = self.is_fresh(self.latest_pose_time, "pose_timeout_s")
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
        with torch.no_grad():
            obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
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
            centered = abs(target.x) <= self.get_float("grab_center_tolerance")
            close_enough = target.y >= self.get_float("grab_area_ratio")
            if not (centered and close_enough):
                return None

            self.publish_cmd(0.0, 0.0)
            self.command_gripper(open_gripper=True)
            self.change_grab_state(self.GRAB_OPENING)
            return (0.0, 0.0)

        if self.grab_state == self.GRAB_OPENING:
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
            self.change_grab_state(self.GRAB_COMPLETE)
            if bool(self.get_parameter("stop_after_grab").value):
                self.active = False
                self.get_logger().info("Object grabbed; RL drive stopped")
            else:
                self.change_grab_state(self.GRAB_TRACKING)
            return (0.0, 0.0)

        if self.grab_state == self.GRAB_COMPLETE:
            return (0.0, 0.0)

        return None

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
            return
        self.grab_state = state
        self.grab_state_started_at = self.get_clock().now()
        self.get_logger().info(f"Grab state -> {state}")

    def grab_state_age_s(self):
        elapsed = self.get_clock().now() - self.grab_state_started_at
        return elapsed.nanoseconds / 1_000_000_000.0

    def publish_state(self, obs, bins, nearest, linear_x, angular_z):
        msg = String()
        msg.data = json.dumps(
            {
                "active": self.active,
                "dry_run": self.dry_run,
                "model_loaded": self.policy is not None,
                "observation_dim": OBSERVATION_DIM,
                "observation_names": OBSERVATION_NAMES,
                "grab_state": self.grab_state,
                "gripper_enabled": bool(self.get_parameter("gripper_enabled").value),
                "obs": [round(v, 4) for v in obs],
                "raw_action": [round(v, 4) for v in self.raw_action],
                "filtered_action": [round(v, 4) for v in self.filtered_action],
                "linear_x": round(float(linear_x), 4),
                "angular_z": round(float(angular_z), 4),
                "avoid_left": round(bins[0], 4),
                "avoid_center": round(bins[1], 4),
                "avoid_right": round(bins[2], 4),
                "nearest_avoid_x": None if nearest is None else round(nearest.x, 4),
                "nearest_avoid_y": None if nearest is None else round(nearest.y, 4),
            },
            ensure_ascii=True,
        )
        self.state_pub.publish(msg)

    def is_fresh(self, stamp, timeout_param):
        if stamp is None:
            return False
        age = self.get_clock().now() - stamp
        return age.nanoseconds / 1_000_000_000.0 <= self.get_float(timeout_param)

    def get_float(self, name):
        return float(self.get_parameter(name).value)

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
