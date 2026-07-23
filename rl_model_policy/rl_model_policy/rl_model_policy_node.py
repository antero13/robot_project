import json
import math
from collections import deque
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState, PWMServoState
from ros_robot_controller_msgs.msg import SetBusServoState, SetPWMServoState
from std_msgs.msg import Bool, Float32, Float64, String

from rl_model_policy.coverage_controller import (
    CoverageController,
    generate_coverage_legs,
    normalize_angle,
)
from rl_model_policy.avoidance_roi import point_is_inside_trapezoid_roi
from rl_model_policy.deterministic_pickup_controller import (
    DeterministicPickupController,
)
from rl_model_policy.mission_coordinator import (
    curved_pose_waypoint_command,
    fixed_heading_dash_command,
    MissionCoordinator,
    MissionPhase,
    ReturnReason,
    storage_pose_bounds_required,
    storage_second_repush_required,
    tapered_waypoint_speed,
    StorageCurveAvoidanceController,
    storage_phase_after_staging_x,
    storage_return_start_phase,
    storage_staging_coordinates,
    storage_visit_dash_heading,
    storage_visit_number,
    waypoint_avoidance_required,
    waypoint_command,
)
from rl_model_policy.observation import (
    OBSERVATION_DIM,
    OBSERVATION_NAMES,
    estimate_target_image_x,
    estimate_target_world_bearing,
    make_pose_observation,
    pose_is_usable,
    quaternion_to_yaw,
    validate_observation,
)
from rl_model_policy.pickup_trigger import pickup_is_ready
from rl_model_policy.leave_start import make_leave_start_command
from rl_model_policy.lane_tof_correction import (
    coarse_heading_is_aligned,
    desired_yaw_for_wall,
    make_lane_tof_command,
    should_run_lane_tof_fine_alignment,
)
from rl_model_policy.main_road_tof_correction import make_main_road_tof_command
from rl_model_policy.storage_tof_correction import (
    desired_yaw_for_storage_axis,
    make_storage_tof_command,
    measurement_gap_timed_out,
    storage_coarse_heading_is_aligned,
)
from rl_model_policy.storage_exit_tof_correction import (
    make_storage_exit_tof_command,
)
from rl_model_policy.target_reacquisition import (
    reacquire_angular_velocity,
    reacquire_duration_for_evidence,
)
from rl_model_policy.target_confirmation import target_is_confirmed
from rl_model_policy.target_activation import (
    coverage_phase_allows_target_search,
    storage_repickup_guard_is_active,
    target_is_eligible,
)


class DeterministicMissionControllerNode(Node):
    """Run deterministic search, ROI avoidance, target approach, and pickup."""

    MODE_IDLE = "IDLE"
    MODE_LEAVE_START = "LEAVE_START"
    MODE_TRACK_TARGET = "TRACK_TARGET"
    MODE_LOCAL_REACQUIRE = "LOCAL_REACQUIRE"
    MODE_COVERAGE_SEARCH = "COVERAGE_SEARCH"
    MODE_WAITING_FOR_POSE = "WAITING_FOR_POSE"
    MODE_GRAB_SEQUENCE = "GRAB_SEQUENCE"
    MODE_RETURN_TO_STORAGE = "RETURN_TO_STORAGE"
    MODE_ENTER_STORAGE = "ENTER_STORAGE"
    MODE_EXIT_STORAGE = "EXIT_STORAGE"
    MODE_MISSION_COMPLETE = "MISSION_COMPLETE"
    MODE_MISSION_TIMEOUT = "MISSION_TIMEOUT"

    GRAB_TRACKING = "TRACKING"
    GRAB_OPENING = "OPENING"
    GRAB_FINAL_FORWARD = "FINAL_FORWARD"
    GRAB_CLOSING = "CLOSING"
    GRAB_COMPLETE = "GRABBED"

    def __init__(self):
        super().__init__("deterministic_mission_controller")

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
        self.declare_parameter(
            "pwm_servo_topic", "/ros_robot_controller/pwm_servo/set_state"
        )
        self.declare_parameter(
            "bus_servo_topic", "/ros_robot_controller/bus_servo/set_state"
        )

        self.declare_parameter("active_on_start", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("timer_rate_hz", 10.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("target_tracking_timeout_s", 1.5)
        self.declare_parameter("target_confirmation_window", 5)
        self.declare_parameter("target_confirmation_min_detections", 3)
        self.declare_parameter("target_activation_center_y_min", 0.30)
        self.declare_parameter("target_tracking_center_y_min", 0.22)
        self.declare_parameter("target_bearing_prediction_enabled", True)
        self.declare_parameter("near_target_loss_enabled", True)
        self.declare_parameter("near_target_loss_margin", 0.10)
        self.declare_parameter("near_target_loss_timeout_s", 0.60)
        self.declare_parameter("near_target_loss_min_missing_s", 0.15)
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
        self.declare_parameter("leave_start_target_yaw_deg", 90.0)
        self.declare_parameter("leave_start_heading_gain", 1.5)
        self.declare_parameter("leave_start_max_angular_speed", 0.60)
        self.declare_parameter("leave_start_heading_tolerance", 0.12)

        self.declare_parameter("coverage_enabled", True)
        self.declare_parameter("coverage_min_x", -1.25)
        self.declare_parameter("coverage_max_x", 1.25)
        self.declare_parameter("coverage_first_entry_y", -1.3343)
        self.declare_parameter("coverage_main_road_y", -1.40)
        self.declare_parameter("coverage_scan_end_y", 1.1)
        self.declare_parameter("coverage_lane_spacing", 1.0)
        self.declare_parameter("coverage_scan_speed", 0.24)
        self.declare_parameter("coverage_transit_speed", 0.40)
        self.declare_parameter("coverage_return_speed", 0.24)
        self.declare_parameter("coverage_waypoint_tolerance", 0.10)
        self.declare_parameter("coverage_heading_tolerance", 0.08)
        self.declare_parameter("coverage_heading_gain", 2.4)
        self.declare_parameter("coverage_max_angular_speed", 1.00)
        self.declare_parameter("coverage_turn_in_place_threshold", 0.65)
        self.declare_parameter("coverage_avoid_danger_threshold", 0.20)
        self.declare_parameter("coverage_avoid_heading_tolerance", 0.14)
        self.declare_parameter("coverage_avoid_angular_speed", 0.45)
        self.declare_parameter("coverage_avoid_linear_scale", 0.70)
        self.declare_parameter("coverage_rejoin_speed", 0.20)
        self.declare_parameter("coverage_rejoin_coordinate_limit", 1.80)
        self.declare_parameter("coverage_reacquire_duration_s", 1.5)
        self.declare_parameter("coverage_reacquire_reverse_after_s", 0.75)
        self.declare_parameter("coverage_reacquire_angular_z", 0.35)
        self.declare_parameter(
            "coverage_reacquire_single_detection_duration_s",
            0.4,
        )
        self.declare_parameter(
            "coverage_reacquire_two_detection_duration_s",
            0.7,
        )
        self.declare_parameter("storage_repickup_guard_enabled", True)
        self.declare_parameter("storage_repickup_guard_start_y", -0.95)
        self.declare_parameter("lane_tof_correction_enabled", True)
        self.declare_parameter("wall_distance_angle_topic", "/wall/distance_angle")
        self.declare_parameter("pose_x_correction_topic", "/robot_pose/correct_x")
        self.declare_parameter("pose_y_correction_topic", "/robot_pose/correct_y")
        self.declare_parameter(
            "pose_yaw_correction_topic", "/robot_pose/correct_yaw"
        )
        self.declare_parameter("lane_tof_left_wall_x_m", -2.0)
        self.declare_parameter("lane_tof_right_wall_x_m", 2.0)
        self.declare_parameter("lane_tof_sensor_forward_offset_m", 0.09)
        self.declare_parameter("lane_tof_measurement_timeout_s", 0.25)
        self.declare_parameter("lane_tof_x_tolerance_m", 0.03)
        self.declare_parameter("lane_tof_min_speed", 0.08)
        self.declare_parameter("lane_tof_slowdown_distance_m", 0.20)
        self.declare_parameter(
            "lane_tof_wall_angle_tolerance_rad", math.radians(4.0)
        )
        self.declare_parameter("lane_tof_angle_kp", 1.2)
        self.declare_parameter("lane_tof_angle_kd", 0.08)
        self.declare_parameter("lane_tof_angle_max_angular_speed", 0.30)
        self.declare_parameter("tof_wall_angle_sign", 1.0)
        self.declare_parameter("tof_validation_samples", 3)
        self.declare_parameter(
            "tof_max_valid_wall_angle_rad", math.radians(25.0)
        )
        self.declare_parameter("tof_max_angle_spread_rad", math.radians(8.0))
        self.declare_parameter("tof_max_distance_spread_m", 0.12)
        self.declare_parameter("tof_alignment_watchdog_s", 4.0)
        self.declare_parameter("main_road_tof_correction_enabled", True)
        self.declare_parameter("main_road_tof_south_wall_y_m", -2.0)
        self.declare_parameter("main_road_tof_sensor_forward_offset_m", 0.09)
        self.declare_parameter("main_road_tof_measurement_timeout_s", 0.25)
        self.declare_parameter("main_road_tof_y_tolerance_m", 0.03)
        self.declare_parameter("main_road_tof_min_speed", 0.05)
        self.declare_parameter("main_road_tof_slowdown_distance_m", 0.20)
        self.declare_parameter(
            "main_road_tof_angle_trigger_rad", math.radians(4.0)
        )
        self.declare_parameter(
            "main_road_tof_angle_release_rad", math.radians(4.0)
        )

        self.declare_parameter("avoid_enabled", True)
        self.declare_parameter("avoid_area_ratio", 0.38)
        self.declare_parameter("avoid_emergency_ratio", 0.68)
        self.declare_parameter("avoid_center_band", 0.75)
        self.declare_parameter("avoid_center_corridor", 0.30)
        self.declare_parameter("avoid_path_margin", 0.30)
        self.declare_parameter("avoid_vfh_center_weight", 0.5)
        self.declare_parameter("avoid_only_if_closer_than_target", True)
        self.declare_parameter("avoid_closer_ratio", 0.85)
        self.declare_parameter("avoid_roi_enabled", True)
        self.declare_parameter("avoid_roi_left_near_x", -0.6563)
        self.declare_parameter("avoid_roi_left_near_y", 0.7483)
        self.declare_parameter("avoid_roi_left_far_x", -0.2649)
        self.declare_parameter("avoid_roi_left_far_y", 0.2576)
        self.declare_parameter("avoid_roi_right_near_x", 0.4620)
        self.declare_parameter("avoid_roi_right_near_y", 0.6992)
        self.declare_parameter("avoid_roi_right_far_x", 0.0951)
        self.declare_parameter("avoid_roi_right_far_y", 0.2567)
        self.declare_parameter("avoid_turn_duration_s", 0.55)
        self.declare_parameter("avoid_turn_angular_z", 0.65)
        self.declare_parameter("avoid_forward_duration_s", 0.85)
        self.declare_parameter("avoid_forward_linear_x", 0.05)
        self.declare_parameter("avoid_forward_angular_z", 0.25)
        self.declare_parameter("avoid_escape_duration_s", 0.70)
        self.declare_parameter("avoid_escape_linear_x", 0.06)
        self.declare_parameter("avoid_escape_angular_z", 0.20)
        self.declare_parameter("avoid_vfh_target_weight", 0.60)
        self.declare_parameter("avoid_vfh_switch_penalty", 0.25)
        self.declare_parameter("avoid_direction_hold_s", 0.8)

        self.declare_parameter("approach_center_tolerance", 0.12)
        self.declare_parameter("approach_max_linear_x", 0.10)
        self.declare_parameter("approach_min_linear_x", 0.03)
        self.declare_parameter("approach_angular_gain", 0.8)
        self.declare_parameter("approach_max_angular_z", 0.45)
        self.declare_parameter("max_angular_speed", 0.80)
        self.declare_parameter("publish_stop_when_inactive", True)

        self.declare_parameter("full_mission_enabled", True)
        self.declare_parameter("mission_duration_s", 180.0)
        self.declare_parameter("force_return_remaining_s", 30.0)
        self.declare_parameter("storage_capacity", 4)
        self.declare_parameter("target_object_count", 7)
        self.declare_parameter("storage_main_road_y", -1.40)
        self.declare_parameter("storage_staging_x", -1.25)
        self.declare_parameter("storage_staging_y", -1.70)
        self.declare_parameter("storage_second_staging_x", -1.60)
        self.declare_parameter("storage_second_staging_y", -1.40)
        self.declare_parameter("storage_exit_x", -1.25)
        self.declare_parameter("storage_center_x", -1.80)
        self.declare_parameter("storage_center_y", -1.80)
        self.declare_parameter("storage_entry_yaw_deg", -90.0)
        self.declare_parameter("storage_return_speed", 0.25)
        self.declare_parameter("storage_entry_speed", 0.30)
        self.declare_parameter("storage_x_entry_speed", 0.40)
        self.declare_parameter("storage_exit_reverse_speed", 0.40)
        self.declare_parameter("storage_entry_dash_duration_s", 1.60)
        self.declare_parameter("storage_second_entry_dash_duration_s", 1.20)
        self.declare_parameter("storage_entry_dash_heading_deg", -165.0)
        self.declare_parameter("storage_second_entry_dash_heading_deg", -113.0)
        self.declare_parameter("storage_exit_dash_duration_s", 1.50)
        self.declare_parameter("storage_second_exit_dash_duration_s", 1.10)
        self.declare_parameter("storage_second_repush_speed", 0.13)
        self.declare_parameter("storage_second_side_shift_speed", 0.40)
        self.declare_parameter("storage_second_side_reverse_duration_s", 0.70)
        self.declare_parameter("storage_second_side_target_x", -1.57)
        self.declare_parameter("storage_second_side_target_y", -1.83)
        self.declare_parameter(
            "storage_second_side_slowdown_distance_m", 0.20
        )
        self.declare_parameter(
            "storage_second_side_align_max_angular_speed", 1.00
        )
        self.declare_parameter(
            "storage_second_side_curve_control_distance_m", 0.20
        )
        self.declare_parameter(
            "storage_second_side_curve_lookahead_distance_m", 0.08
        )
        self.declare_parameter("storage_second_repush_duration_s", 1.00)
        self.declare_parameter("storage_contact_settle_duration_s", 0.20)
        self.declare_parameter("storage_dash_heading_tolerance", 0.05)
        self.declare_parameter("storage_dash_max_angular_speed", 0.30)
        self.declare_parameter("storage_waypoint_tolerance", 0.10)
        self.declare_parameter("storage_entry_tolerance", 0.04)
        self.declare_parameter("storage_heading_tolerance", 0.14)
        self.declare_parameter("storage_final_yaw_tolerance", 0.12)
        self.declare_parameter("storage_heading_gain", 2.4)
        self.declare_parameter("storage_max_angular_speed", 0.80)
        self.declare_parameter("storage_tof_angle_gain", 1.5)
        self.declare_parameter("storage_tof_max_angular_speed", 0.60)
        self.declare_parameter("storage_avoid_danger_threshold", 0.20)
        self.declare_parameter("storage_tof_correction_enabled", True)
        self.declare_parameter("storage_tof_left_wall_x_m", -2.0)
        self.declare_parameter("storage_tof_bottom_wall_y_m", -2.0)
        self.declare_parameter("storage_tof_sensor_forward_offset_m", 0.09)
        self.declare_parameter("storage_tof_measurement_timeout_s", 0.25)
        self.declare_parameter("storage_exit_tof_fallback_timeout_s", 1.0)
        self.declare_parameter("storage_tof_xy_tolerance_m", 0.03)
        self.declare_parameter("storage_tof_min_speed", 0.05)
        self.declare_parameter("storage_tof_slowdown_distance_m", 0.20)
        self.declare_parameter(
            "storage_tof_wall_angle_tolerance_rad", math.radians(4.0)
        )
        self.declare_parameter(
            "storage_exit_tof_angle_trigger_rad", math.radians(4.0)
        )
        self.declare_parameter(
            "storage_exit_tof_angle_release_rad", math.radians(4.0)
        )

        self.declare_parameter("gripper_enabled", True)
        self.declare_parameter("gripper_type", "bus")
        self.declare_parameter("gripper_servo_id", 1)
        self.declare_parameter("gripper_open_position", 1000)
        self.declare_parameter("gripper_closed_position", 300)
        self.declare_parameter("gripper_move_duration_s", 0.5)
        self.declare_parameter("gripper_open_before_start", True)
        self.declare_parameter("start_gripper_close_delay_s", 0.0)
        self.declare_parameter("grab_center_tolerance", 0.18)
        self.declare_parameter("grab_area_ratio", 0.70)
        self.declare_parameter("grab_detection_timeout_s", 0.25)
        self.declare_parameter("final_forward_linear_x", 0.22)
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
        self.latest_target_visible = False
        self.latest_target_visibility_time = None
        self.near_target_candidate = None
        self.near_target_candidate_time = None
        self.near_target_candidate_label = None
        self.near_target_missing_started_at = None
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
        self.target_visibility_history = deque(maxlen=self.target_confirmation_window)
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
        self.pickup_controller = DeterministicPickupController(
            center_tolerance=self.get_float("approach_center_tolerance"),
            approach_max_linear_x=self.get_float("approach_max_linear_x"),
            approach_min_linear_x=self.get_float("approach_min_linear_x"),
            approach_angular_gain=self.get_float("approach_angular_gain"),
            approach_max_angular_z=self.get_float("approach_max_angular_z"),
            grab_area_ratio=self.get_float("grab_area_ratio"),
            avoid_turn_duration_s=self.get_float("avoid_turn_duration_s"),
            avoid_turn_angular_z=self.get_float("avoid_turn_angular_z"),
            avoid_forward_duration_s=self.get_float("avoid_forward_duration_s"),
            avoid_forward_linear_x=self.get_float("avoid_forward_linear_x"),
            avoid_forward_angular_z=self.get_float("avoid_forward_angular_z"),
            avoid_escape_duration_s=self.get_float("avoid_escape_duration_s"),
            avoid_escape_linear_x=self.get_float("avoid_escape_linear_x"),
            avoid_escape_angular_z=self.get_float("avoid_escape_angular_z"),
            avoid_vfh_target_weight=self.get_float("avoid_vfh_target_weight"),
            avoid_vfh_switch_penalty=self.get_float("avoid_vfh_switch_penalty"),
            avoid_direction_hold_s=self.get_float("avoid_direction_hold_s"),
        )
        self.last_cmd = (0.0, 0.0)
        self.grab_state = self.GRAB_TRACKING
        self.grab_state_started_at = self.get_clock().now()
        self.grab_state_elapsed_offset_s = 0.0
        self.motion_paused = False
        self.start_motion_not_before_s = None
        self.pickup_label = None
        self.pickup_target_x = None
        self.control_mode = self.MODE_IDLE
        self.leave_start_active = self.active and bool(
            self.get_parameter("leave_start_enabled").value
        )
        self.leave_start_origin = None
        self.leave_start_yaw = None
        self.leave_start_traveled_m = 0.0
        if self.leave_start_active:
            self.control_mode = self.MODE_LEAVE_START
        self.had_visible_target = False
        self.target_lost_started_at = None
        self.target_reacquire_detection_count = 0
        self.target_reacquire_confirmed = False
        self.coverage_command = None
        self.latest_wall_distance_m = None
        self.latest_wall_angle_rad = None
        self.latest_wall_min_distance_m = None
        self.latest_wall_measurement_time = None
        self.wall_measurement_history = deque(maxlen=10)
        self.pending_pose_x_correction = None
        self.pending_pose_x_correction_time = None
        self.pending_pose_y_correction = None
        self.pending_pose_y_correction_time = None
        self.pending_pose_yaw_correction = None
        self.pending_pose_yaw_correction_time = None
        self.storage_exit_tof_missing_started_at = None
        self.storage_exit_tof_angle_alignment_active = False
        self.storage_tof_coarse_heading_aligned = False
        self.storage_exit_tof_coarse_heading_aligned = False
        self.lane_tof_started_at_s = None
        self.lane_tof_coarse_heading_aligned = False
        self.lane_tof_previous_angle_rad = None
        self.lane_tof_previous_angle_time_s = None
        self.main_road_tof_started_at_s = None
        self.storage_dash_timer_phase = None
        self.storage_dash_elapsed_s = 0.0
        self.storage_dash_last_update_s = None
        self.mission_waypoint = None
        self.storage_side_curve_start = None
        self.return_lane_x = 0.0
        self.return_lane_number = 0
        self.storage_visit_number = 1
        storage_avoid_threshold = self.get_float(
            "storage_avoid_danger_threshold"
        )
        self.storage_curve_avoidance = StorageCurveAvoidanceController(
            danger_threshold=storage_avoid_threshold,
            release_threshold=storage_avoid_threshold * 0.60,
            linear_scale=self.get_float("coverage_avoid_linear_scale"),
            angular_speed=self.get_float("coverage_avoid_angular_speed"),
            direction_hold_s=self.get_float("avoid_direction_hold_s"),
            clear_samples=3,
            max_angular_speed=self.get_float("storage_max_angular_speed"),
        )

        self.mission = MissionCoordinator(
            storage_capacity=int(self.get_parameter("storage_capacity").value),
            target_object_count=int(self.get_parameter("target_object_count").value),
            mission_duration_s=self.get_float("mission_duration_s"),
            force_return_remaining_s=self.get_float("force_return_remaining_s"),
        )
        if self.active:
            self.mission.start(self.now_s())

        self.model_observation_dim = OBSERVATION_DIM
        self.coverage_controller = self.create_coverage_controller()
        self.lane_tof_alignment_active = False
        self.main_road_tof_alignment_active = False
        self.main_road_tof_angle_alignment_active = False

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, 10
        )
        self.pose_x_correction_pub = self.create_publisher(
            Float64,
            self.get_parameter("pose_x_correction_topic").value,
            10,
        )
        self.pose_y_correction_pub = self.create_publisher(
            Float64,
            self.get_parameter("pose_y_correction_topic").value,
            10,
        )
        self.pose_yaw_correction_pub = self.create_publisher(
            Float64,
            self.get_parameter("pose_yaw_correction_topic").value,
            10,
        )
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
        self.wall_distance_sub = self.create_subscription(
            Vector3Stamped,
            self.get_parameter("wall_distance_angle_topic").value,
            self.wall_distance_callback,
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
                "Odometry is enabled for deterministic coverage search"
            )
        self.control_sub = self.create_subscription(
            String,
            self.get_parameter("control_topic").value,
            self.control_callback,
            10,
        )

        self.idle_gripper_timer = None
        if self.active:
            self.command_gripper(open_gripper=False)
            self.start_motion_not_before_s = None
        elif (
            bool(self.get_parameter("gripper_enabled").value)
            and bool(self.get_parameter("gripper_open_before_start").value)
        ):
            # Delay the first command briefly so the servo controller has time
            # to match this publisher after all launch processes start.
            self.idle_gripper_timer = self.create_timer(
                0.75,
                self.open_gripper_before_start,
            )

        timer_rate_hz = self.get_float("timer_rate_hz")
        self.timer = self.create_timer(1.0 / timer_rate_hz, self.tick)

        self.get_logger().info(
            "Deterministic mission controller ready. "
            f"active={self.active}, dry_run={self.dry_run}, rl_enabled=False"
        )
        self.get_logger().info(
            "Send start/pause_motion/resume_motion/stop on "
            f"{self.get_parameter('control_topic').value}; publishing cmd_vel on "
            f"{self.get_parameter('cmd_vel_topic').value}"
        )

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
        if not self.target_search_is_allowed():
            self.target_visibility_history.clear()
            return
        visible = bool(msg.data)
        was_visible = self.latest_target_visible
        self.latest_target_visible = visible
        self.latest_target_visibility_time = self.get_clock().now()
        if visible:
            self.near_target_missing_started_at = None
        elif was_visible or self.near_target_missing_started_at is None:
            self.near_target_missing_started_at = self.get_clock().now()
        self.target_visibility_history.append(visible)

    def target_center_y_callback(self, msg):
        self.latest_target_center_y = float(msg.data)
        self.latest_target_center_y_time = self.get_clock().now()

    def wall_distance_callback(self, msg):
        values = (
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        )
        if not all(math.isfinite(value) for value in values):
            return
        angle_sign = self.get_float("tof_wall_angle_sign")
        if abs(abs(angle_sign) - 1.0) > 1e-6:
            self.get_logger().warning(
                "tof_wall_angle_sign must be -1 or 1; ignoring wall measurement"
            )
            return
        corrected_angle = angle_sign * values[1]
        now = self.get_clock().now()
        self.latest_wall_distance_m = values[0]
        self.latest_wall_angle_rad = corrected_angle
        self.latest_wall_min_distance_m = values[2]
        self.latest_wall_measurement_time = now
        self.wall_measurement_history.append(
            (self.now_s(), values[0], corrected_angle)
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

        raw_objects = (
            payload.get("objects", payload) if isinstance(payload, dict) else payload
        )
        if not isinstance(raw_objects, list):
            return

        objects = []
        for raw in raw_objects:
            if not isinstance(raw, dict):
                continue
            try:
                x = float(raw.get("x", raw.get("point_x", 0.0)))
                y = float(raw.get("y", raw.get("point_y", 0.0)))
                center_y = float(
                    raw.get("center_y", raw.get("bbox_center_y", y))
                )
                bottom_y = float(raw.get("bottom_y", y))
                confidence = float(raw.get("confidence", raw.get("z", 1.0)))
            except (TypeError, ValueError):
                continue
            objects.append(
                self.make_point(
                    x,
                    y,
                    confidence,
                    center_y=center_y,
                    bottom_y=bottom_y,
                )
            )

        self.latest_avoid_objects = objects
        self.latest_avoid_objects_time = self.get_clock().now()

    def control_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ("start", "run", "demo"):
            self.mission.start(self.now_s())
            self.storage_visit_number = 1
            self.coverage_controller.reset()
            self.pickup_controller.reset()
            self.coverage_command = None
            self.pending_pose_x_correction = None
            self.pending_pose_x_correction_time = None
            self.pending_pose_y_correction = None
            self.pending_pose_y_correction_time = None
            self.pending_pose_yaw_correction = None
            self.pending_pose_yaw_correction_time = None
            self.reset_main_road_tof_alignment()
            self.reset_lane_tof_alignment()
            self.reset_storage_tof_alignment()
            self.mission_waypoint = None
            self.reset_target_reacquisition()
            self.target_visibility_history.clear()
            self.clear_near_target_candidate()
            self.pickup_target_x = None
            self.motion_paused = False
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(open_gripper=False)
            self.start_motion_not_before_s = None
            self.active = True
            self.begin_leave_start()
            self.get_logger().info("Deterministic mission controller started")
        elif command in ("pause", "pause_motion"):
            self.set_motion_paused(True)
        elif command in ("resume", "resume_motion"):
            self.set_motion_paused(False)
        elif command == "stop":
            self.active = False
            self.start_motion_not_before_s = None
            self.mission.stop(self.now_s())
            self.motion_paused = False
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.pickup_controller.reset()
            self.coverage_command = None
            self.cancel_leave_start()
            self.set_control_mode(self.MODE_IDLE)
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Deterministic mission controller stopped")
        elif command == "reset":
            self.active = False
            self.start_motion_not_before_s = None
            self.mission.reset()
            self.storage_visit_number = 1
            self.latest_target = None
            self.latest_target_label = None
            self.latest_target_label_time = None
            self.latest_avoid = None
            self.latest_avoid_objects = []
            self.last_target_world_bearing = None
            self.time_since_target_seen = self.get_float("episode_length_s")
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.pickup_controller.reset()
            self.coverage_controller.reset()
            self.coverage_command = None
            self.pending_pose_x_correction = None
            self.pending_pose_x_correction_time = None
            self.pending_pose_y_correction = None
            self.pending_pose_y_correction_time = None
            self.pending_pose_yaw_correction = None
            self.pending_pose_yaw_correction_time = None
            self.reset_main_road_tof_alignment()
            self.reset_lane_tof_alignment()
            self.reset_storage_tof_alignment()
            self.reset_target_reacquisition()
            self.target_visibility_history.clear()
            self.clear_near_target_candidate()
            self.motion_paused = False
            self.pickup_label = None
            self.pickup_target_x = None
            self.mission_waypoint = None
            self.cancel_leave_start()
            self.set_control_mode(self.MODE_IDLE)
            self.change_grab_state(self.GRAB_TRACKING)
            self.command_gripper(
                open_gripper=bool(
                    self.get_parameter("gripper_open_before_start").value
                )
            )
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Deterministic mission controller reset")
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
        pickup_objects = self.pickup_avoid_objects(objects)
        pickup_bins = self.avoid_bins(pickup_objects)[:3]
        if self.active and self.full_mission_is_enabled():
            previous_phase = self.mission.phase
            pickup_in_progress = self.grab_state != self.GRAB_TRACKING
            self.mission.update_time(
                now_s,
                defer_storage_return=pickup_in_progress,
            )
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
        target_search_allowed = self.target_search_is_allowed()
        if collecting and target_search_allowed:
            self.update_target_history(target)
        tracking_target = self.control_mode == self.MODE_TRACK_TARGET
        target_control_ready = (
            target_search_allowed
            and target is not None
            and self.target_activation_is_met(tracking_active=tracking_target)
            and (tracking_target or self.target_confirmation_is_met())
        )
        confirmed_target = target if target_control_ready else None
        if confirmed_target is not None:
            self.target_reacquire_confirmed = True
        self.update_near_target_candidate(confirmed_target)
        near_target_loss_ready = self.near_target_loss_pickup_ready()
        pickup_avoid_required = (
            confirmed_target is not None
            and self.should_avoid_target_path(pickup_objects, confirmed_target)
        )
        grab_cmd = (
            self.update_grab_sequence(
                confirmed_target,
                allow_near_target_loss=near_target_loss_ready,
            )
            if (
                self.active
                and collecting
                and not self.motion_paused
                and not self.coverage_controller.rejoin_active
                and not self.leave_start_active
                and not pickup_avoid_required
                and not self.pickup_controller.is_avoiding
            )
            else None
        )
        if self.active and self.start_motion_is_held(now_s):
            linear_x, angular_z = (0.0, 0.0)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif self.active and self.motion_paused:
            self.pause_storage_dash_timer()
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
        elif self.active and collecting and self.coverage_controller.rejoin_active:
            linear_x, angular_z = self.coverage_search_command(bins)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        elif self.active and collecting and self.main_road_tof_alignment_active:
            linear_x, angular_z = self.coverage_search_command(bins)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        elif (
            self.active
            and collecting
            and (
                confirmed_target is not None
                or self.pickup_controller.is_avoiding
            )
        ):
            self.reset_lane_tof_alignment()
            self.set_control_mode(self.MODE_TRACK_TARGET)
            pickup_command = self.pickup_controller.command(
                now_s=now_s,
                target_x=(
                    None if confirmed_target is None else confirmed_target.x
                ),
                target_y=(
                    None if confirmed_target is None else confirmed_target.y
                ),
                avoid_required=pickup_avoid_required,
                avoid_bins=pickup_bins,
            )
            linear_x = pickup_command.linear_x
            angular_z = pickup_command.angular_z
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
        elif (
            self.active
            and target_search_allowed
            and self.should_locally_reacquire()
        ):
            self.set_control_mode(self.MODE_LOCAL_REACQUIRE)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
            self.coverage_command = None
            linear_x = 0.0
            angular_z = reacquire_angular_velocity(
                last_target_direction=self.last_target_direction,
                elapsed_s=self.target_lost_age_s(),
                reverse_after_s=self.get_float("coverage_reacquire_reverse_after_s"),
                angular_speed=self.get_float("coverage_reacquire_angular_z"),
                reverse_enabled=self.target_reacquire_confirmed,
            )
        elif self.active and self.coverage_is_enabled():
            linear_x, angular_z = self.coverage_search_command(bins)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
        else:
            if self.mission.phase == MissionPhase.COMPLETE:
                self.set_control_mode(self.MODE_MISSION_COMPLETE)
            elif self.mission.phase == MissionPhase.TIMEOUT:
                self.set_control_mode(self.MODE_MISSION_TIMEOUT)
            else:
                self.set_control_mode(self.MODE_IDLE)
            self.raw_action = [0.0, 0.0]
            self.filtered_action = [0.0, 0.0]
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
            self.pickup_controller.reset()
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                "Base motion paused; perception updates remain active"
            )
            return

        self.motion_paused = False
        self.grab_state_started_at = self.get_clock().now()
        self.get_logger().info("Base motion resumed")

    def open_gripper_before_start(self):
        if (
            self.active
            or not bool(self.get_parameter("gripper_enabled").value)
            or not bool(self.get_parameter("gripper_open_before_start").value)
        ):
            self.cancel_idle_gripper_timer()
            return
        self.command_gripper(open_gripper=True)
        gripper_type = str(self.get_parameter("gripper_type").value).strip().lower()
        publisher = (
            self.bus_servo_pub if gripper_type == "bus" else self.pwm_servo_pub
        )
        if publisher.get_subscription_count() > 0:
            self.cancel_idle_gripper_timer()

    def cancel_idle_gripper_timer(self):
        if self.idle_gripper_timer is not None:
            self.idle_gripper_timer.cancel()
            self.idle_gripper_timer = None

    def start_motion_is_held(self, now_s):
        if self.start_motion_not_before_s is None:
            return False
        if float(now_s) < float(self.start_motion_not_before_s):
            return True
        self.start_motion_not_before_s = None
        return False

    def prepare_storage_return(self):
        self.cancel_leave_start()
        self.coverage_controller.cancel_rejoin()
        self.coverage_controller.cancel_preferred_lane_end_turn()
        self.pickup_target_x = None
        self.reset_storage_dash_timer()
        self.pending_pose_x_correction = None
        self.pending_pose_x_correction_time = None
        self.pending_pose_y_correction = None
        self.pending_pose_y_correction_time = None
        self.pending_pose_yaw_correction = None
        self.pending_pose_yaw_correction_time = None
        self.reset_main_road_tof_alignment()
        self.reset_storage_tof_alignment()
        self.storage_visit_number = storage_visit_number(
            self.mission.delivered_count
        )
        staging_x, _ = self.active_storage_staging()
        return_min_x = min(
            self.get_float("coverage_min_x"),
            staging_x,
        )
        return_max_x = max(
            self.get_float("coverage_max_x"),
            staging_x,
        )
        self.return_lane_number = int(
            self.coverage_controller.current_leg.lane_number
        )
        self.return_lane_x = self.clamp(
            self.robot_x,
            return_min_x,
            return_max_x,
        )
        rejoin_started = self.coverage_controller.begin_rejoin(self.robot_y)
        if rejoin_started:
            self.return_lane_x = float(self.coverage_controller.current_leg.target_x)
            self.mission.set_phase(
                MissionPhase.REJOIN_STORAGE_LANE,
                self.now_s(),
            )
            self.mission_waypoint = (self.return_lane_x, self.robot_y)
            return_route = "rejoin lane, then return to the southern main road"
        else:
            self.coverage_controller.cancel_rejoin()
            return_phase = self.start_storage_main_road_return(self.now_s())
            return_route = f"direct {return_phase}"
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_center_y = None
        self.latest_target_center_y_time = None
        self.target_visibility_history.clear()
        self.reset_target_reacquisition()
        self.filtered_action = [0.0, 0.0]
        self.get_logger().info(
            "Returning to storage: "
            f"visit={self.storage_visit_number}, "
            f"reason={self.mission.return_reason}, "
            f"onboard={self.mission.onboard_count}, "
            f"delivered={self.mission.delivered_count}, route={return_route}"
        )

    def start_storage_main_road_return(self, now_s):
        phase = storage_return_start_phase()
        _, staging_y = self.active_storage_staging()
        self.mission.set_phase(phase, now_s)
        self.mission_waypoint = (
            self.return_lane_x,
            staging_y,
        )
        return phase

    def begin_storage_staging_return(self, now_s):
        staging_x, staging_y = self.active_storage_staging()
        self.reset_storage_tof_alignment()
        self.mission.set_phase(MissionPhase.RETURN_STAGING, now_s)
        self.mission_waypoint = (staging_x, staging_y)

    def active_storage_staging(self):
        return storage_staging_coordinates(
            self.storage_visit_number,
            self.get_float("storage_staging_x"),
            self.get_float("storage_staging_y"),
            self.get_float("storage_second_staging_x"),
            self.get_float("storage_second_staging_y"),
        )

    def active_storage_exit_dash_duration(self):
        parameter = (
            "storage_exit_dash_duration_s"
            if self.storage_visit_number == 1
            else "storage_second_exit_dash_duration_s"
        )
        return self.get_float(parameter)

    def active_storage_entry_dash_duration(self):
        parameter = (
            "storage_entry_dash_duration_s"
            if self.storage_visit_number == 1
            else "storage_second_entry_dash_duration_s"
        )
        return self.get_float(parameter)

    def reset_main_road_tof_alignment(self):
        self.main_road_tof_alignment_active = False
        self.main_road_tof_angle_alignment_active = False
        self.main_road_tof_started_at_s = None

    def reset_lane_tof_alignment(self):
        self.lane_tof_alignment_active = False
        self.lane_tof_started_at_s = None
        self.lane_tof_coarse_heading_aligned = False
        self.lane_tof_previous_angle_rad = None
        self.lane_tof_previous_angle_time_s = None

    def reset_storage_tof_alignment(self):
        self.storage_tof_coarse_heading_aligned = False
        self.storage_exit_tof_coarse_heading_aligned = False
        self.storage_exit_tof_angle_alignment_active = False

    def start_tof_watchdog(self, timer_name, now_s):
        if getattr(self, timer_name) is None:
            setattr(self, timer_name, float(now_s))

    def tof_watchdog_expired(self, timer_name, now_s):
        started_at_s = getattr(self, timer_name)
        if started_at_s is None:
            return False
        return (
            max(0.0, float(now_s) - float(started_at_s))
            >= self.get_float("tof_alignment_watchdog_s")
        )

    def validated_wall_measurement(
        self,
        timeout_parameter="lane_tof_measurement_timeout_s",
    ):
        age_s = self.wall_measurement_age_s()
        if (
            age_s is None
            or age_s > self.get_float(timeout_parameter)
        ):
            return (None, None, age_s)

        sample_count = max(1, int(self.get_parameter("tof_validation_samples").value))
        timeout_s = self.get_float(timeout_parameter)
        now_s = self.now_s()
        samples = [
            sample
            for sample in self.wall_measurement_history
            if 0.0 <= now_s - sample[0] <= timeout_s
        ][-sample_count:]
        if len(samples) < sample_count:
            return (None, None, age_s)

        distances = [sample[1] for sample in samples]
        angles = [sample[2] for sample in samples]
        if any(
            abs(angle) > self.get_float("tof_max_valid_wall_angle_rad")
            for angle in angles
        ):
            return (None, None, age_s)
        if (
            max(angles) - min(angles)
            > self.get_float("tof_max_angle_spread_rad")
        ):
            return (None, None, age_s)
        if (
            max(distances) - min(distances)
            > self.get_float("tof_max_distance_spread_m")
        ):
            return (None, None, age_s)

        distances.sort()
        angles.sort()
        middle = sample_count // 2
        if sample_count % 2:
            return (distances[middle], angles[middle], age_s)
        return (
            0.5 * (distances[middle - 1] + distances[middle]),
            0.5 * (angles[middle - 1] + angles[middle]),
            age_s,
        )

    def main_road_tof_command(
        self,
        *,
        target_y,
        transit_speed,
        storage_return,
        now_s=None,
    ):
        now_s = self.now_s() if now_s is None else float(now_s)
        self.start_tof_watchdog("main_road_tof_started_at_s", now_s)
        if self.tof_watchdog_expired("main_road_tof_started_at_s", now_s):
            self.get_logger().warning(
                "South-wall ToF alignment watchdog expired; continuing "
                "without applying a pose correction"
            )
            self.reset_main_road_tof_alignment()
            if storage_return:
                self.begin_storage_staging_return(now_s)
            else:
                self.coverage_controller.complete_current_leg("SCAN_LANE_DOWN")
                self.coverage_command = self.coverage_controller.hold_command(
                    "MAIN_ROAD_TOF_WATCHDOG_FALLBACK"
                )
            return (0.0, 0.0)

        distance_m, wall_angle_rad, measurement_age_s = (
            self.validated_wall_measurement(
                "main_road_tof_measurement_timeout_s"
            )
        )
        command = make_main_road_tof_command(
            distance_m=distance_m,
            wall_angle_rad=wall_angle_rad,
            measurement_age_s=measurement_age_s,
            robot_yaw=self.robot_yaw,
            target_y=target_y,
            south_wall_y_m=self.get_float("main_road_tof_south_wall_y_m"),
            sensor_forward_offset_m=self.get_float(
                "main_road_tof_sensor_forward_offset_m"
            ),
            transit_speed=transit_speed,
            minimum_speed=self.get_float("main_road_tof_min_speed"),
            slowdown_distance_m=self.get_float(
                "main_road_tof_slowdown_distance_m"
            ),
            y_tolerance_m=self.get_float("main_road_tof_y_tolerance_m"),
            measurement_timeout_s=self.get_float(
                "main_road_tof_measurement_timeout_s"
            ),
            angle_alignment_active=self.main_road_tof_angle_alignment_active,
            angle_trigger_rad=self.get_float("main_road_tof_angle_trigger_rad"),
            angle_release_rad=self.get_float("main_road_tof_angle_release_rad"),
            angle_gain=self.get_float("coverage_heading_gain"),
            max_angular_speed=self.get_float("coverage_max_angular_speed"),
            heading_tolerance=self.get_float("coverage_heading_tolerance"),
        )
        self.main_road_tof_alignment_active = True
        self.main_road_tof_angle_alignment_active = (
            command.angle_alignment_active
        )
        self.mission_waypoint = (self.robot_x, float(target_y))
        self.set_control_mode(
            self.MODE_RETURN_TO_STORAGE
            if storage_return
            else self.MODE_COVERAGE_SEARCH
        )
        if not command.reached:
            if not storage_return:
                self.coverage_command = self.coverage_controller.external_command(
                    command.linear_x,
                    command.angular_z,
                    command.phase,
                )
            return (command.linear_x, command.angular_z)

        self.publish_pose_y_correction(float(target_y))
        self.publish_pose_yaw_correction(-math.pi / 2.0)
        measured_y = command.measured_robot_y
        wall_angle = wall_angle_rad
        self.reset_main_road_tof_alignment()
        correction_name = (
            "storage-approach" if storage_return else "main-road"
        )
        self.get_logger().info(
            f"South-wall {correction_name} correction complete: "
            f"measured_y={measured_y:.3f}, "
            f"wall_angle={math.degrees(wall_angle):.2f} deg, "
            f"pose_y->{float(target_y):.3f}, pose_yaw->-90.0 deg"
        )

        if storage_return:
            self.begin_storage_staging_return(now_s)
        else:
            self.coverage_controller.complete_current_leg("SCAN_LANE_DOWN")
            self.coverage_command = self.coverage_controller.hold_command(
                "MAIN_ROAD_SOUTH_TOF_ALIGNED"
            )
        return (0.0, 0.0)

    def storage_mission_command(self, bins, now_s):
        staging_x, staging_y = self.active_storage_staging()
        if not self.storage_pose_is_valid():
            self.pause_storage_dash_timer()
            self.mission_waypoint = None
            self.set_control_mode(self.MODE_WAITING_FOR_POSE)
            return (0.0, 0.0)

        waiting_for_x = self.waiting_for_pose_x_correction()
        waiting_for_y = self.waiting_for_pose_y_correction()
        waiting_for_yaw = self.waiting_for_pose_yaw_correction()
        if waiting_for_x or waiting_for_y or waiting_for_yaw:
            self.pause_storage_dash_timer()
            if self.mission.phase == MissionPhase.EXIT_STORAGE:
                self.mission_waypoint = (
                    self.get_float("storage_center_x"),
                    self.get_float("storage_center_y"),
                )
            elif waiting_for_x:
                self.mission_waypoint = (
                    self.pending_pose_x_correction,
                    staging_y,
                )
            elif waiting_for_y:
                self.mission_waypoint = (
                    self.robot_x,
                    self.pending_pose_y_correction,
                )
            elif (
                waiting_for_yaw
                and self.mission.phase == MissionPhase.RETURN_STAGING
            ):
                self.mission_waypoint = (
                    self.robot_x,
                    staging_y,
                )
            else:
                self.mission_waypoint = (
                    self.return_lane_x,
                    staging_y,
                )
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            return (0.0, 0.0)

        phase = self.mission.phase
        entry_yaw = math.radians(self.get_float("storage_entry_yaw_deg"))

        if phase == MissionPhase.REJOIN_STORAGE_LANE:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            target_y = self.coverage_controller.rejoin_target_y
            if target_y is None:
                self.start_storage_main_road_return(now_s)
                return (0.0, 0.0)
            self.mission_waypoint = (self.return_lane_x, target_y)
            command = self.coverage_controller.command(
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                avoid_left=bins[0],
                avoid_center=bins[1],
                avoid_right=bins[2],
            )
            if not self.coverage_controller.rejoin_active:
                self.start_storage_main_road_return(now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.RETURN_MAIN_ROAD:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                self.return_lane_x,
                staging_y,
                self.get_float("storage_return_speed"),
            )
            if command.reached:
                if bool(
                    self.get_parameter("main_road_tof_correction_enabled").value
                ):
                    self.main_road_tof_alignment_active = True
                    self.main_road_tof_angle_alignment_active = False
                    self.mission.set_phase(
                        MissionPhase.CORRECT_MAIN_ROAD_SOUTH,
                        now_s,
                    )
                    self.get_logger().info(
                        "Storage return reached the visit approach y; starting "
                        "south-wall ToF distance correction"
                    )
                else:
                    self.begin_storage_staging_return(now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CORRECT_MAIN_ROAD_SOUTH:
            return self.main_road_tof_command(
                target_y=staging_y,
                transit_speed=self.get_float("storage_return_speed"),
                storage_return=True,
                now_s=now_s,
            )

        if phase == MissionPhase.RETURN_STAGING:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                staging_x,
                staging_y,
                self.get_float("storage_return_speed"),
                final_yaw=math.pi,
            )
            if command.reached:
                if bool(self.get_parameter("storage_tof_correction_enabled").value):
                    self.storage_tof_coarse_heading_aligned = False
                    self.mission.set_phase(
                        MissionPhase.CORRECT_STORAGE_STAGING_X,
                        now_s,
                    )
                    self.mission_waypoint = (
                        staging_x,
                        staging_y,
                    )
                else:
                    return self.begin_storage_entry_open(now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CORRECT_STORAGE_STAGING_X:
            return self.storage_tof_axis_command("x", now_s)

        if phase == MissionPhase.CORRECT_STORAGE_STAGING_Y:
            return self.storage_tof_axis_command("y", now_s)

        if phase == MissionPhase.MOVE_TO_STORAGE_Y:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                self.return_lane_x,
                staging_y,
                self.get_float("storage_entry_speed"),
                final_yaw=entry_yaw,
                waypoint_tolerance=self.get_float("storage_entry_tolerance"),
            )
            if command.reached:
                if bool(self.get_parameter("storage_tof_correction_enabled").value):
                    self.storage_tof_coarse_heading_aligned = False
                    self.mission.set_phase(MissionPhase.CORRECT_STORAGE_Y, now_s)
                else:
                    self.mission.set_phase(MissionPhase.ALIGN_STORAGE_ENTRY, now_s)
                self.mission_waypoint = (
                    self.return_lane_x,
                    staging_y,
                )
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CORRECT_STORAGE_X:
            return self.storage_tof_axis_command("x", now_s)

        if phase == MissionPhase.CORRECT_STORAGE_Y:
            return self.storage_tof_axis_command("y", now_s)

        if phase == MissionPhase.ALIGN_STORAGE_ENTRY:
            self.set_control_mode(self.MODE_ENTER_STORAGE)
            self.mission_waypoint = (
                self.return_lane_x,
                staging_y,
            )
            command = waypoint_command(
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                target_x=self.robot_x,
                target_y=self.robot_y,
                speed=0.0,
                waypoint_tolerance=self.get_float("storage_entry_tolerance"),
                heading_tolerance=self.get_float("storage_heading_tolerance"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_max_angular_speed"),
                final_yaw=math.pi,
                final_yaw_tolerance=self.get_float("storage_final_yaw_tolerance"),
            )
            if command.reached:
                return self.begin_storage_entry_open(now_s)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.OPEN_STORAGE_ENTRY:
            return self.begin_storage_entry_open(now_s)

        if phase == MissionPhase.ALIGN_STORAGE_DASH:
            self.set_control_mode(self.MODE_ENTER_STORAGE)
            self.mission_waypoint = (
                self.get_float("storage_center_x"),
                self.get_float("storage_center_y"),
            )
            command = waypoint_command(
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                target_x=self.robot_x,
                target_y=self.robot_y,
                speed=0.0,
                waypoint_tolerance=self.get_float("storage_entry_tolerance"),
                heading_tolerance=self.get_float("storage_dash_heading_tolerance"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_max_angular_speed"),
                final_yaw=self.storage_entry_dash_yaw(),
                final_yaw_tolerance=self.get_float(
                    "storage_dash_heading_tolerance"
                ),
            )
            if command.reached:
                self.reset_storage_dash_timer()
                self.mission.set_phase(MissionPhase.ENTER_STORAGE, now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.ENTER_STORAGE:
            self.set_control_mode(self.MODE_ENTER_STORAGE)
            self.mission_waypoint = (
                self.get_float("storage_center_x"),
                self.get_float("storage_center_y"),
            )
            elapsed_s = self.update_storage_dash_timer(phase, now_s)
            entry_duration_s = self.active_storage_entry_dash_duration()
            command = fixed_heading_dash_command(
                robot_yaw=self.robot_yaw,
                desired_yaw=self.storage_entry_dash_yaw(),
                speed=self.get_float("storage_x_entry_speed"),
                elapsed_s=elapsed_s,
                duration_s=entry_duration_s,
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_dash_max_angular_speed"),
            )
            if not command.reached:
                return (command.linear_x, command.angular_z)
            if elapsed_s < (
                entry_duration_s
                + self.get_float("storage_contact_settle_duration_s")
            ):
                return (0.0, 0.0)
            return self.complete_storage_entry(now_s)

        if phase == MissionPhase.EXIT_STORAGE:
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            self.mission_waypoint = (
                staging_x,
                staging_y,
            )
            elapsed_s = self.update_storage_dash_timer(phase, now_s)
            command = fixed_heading_dash_command(
                robot_yaw=self.robot_yaw,
                desired_yaw=self.storage_entry_dash_yaw(),
                speed=-abs(self.get_float("storage_exit_reverse_speed")),
                elapsed_s=elapsed_s,
                duration_s=self.active_storage_exit_dash_duration(),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_dash_max_angular_speed"),
            )
            if command.reached:
                self.reset_storage_dash_timer()
                if storage_second_repush_required(self.storage_visit_number):
                    return self.begin_storage_second_repush_close(now_s)
                return self.begin_storage_exit_west_alignment(now_s)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CLOSE_STORAGE_REPUSH:
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            self.mission_waypoint = (
                self.get_float("storage_center_x"),
                self.get_float("storage_center_y"),
            )
            if self.mission.phase_age_s(now_s) < self.get_float(
                "gripper_move_duration_s"
            ):
                return (0.0, 0.0)
            self.reset_storage_dash_timer()
            self.mission.set_phase(MissionPhase.REPUSH_STORAGE, now_s)
            self.get_logger().info(
                "Second storage gripper close complete; starting slow repush"
            )
            return (0.0, 0.0)

        if phase == MissionPhase.REPUSH_STORAGE:
            self.set_control_mode(self.MODE_ENTER_STORAGE)
            self.mission_waypoint = (
                self.get_float("storage_center_x"),
                self.get_float("storage_center_y"),
            )
            elapsed_s = self.update_storage_dash_timer(phase, now_s)
            command = fixed_heading_dash_command(
                robot_yaw=self.robot_yaw,
                desired_yaw=self.storage_entry_dash_yaw(),
                speed=abs(self.get_float("storage_second_repush_speed")),
                elapsed_s=elapsed_s,
                duration_s=self.get_float("storage_second_repush_duration_s"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_dash_max_angular_speed"),
            )
            if command.reached:
                self.reset_storage_dash_timer()
                self.mission.set_phase(MissionPhase.EXIT_STORAGE_REPUSH, now_s)
                return (0.0, 0.0)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.EXIT_STORAGE_REPUSH:
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            self.mission_waypoint = (staging_x, staging_y)
            elapsed_s = self.update_storage_dash_timer(phase, now_s)
            command = fixed_heading_dash_command(
                robot_yaw=self.robot_yaw,
                desired_yaw=self.storage_entry_dash_yaw(),
                speed=-abs(self.get_float("storage_second_repush_speed")),
                elapsed_s=elapsed_s,
                duration_s=self.get_float("storage_second_repush_duration_s"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_dash_max_angular_speed"),
            )
            if command.reached:
                self.reset_storage_dash_timer()
                return self.begin_storage_side_waypoint_route(now_s)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.ALIGN_STORAGE_SIDE_WEST:
            return self.storage_side_west_alignment_command(now_s)

        if phase == MissionPhase.REVERSE_STORAGE_SIDE_CLEARANCE:
            return self.storage_side_clearance_reverse_command(now_s)

        if phase == MissionPhase.MOVE_STORAGE_SIDE_WAYPOINT:
            return self.storage_side_waypoint_command(now_s)

        if phase in (
            MissionPhase.ALIGN_STORAGE_EXIT_WEST,
            MissionPhase.ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH,
        ):
            self.set_control_mode(self.MODE_EXIT_STORAGE)
            self.mission_waypoint = (self.robot_x, self.robot_y)
            command = waypoint_command(
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                target_x=self.robot_x,
                target_y=self.robot_y,
                speed=0.0,
                waypoint_tolerance=self.get_float("storage_entry_tolerance"),
                heading_tolerance=self.get_float("storage_heading_tolerance"),
                heading_gain=self.get_float("storage_heading_gain"),
                max_angular_speed=self.get_float("storage_max_angular_speed"),
                final_yaw=math.pi,
                final_yaw_tolerance=self.get_float(
                    "storage_final_yaw_tolerance"
                ),
            )
            if command.reached:
                if phase == MissionPhase.ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH:
                    self.get_logger().info(
                        "Second storage repush return complete; west-facing "
                        "odometry alignment complete"
                    )
                    return self.continue_storage_exit_after_close(now_s)
                self.get_logger().info(
                    "Storage reverse complete; west-facing odometry alignment "
                    "complete, closing gripper"
                )
                return self.begin_storage_exit_close(now_s)
            return (command.linear_x, command.angular_z)

        if phase == MissionPhase.CORRECT_STORAGE_EXIT_X:
            return self.storage_exit_tof_command(now_s)

        if phase == MissionPhase.CLOSE_STORAGE_EXIT:
            return self.begin_storage_exit_close(now_s)

        if phase == MissionPhase.RETURN_FROM_STORAGE:
            self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
            command = self.storage_waypoint_command(
                bins,
                self.get_float("storage_exit_x"),
                self.get_float("storage_main_road_y"),
                self.get_float("storage_return_speed"),
                final_yaw=math.pi / 2.0,
            )
            if not command.reached:
                return (command.linear_x, command.angular_z)
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

    def storage_tof_axis_command(self, axis, now_s):
        axis = str(axis).strip().lower()
        staging_x, staging_y = self.active_storage_staging()
        staging_y_alignment = (
            axis == "y"
            and self.mission.phase == MissionPhase.CORRECT_STORAGE_STAGING_Y
        )
        if self.mission.phase_age_s(now_s) >= self.get_float(
            "tof_alignment_watchdog_s"
        ):
            self.get_logger().warning(
                f"Storage ToF {axis} alignment watchdog expired; continuing "
                "without applying a pose correction"
            )
            if axis == "x":
                return self.complete_storage_staging_x_alignment(now_s)
            if staging_y_alignment:
                return self.begin_storage_entry_open(now_s)
            self.mission.set_phase(MissionPhase.ALIGN_STORAGE_ENTRY, now_s)
            return (0.0, 0.0)

        if axis == "x":
            target_coordinate = staging_x
        elif staging_y_alignment:
            target_coordinate = staging_y
        else:
            target_coordinate = staging_y
        wall_coordinate = self.get_float(
            "storage_tof_left_wall_x_m"
            if axis == "x"
            else "storage_tof_bottom_wall_y_m"
        )
        distance_m, wall_angle_rad, measurement_age_s = (
            self.validated_wall_measurement("storage_tof_measurement_timeout_s")
        )
        if (
            not self.storage_tof_coarse_heading_aligned
            and storage_coarse_heading_is_aligned(
                self.robot_yaw,
                axis,
                self.get_float("storage_final_yaw_tolerance"),
            )
        ):
            self.storage_tof_coarse_heading_aligned = True
            self.get_logger().info(
                f"Storage ToF {axis} coarse odometry heading acquired; "
                "ToF angle is now authoritative"
            )
        command = make_storage_tof_command(
            axis=axis,
            distance_m=distance_m,
            measurement_age_s=measurement_age_s,
            robot_yaw=self.robot_yaw,
            target_coordinate=target_coordinate,
            wall_coordinate_m=wall_coordinate,
            sensor_forward_offset_m=self.get_float(
                "storage_tof_sensor_forward_offset_m"
            ),
            transit_speed=self.get_float(
                "storage_return_speed" if axis == "x" else "storage_entry_speed"
            ),
            minimum_speed=self.get_float("storage_tof_min_speed"),
            slowdown_distance_m=self.get_float("storage_tof_slowdown_distance_m"),
            coordinate_tolerance_m=self.get_float("storage_tof_xy_tolerance_m"),
            measurement_timeout_s=self.get_float("storage_tof_measurement_timeout_s"),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float("storage_max_angular_speed"),
            wall_angle_gain=self.get_float("storage_tof_angle_gain"),
            wall_angle_max_angular_speed=self.get_float(
                "storage_tof_max_angular_speed"
            ),
            heading_tolerance=self.get_float("storage_final_yaw_tolerance"),
            advance_without_measurement=False,
            wall_angle_rad=wall_angle_rad,
            wall_angle_tolerance_rad=self.get_float(
                "storage_tof_wall_angle_tolerance_rad"
            ),
            coarse_heading_aligned=self.storage_tof_coarse_heading_aligned,
        )
        self.mission_waypoint = (
            (
                staging_x
                if axis == "x" or staging_y_alignment
                else self.return_lane_x
            ),
            staging_y,
        )
        self.set_control_mode(self.MODE_RETURN_TO_STORAGE)
        if not command.reached:
            return (command.linear_x, command.angular_z)

        correction = Float64()
        correction.data = target_coordinate
        if not self.dry_run:
            if axis == "x":
                self.pose_x_correction_pub.publish(correction)
                self.pending_pose_x_correction = correction.data
                self.pending_pose_x_correction_time = self.get_clock().now()
            else:
                self.pose_y_correction_pub.publish(correction)
                self.pending_pose_y_correction = correction.data
                self.pending_pose_y_correction_time = self.get_clock().now()
        corrected_yaw = desired_yaw_for_storage_axis(axis)
        self.publish_pose_yaw_correction(corrected_yaw)

        if axis == "y" and not staging_y_alignment:
            self.mission.set_phase(MissionPhase.ALIGN_STORAGE_ENTRY, now_s)
        self.get_logger().info(
            f"Storage ToF {axis} alignment complete: "
            f"measured_{axis}={command.measured_coordinate:.3f}, "
            f"wall_angle={math.degrees(wall_angle_rad):.2f} deg, "
            f"pose_{axis}->{correction.data:.3f}, "
            f"pose_yaw->{math.degrees(corrected_yaw):.1f} deg"
        )
        if axis == "x":
            return self.complete_storage_staging_x_alignment(now_s)
        if staging_y_alignment:
            return self.begin_storage_entry_open(now_s)
        return (0.0, 0.0)

    def complete_storage_staging_x_alignment(self, now_s):
        staging_x, staging_y = self.active_storage_staging()
        next_phase = storage_phase_after_staging_x(self.return_lane_number)
        if next_phase == MissionPhase.CORRECT_STORAGE_STAGING_Y:
            self.storage_tof_coarse_heading_aligned = False
            self.mission.set_phase(MissionPhase.CORRECT_STORAGE_STAGING_Y, now_s)
            self.mission_waypoint = (
                staging_x,
                staging_y,
            )
            self.get_logger().info(
                f"Lane {self.return_lane_number} storage return: starting "
                "final south-wall ToF y alignment"
            )
            return (0.0, 0.0)
        return self.begin_storage_entry_open(now_s)

    def storage_exit_tof_command(self, now_s):
        _, staging_y = self.active_storage_staging()
        target_coordinate = self.get_float("storage_exit_x")
        if self.mission.phase_age_s(now_s) >= self.get_float(
            "tof_alignment_watchdog_s"
        ):
            self.get_logger().warning(
                "Storage-exit ToF watchdog expired; continuing without "
                "applying a pose correction"
            )
            return self.complete_storage_exit_tof(
                now_s,
                target_coordinate,
                measured_coordinate=None,
            )

        distance_m, wall_angle_rad, measurement_age_s = (
            self.validated_wall_measurement("storage_tof_measurement_timeout_s")
        )
        if (
            not self.storage_exit_tof_coarse_heading_aligned
            and storage_coarse_heading_is_aligned(
                self.robot_yaw,
                "x",
                self.get_float("storage_final_yaw_tolerance"),
            )
        ):
            self.storage_exit_tof_coarse_heading_aligned = True
            self.get_logger().info(
                "Storage-exit coarse west heading acquired; "
                "ToF angle is now authoritative"
            )
        command = make_storage_exit_tof_command(
            distance_m=distance_m,
            wall_angle_rad=wall_angle_rad,
            measurement_age_s=measurement_age_s,
            robot_yaw=self.robot_yaw,
            target_x=target_coordinate,
            west_wall_x_m=self.get_float("storage_tof_left_wall_x_m"),
            sensor_forward_offset_m=self.get_float(
                "storage_tof_sensor_forward_offset_m"
            ),
            transit_speed=self.get_float("storage_return_speed"),
            minimum_speed=self.get_float("storage_tof_min_speed"),
            slowdown_distance_m=self.get_float("storage_tof_slowdown_distance_m"),
            x_tolerance_m=self.get_float("storage_tof_xy_tolerance_m"),
            measurement_timeout_s=self.get_float("storage_tof_measurement_timeout_s"),
            angle_alignment_active=(
                self.storage_exit_tof_angle_alignment_active
            ),
            angle_trigger_rad=self.get_float(
                "storage_exit_tof_angle_trigger_rad"
            ),
            angle_release_rad=self.get_float(
                "storage_exit_tof_angle_release_rad"
            ),
            angle_gain=self.get_float("storage_tof_angle_gain"),
            max_angular_speed=self.get_float("storage_tof_max_angular_speed"),
            coarse_heading_gain=self.get_float("storage_heading_gain"),
            coarse_max_angular_speed=self.get_float(
                "storage_max_angular_speed"
            ),
            heading_tolerance=self.get_float("storage_final_yaw_tolerance"),
            coarse_heading_aligned=(
                self.storage_exit_tof_coarse_heading_aligned
            ),
        )
        self.storage_exit_tof_angle_alignment_active = (
            command.angle_alignment_active
        )
        self.mission_waypoint = (
            target_coordinate,
            staging_y,
        )
        self.set_control_mode(self.MODE_EXIT_STORAGE)
        if command.phase == "WAITING_FOR_STORAGE_TOF_X":
            if self.storage_exit_tof_missing_started_at is None:
                self.storage_exit_tof_missing_started_at = now_s
            if measurement_gap_timed_out(
                self.storage_exit_tof_missing_started_at,
                now_s,
                self.get_float("storage_exit_tof_fallback_timeout_s"),
            ):
                return self.complete_storage_exit_tof(
                    now_s,
                    target_coordinate,
                    measured_coordinate=None,
                )
        else:
            self.storage_exit_tof_missing_started_at = None
        if not command.reached:
            return (command.linear_x, command.angular_z)

        return self.complete_storage_exit_tof(
            now_s,
            target_coordinate,
            measured_coordinate=command.measured_robot_x,
        )

    def complete_storage_exit_tof(
        self,
        now_s,
        target_coordinate,
        measured_coordinate,
    ):
        if measured_coordinate is None:
            self.get_logger().warning(
                "No valid storage-exit ToF for the fallback timeout; "
                "continuing with odometry without changing pose or yaw"
            )
        else:
            correction = Float64()
            correction.data = target_coordinate
            if not self.dry_run:
                self.pose_x_correction_pub.publish(correction)
                self.pending_pose_x_correction = correction.data
                self.pending_pose_x_correction_time = self.get_clock().now()
            self.publish_pose_yaw_correction(math.pi)
            self.get_logger().info(
                "Storage exit west-wall correction complete: "
                f"measured_x={measured_coordinate:.3f}, "
                f"wall_angle={math.degrees(self.latest_wall_angle_rad):.2f} deg, "
                f"pose_x->{correction.data:.3f}, pose_yaw->180.0 deg"
            )
        self.storage_exit_tof_missing_started_at = None
        self.storage_exit_tof_angle_alignment_active = False
        self.storage_exit_tof_coarse_heading_aligned = False
        self.mission.set_phase(MissionPhase.RETURN_FROM_STORAGE, now_s)
        self.mission_waypoint = (
            self.get_float("storage_exit_x"),
            self.get_float("storage_main_road_y"),
        )
        return (0.0, 0.0)

    def storage_entry_dash_yaw(self):
        return storage_visit_dash_heading(
            self.storage_visit_number,
            self.get_float("storage_entry_dash_heading_deg"),
            self.get_float("storage_second_entry_dash_heading_deg"),
        )

    def reset_storage_dash_timer(self):
        self.storage_dash_timer_phase = None
        self.storage_dash_elapsed_s = 0.0
        self.storage_dash_last_update_s = None

    def pause_storage_dash_timer(self):
        self.storage_dash_last_update_s = None

    def update_storage_dash_timer(self, phase, now_s):
        phase = str(phase)
        now_s = float(now_s)
        if self.storage_dash_timer_phase != phase:
            self.storage_dash_timer_phase = phase
            self.storage_dash_elapsed_s = 0.0
            self.storage_dash_last_update_s = now_s
            return 0.0
        if self.storage_dash_last_update_s is None:
            self.storage_dash_last_update_s = now_s
            return self.storage_dash_elapsed_s
        self.storage_dash_elapsed_s += max(
            0.0,
            now_s - self.storage_dash_last_update_s,
        )
        self.storage_dash_last_update_s = now_s
        return self.storage_dash_elapsed_s

    def publish_pose_yaw_correction(self, corrected_yaw):
        corrected_yaw = normalize_angle(float(corrected_yaw))
        if self.dry_run:
            return
        correction = Float64()
        correction.data = corrected_yaw
        self.pose_yaw_correction_pub.publish(correction)
        self.pending_pose_yaw_correction = corrected_yaw
        self.pending_pose_yaw_correction_time = self.get_clock().now()

    def publish_pose_y_correction(self, corrected_y):
        corrected_y = float(corrected_y)
        if self.dry_run:
            return
        correction = Float64()
        correction.data = corrected_y
        self.pose_y_correction_pub.publish(correction)
        self.pending_pose_y_correction = corrected_y
        self.pending_pose_y_correction_time = self.get_clock().now()

    def correct_pose_at_storage_contact(self):
        corrected_x = self.get_float("storage_center_x")
        corrected_y = self.get_float("storage_center_y")
        if self.dry_run:
            return

        x_correction = Float64()
        x_correction.data = corrected_x
        y_correction = Float64()
        y_correction.data = corrected_y
        correction_time = self.get_clock().now()
        self.pose_x_correction_pub.publish(x_correction)
        self.pose_y_correction_pub.publish(y_correction)
        self.pending_pose_x_correction = corrected_x
        self.pending_pose_x_correction_time = correction_time
        self.pending_pose_y_correction = corrected_y
        self.pending_pose_y_correction_time = correction_time

    def begin_storage_entry_open(self, now_s):
        self.storage_tof_coarse_heading_aligned = False
        self.publish_cmd(0.0, 0.0)
        self.command_gripper(open_gripper=True)
        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_DASH, now_s)
        self.mission_waypoint = (
            self.get_float("storage_center_x"),
            self.get_float("storage_center_y"),
        )
        self.get_logger().info(
            "Storage gripper open commanded; continuing without waiting"
        )
        return (0.0, 0.0)

    def begin_storage_exit_close(self, now_s):
        self.publish_cmd(0.0, 0.0)
        self.command_gripper(open_gripper=False)
        self.get_logger().info(
            "Storage gripper close commanded; continuing without waiting"
        )
        return self.continue_storage_exit_after_close(now_s)

    def begin_storage_second_repush_close(self, now_s):
        self.publish_cmd(0.0, 0.0)
        self.command_gripper(open_gripper=False)
        self.mission.set_phase(MissionPhase.CLOSE_STORAGE_REPUSH, now_s)
        self.mission_waypoint = (
            self.get_float("storage_center_x"),
            self.get_float("storage_center_y"),
        )
        self.get_logger().info(
            "Second storage reverse complete; closing gripper before slow repush"
        )
        return (0.0, 0.0)

    def begin_storage_side_waypoint_route(self, now_s):
        self.publish_cmd(0.0, 0.0)
        self.storage_side_curve_start = None
        self.mission.set_phase(MissionPhase.ALIGN_STORAGE_SIDE_WEST, now_s)
        self.mission_waypoint = (self.robot_x, self.robot_y)
        self.get_logger().info(
            "Second storage upper repush return complete; rotating west "
            "before the side-waypoint route"
        )
        return (0.0, 0.0)

    def storage_side_west_alignment_command(self, now_s):
        self.set_control_mode(self.MODE_EXIT_STORAGE)
        self.mission_waypoint = (self.robot_x, self.robot_y)
        command = waypoint_command(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            target_x=self.robot_x,
            target_y=self.robot_y,
            speed=0.0,
            waypoint_tolerance=self.get_float("storage_entry_tolerance"),
            heading_tolerance=self.get_float("storage_heading_tolerance"),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float(
                "storage_second_side_align_max_angular_speed"
            ),
            final_yaw=math.pi,
            final_yaw_tolerance=self.get_float(
                "storage_final_yaw_tolerance"
            ),
        )
        if not command.reached:
            return (command.linear_x, command.angular_z)
        self.reset_storage_dash_timer()
        self.mission.set_phase(MissionPhase.REVERSE_STORAGE_SIDE_CLEARANCE, now_s)
        self.get_logger().info(
            "West alignment complete; reversing for 0.7 seconds before "
            "the side waypoint"
        )
        return (0.0, 0.0)

    def storage_side_clearance_reverse_command(self, now_s):
        self.set_control_mode(self.MODE_EXIT_STORAGE)
        self.mission_waypoint = (self.robot_x, self.robot_y)
        elapsed_s = self.update_storage_dash_timer(
            MissionPhase.REVERSE_STORAGE_SIDE_CLEARANCE,
            now_s,
        )
        command = fixed_heading_dash_command(
            robot_yaw=self.robot_yaw,
            desired_yaw=math.pi,
            speed=-abs(self.get_float("storage_second_side_shift_speed")),
            elapsed_s=elapsed_s,
            duration_s=self.get_float(
                "storage_second_side_reverse_duration_s"
            ),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float("storage_dash_max_angular_speed"),
        )
        if not command.reached:
            return (command.linear_x, command.angular_z)
        self.reset_storage_dash_timer()
        self.storage_side_curve_start = (self.robot_x, self.robot_y)
        self.mission.set_phase(MissionPhase.MOVE_STORAGE_SIDE_WAYPOINT, now_s)
        self.get_logger().info(
            "Timed west-facing reverse complete; moving to the side waypoint"
        )
        return (0.0, 0.0)

    def storage_side_waypoint_command(self, now_s):
        target_x = self.get_float("storage_second_side_target_x")
        target_y = self.get_float("storage_second_side_target_y")
        if self.storage_side_curve_start is None:
            self.storage_side_curve_start = (self.robot_x, self.robot_y)
        start_x, start_y = self.storage_side_curve_start
        waypoint_tolerance = self.get_float("storage_entry_tolerance")
        distance = math.hypot(
            target_x - self.robot_x,
            target_y - self.robot_y,
        )
        speed = tapered_waypoint_speed(
            distance=distance,
            fast_speed=self.get_float("storage_second_side_shift_speed"),
            slow_speed=self.get_float("storage_second_repush_speed"),
            slowdown_distance=self.get_float(
                "storage_second_side_slowdown_distance_m"
            ),
            waypoint_tolerance=waypoint_tolerance,
        )
        self.set_control_mode(self.MODE_EXIT_STORAGE)
        self.mission_waypoint = (target_x, target_y)
        command = curved_pose_waypoint_command(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            start_x=start_x,
            start_y=start_y,
            target_x=target_x,
            target_y=target_y,
            speed=speed,
            final_yaw=math.pi,
            waypoint_tolerance=waypoint_tolerance,
            heading_tolerance=self.get_float("storage_heading_tolerance"),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float("storage_max_angular_speed"),
            final_yaw_tolerance=self.get_float(
                "storage_dash_heading_tolerance"
            ),
            control_distance=self.get_float(
                "storage_second_side_curve_control_distance_m"
            ),
            lookahead_distance=self.get_float(
                "storage_second_side_curve_lookahead_distance_m"
            ),
        )
        if not command.reached:
            return (command.linear_x, command.angular_z)
        self.get_logger().info(
            "Second storage side waypoint reached; starting the existing "
            "west alignment and ToF x correction"
        )
        return self.begin_storage_exit_west_alignment(
            now_s,
            gripper_already_closed=True,
        )

    def continue_storage_exit_after_close(self, now_s):
        _, staging_y = self.active_storage_staging()
        if bool(self.get_parameter("storage_tof_correction_enabled").value):
            self.storage_exit_tof_missing_started_at = None
            self.storage_exit_tof_angle_alignment_active = False
            self.storage_exit_tof_coarse_heading_aligned = False
            self.mission.set_phase(MissionPhase.CORRECT_STORAGE_EXIT_X, now_s)
        else:
            self.mission.set_phase(MissionPhase.RETURN_FROM_STORAGE, now_s)
        self.mission_waypoint = (
            self.get_float("storage_exit_x"),
            staging_y,
        )
        return (0.0, 0.0)

    def begin_storage_exit_west_alignment(
        self,
        now_s,
        gripper_already_closed=False,
    ):
        self.publish_cmd(0.0, 0.0)
        phase = (
            MissionPhase.ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH
            if gripper_already_closed
            else MissionPhase.ALIGN_STORAGE_EXIT_WEST
        )
        self.mission.set_phase(phase, now_s)
        self.mission_waypoint = (self.robot_x, self.robot_y)
        if gripper_already_closed:
            self.get_logger().info(
                "Second storage repush return complete; rotating toward west "
                "with gripper closed"
            )
        else:
            self.get_logger().info(
                "Storage reverse complete; rotating toward west with odometry "
                "before closing gripper"
            )
        return (0.0, 0.0)

    def complete_storage_entry(self, now_s):
        staging_x, staging_y = self.active_storage_staging()
        self.publish_cmd(0.0, 0.0)
        self.reset_storage_dash_timer()
        self.correct_pose_at_storage_contact()
        deposited_count = self.mission.onboard_count
        self.mission.record_deposit(now_s)
        self.mission_waypoint = (staging_x, staging_y)
        self.get_logger().info(
            f"Deposited {deposited_count} object(s); "
            f"total delivered={self.mission.delivered_count}; "
            f"pose corrected to ({self.get_float('storage_center_x'):.3f}, "
            f"{self.get_float('storage_center_y'):.3f}); "
            "reversing with gripper open"
        )
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
        tolerance = (
            self.get_float("storage_waypoint_tolerance")
            if waypoint_tolerance is None
            else float(waypoint_tolerance)
        )
        command = waypoint_command(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            target_x=target_x,
            target_y=target_y,
            speed=speed,
            waypoint_tolerance=tolerance,
            heading_tolerance=self.get_float("storage_heading_tolerance"),
            heading_gain=self.get_float("storage_heading_gain"),
            max_angular_speed=self.get_float("storage_max_angular_speed"),
            final_yaw=final_yaw,
            final_yaw_tolerance=self.get_float("storage_final_yaw_tolerance"),
        )
        distance = math.hypot(
            float(target_x) - self.robot_x,
            float(target_y) - self.robot_y,
        )
        if distance <= tolerance:
            self.storage_curve_avoidance.reset()
            return command

        avoidance_required = waypoint_avoidance_required(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            target_x=target_x,
            target_y=target_y,
            waypoint_tolerance=tolerance,
            linear_x=command.linear_x,
            avoid_center=bins[1],
            danger_threshold=self.get_float("storage_avoid_danger_threshold"),
        )
        was_active = self.storage_curve_avoidance.active
        command = self.storage_curve_avoidance.command(
            now_s=self.now_s(),
            base_command=command,
            nominal_speed=speed,
            avoid_left=bins[0],
            avoid_center=bins[1],
            avoid_right=bins[2],
            allow_start=avoidance_required,
        )
        if not was_active and self.storage_curve_avoidance.active:
            side = "left" if self.storage_curve_avoidance.direction > 0.0 else "right"
            self.get_logger().info(
                "Storage-route obstacle detected; curving forward to the "
                f"{side} with the direction latched"
            )
        elif was_active and not self.storage_curve_avoidance.active:
            self.get_logger().info(
                "Storage-route obstacle cleared; resuming waypoint navigation"
            )
        return command

    def storage_pose_is_valid(self):
        pose_is_fresh = self.is_fresh(
            self.latest_pose_time,
            "pose_timeout_s",
        )
        if not pose_is_fresh:
            return False

        # Command-integrated odometry can continue through the physical storage
        # wall while the robot deliberately presses against it. The entry and
        # reverse motions are timed and only need a fresh yaw, so an out-of-bounds
        # x/y estimate must not freeze their timer before the contact correction.
        if not storage_pose_bounds_required(self.mission.phase):
            return True

        return pose_is_usable(
            self.robot_x,
            self.robot_y,
            self.get_float("arena_half_extent_m"),
            self.get_float("pose_bounds_tolerance_m"),
        )

    def resume_collection_after_storage(self):
        self.coverage_controller = self.create_coverage_controller(reverse_order=True)
        self.reset_lane_tof_alignment()
        self.reset_main_road_tof_alignment()
        self.reset_storage_tof_alignment()
        self.coverage_command = None
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_label = None
        self.latest_target_label_time = None
        self.reset_target_reacquisition()
        self.change_grab_state(self.GRAB_TRACKING)
        self.set_control_mode(self.MODE_COVERAGE_SEARCH)
        self.get_logger().info(
            "Storage return complete; scanning lanes 4->3->2->1 with "
            f"{self.mission.delivered_count}/{self.mission.target_object_count} delivered"
        )

    def create_coverage_controller(self, reverse_order=False):
        legs = generate_coverage_legs(
            min_x=self.get_float("coverage_min_x"),
            max_x=self.get_float("coverage_max_x"),
            main_road_y=self.get_float("coverage_main_road_y"),
            scan_end_y=self.get_float("coverage_scan_end_y"),
            lane_spacing=self.get_float("coverage_lane_spacing"),
            scan_speed=self.get_float("coverage_scan_speed"),
            transit_speed=self.get_float("coverage_transit_speed"),
            return_speed=self.get_float("coverage_return_speed"),
            reverse_order=reverse_order,
            first_entry_y=self.get_float("coverage_first_entry_y"),
        )
        controller = CoverageController(
            legs=legs,
            waypoint_tolerance=self.get_float("coverage_waypoint_tolerance"),
            heading_tolerance=self.get_float("coverage_heading_tolerance"),
            heading_gain=self.get_float("coverage_heading_gain"),
            max_angular_speed=self.get_float("coverage_max_angular_speed"),
            turn_in_place_threshold=self.get_float("coverage_turn_in_place_threshold"),
            avoid_danger_threshold=self.get_float("coverage_avoid_danger_threshold"),
            avoid_heading_tolerance=self.get_float(
                "coverage_avoid_heading_tolerance"
            ),
            avoid_angular_speed=self.get_float("coverage_avoid_angular_speed"),
            avoid_linear_scale=self.get_float("coverage_avoid_linear_scale"),
            rejoin_speed=self.get_float("coverage_rejoin_speed"),
            rejoin_coordinate_limit=self.get_float(
                "coverage_rejoin_coordinate_limit"
            ),
        )
        scan_lane_count = sum(leg.phase.startswith("SCAN_LANE") for leg in legs)
        self.get_logger().info(
            f"Coverage search ready with {len(legs)} legs "
            f"({scan_lane_count} scan lanes)"
        )
        return controller

    def update_target_history(self, target):
        if target is not None:
            detections = max(1, sum(self.target_visibility_history))
            self.target_reacquire_detection_count = max(
                self.target_reacquire_detection_count,
                detections,
            )
            if (
                self.control_mode == self.MODE_TRACK_TARGET
                or self.target_confirmation_is_met()
            ):
                self.target_reacquire_confirmed = True
            self.had_visible_target = True
            self.target_lost_started_at = None
            return
        if self.had_visible_target and self.target_lost_started_at is None:
            self.target_lost_started_at = self.get_clock().now()

    def should_locally_reacquire(self):
        if not self.had_visible_target or self.target_lost_started_at is None:
            return False
        elapsed_s = self.target_lost_age_s()
        duration_s = self.target_reacquire_duration_s()
        if duration_s > 0.0 and elapsed_s <= duration_s:
            return True
        self.reset_target_reacquisition()
        return False

    def target_reacquire_duration_s(self):
        return reacquire_duration_for_evidence(
            self.target_reacquire_detection_count,
            self.target_reacquire_confirmed,
            self.get_float("coverage_reacquire_single_detection_duration_s"),
            self.get_float("coverage_reacquire_two_detection_duration_s"),
            self.get_float("coverage_reacquire_duration_s"),
        )

    def reset_target_reacquisition(self):
        self.had_visible_target = False
        self.target_lost_started_at = None
        self.target_reacquire_detection_count = 0
        self.target_reacquire_confirmed = False

    def target_confirmation_is_met(self):
        return target_is_confirmed(
            self.target_visibility_history,
            self.target_confirmation_window,
            self.target_confirmation_min_detections,
        )

    def current_target_center_y(self):
        center_y = None
        if self.target_data_is_fresh(self.latest_target_center_y_time):
            center_y = self.latest_target_center_y
        return center_y

    def target_data_is_fresh(self, stamp):
        timeout_param = (
            "target_tracking_timeout_s"
            if self.control_mode == self.MODE_TRACK_TARGET
            else "target_timeout_s"
        )
        return self.is_fresh(stamp, timeout_param)

    def target_activation_is_met(self, tracking_active=None):
        if tracking_active is None:
            tracking_active = self.control_mode == self.MODE_TRACK_TARGET
        return target_is_eligible(
            self.current_target_center_y(),
            self.get_float("target_activation_center_y_min"),
            self.get_float("target_tracking_center_y_min"),
            tracking_active,
        )

    def target_search_is_allowed(self):
        """Block YOLO target pursuit on the main road until lane scanning starts."""
        leg = self.coverage_controller.current_leg
        coverage_allowed = coverage_phase_allows_target_search(
            leg.phase,
            coverage_enabled=self.coverage_is_enabled(),
            leave_start_active=self.leave_start_active,
            rejoin_active=self.coverage_controller.rejoin_active,
            main_road_alignment_active=self.main_road_tof_alignment_active,
        )
        return coverage_allowed and not self.storage_repickup_guard_is_active()

    def storage_repickup_guard_is_active(self):
        leg = self.coverage_controller.current_leg
        return storage_repickup_guard_is_active(
            enabled=self.get_parameter("storage_repickup_guard_enabled").value,
            delivered_count=self.mission.delivered_count,
            lane_number=leg.lane_number,
            coverage_phase=leg.phase,
            robot_y=self.robot_y,
            start_y=self.get_float("storage_repickup_guard_start_y"),
        )

    def begin_leave_start(self):
        self.leave_start_origin = None
        self.leave_start_yaw = None
        self.leave_start_traveled_m = 0.0
        self.leave_start_active = bool(self.get_parameter("leave_start_enabled").value)
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
            self.leave_start_yaw = math.radians(
                self.get_float("leave_start_target_yaw_deg")
            )
            self.get_logger().info(
                "Leaving start zone on a moving right-turn arc toward the first lane"
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
            heading_tolerance=self.get_float("leave_start_heading_tolerance"),
        )
        self.leave_start_traveled_m = command.traveled_m
        if command.complete:
            self.leave_start_active = False
            self.set_control_mode(self.MODE_COVERAGE_SEARCH)
            self.get_logger().info(
                "Start zone exit complete; beginning coverage search"
            )
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

        waiting_for_x = self.waiting_for_pose_x_correction()
        waiting_for_y = self.waiting_for_pose_y_correction()
        waiting_for_yaw = self.waiting_for_pose_yaw_correction()
        if waiting_for_x or waiting_for_y or waiting_for_yaw:
            self.coverage_command = self.coverage_controller.hold_command(
                "WAITING_FOR_POSE_LANDMARK_CORRECTION"
            )
            self.set_control_mode(self.MODE_COVERAGE_SEARCH)
            return (0.0, 0.0)

        leg_phase = self.coverage_controller.current_leg.phase
        main_road_tof_enabled = bool(
            self.get_parameter("main_road_tof_correction_enabled").value
        )
        main_road_waypoint_reached = (
            leg_phase == "SCAN_LANE_DOWN"
            and self.coverage_controller.current_leg_reached(
                self.robot_x,
                self.robot_y,
            )
        )
        run_main_road_tof = main_road_tof_enabled and (
            main_road_waypoint_reached or self.main_road_tof_alignment_active
        )
        if run_main_road_tof:
            if not self.main_road_tof_alignment_active:
                self.main_road_tof_alignment_active = True
                self.main_road_tof_angle_alignment_active = False
                self.get_logger().info(
                    "Lane-down waypoint reached; starting south-wall ToF "
                    "distance correction"
                )
            return self.main_road_tof_command(
                target_y=self.get_float("coverage_main_road_y"),
                transit_speed=self.get_float("coverage_return_speed"),
                storage_return=False,
            )

        lane_tof_enabled = bool(self.get_parameter("lane_tof_correction_enabled").value)
        waypoint_reached = (
            leg_phase == "SHIFT_TO_NEXT_LANE"
            and self.coverage_controller.current_leg_reached(
                self.robot_x,
                self.robot_y,
            )
        )
        run_lane_tof = should_run_lane_tof_fine_alignment(
            enabled=lane_tof_enabled,
            leg_phase=leg_phase,
            waypoint_reached=waypoint_reached,
            alignment_active=self.lane_tof_alignment_active,
        )
        if run_lane_tof:
            if not self.lane_tof_alignment_active:
                shift_leg = self.coverage_controller.current_leg
                wall_side = self.coverage_controller.current_shift_wall_side()
                wall_name = "east" if wall_side == "right" else "west"
                self.get_logger().info(
                    f"Lane {shift_leg.lane_number} waypoint reached; "
                    f"starting {wall_name}-wall ToF x fine alignment"
                )
            self.lane_tof_alignment_active = True
            return self.lane_tof_shift_command(self.now_s())
        self.reset_lane_tof_alignment()

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

    def lane_tof_shift_command(self, now_s):
        leg = self.coverage_controller.current_leg
        wall_side = self.coverage_controller.current_shift_wall_side()
        self.start_tof_watchdog("lane_tof_started_at_s", now_s)
        if self.tof_watchdog_expired("lane_tof_started_at_s", now_s):
            self.get_logger().warning(
                "Lane ToF alignment watchdog expired; continuing without "
                "applying a pose correction"
            )
            self.reset_lane_tof_alignment()
            self.coverage_controller.complete_current_leg("SHIFT_TO_NEXT_LANE")
            self.coverage_command = self.coverage_controller.hold_command(
                "LANE_TOF_WATCHDOG_FALLBACK"
            )
            return (0.0, 0.0)

        distance_m, wall_angle_rad, age_s = self.validated_wall_measurement(
            "lane_tof_measurement_timeout_s"
        )
        if (
            not self.lane_tof_coarse_heading_aligned
            and coarse_heading_is_aligned(
                self.robot_yaw,
                wall_side,
                self.get_float("coverage_heading_tolerance"),
            )
        ):
            self.lane_tof_coarse_heading_aligned = True
            self.get_logger().info(
                "Lane coarse odometry heading acquired; "
                "ToF angle is now authoritative"
            )
        wall_angle_dt_s = None
        if self.lane_tof_previous_angle_time_s is not None:
            wall_angle_dt_s = max(
                0.0,
                float(now_s) - self.lane_tof_previous_angle_time_s,
            )
        command = make_lane_tof_command(
            distance_m=distance_m,
            measurement_age_s=age_s,
            robot_yaw=self.robot_yaw,
            target_x=leg.target_x,
            left_wall_x_m=self.get_float("lane_tof_left_wall_x_m"),
            right_wall_x_m=self.get_float("lane_tof_right_wall_x_m"),
            wall_side=wall_side,
            sensor_forward_offset_m=self.get_float("lane_tof_sensor_forward_offset_m"),
            transit_speed=abs(float(leg.speed)),
            minimum_speed=self.get_float("lane_tof_min_speed"),
            slowdown_distance_m=self.get_float("lane_tof_slowdown_distance_m"),
            x_tolerance_m=self.get_float("lane_tof_x_tolerance_m"),
            measurement_timeout_s=self.get_float("lane_tof_measurement_timeout_s"),
            heading_gain=self.get_float("coverage_heading_gain"),
            max_angular_speed=self.get_float("coverage_max_angular_speed"),
            heading_tolerance=self.get_float("coverage_heading_tolerance"),
            wall_angle_rad=wall_angle_rad,
            wall_angle_tolerance_rad=self.get_float(
                "lane_tof_wall_angle_tolerance_rad"
            ),
            coarse_heading_aligned=self.lane_tof_coarse_heading_aligned,
            wall_angle_previous_rad=self.lane_tof_previous_angle_rad,
            wall_angle_dt_s=wall_angle_dt_s,
            wall_angle_kp=self.get_float("lane_tof_angle_kp"),
            wall_angle_kd=self.get_float("lane_tof_angle_kd"),
            wall_angle_max_angular_speed=self.get_float(
                "lane_tof_angle_max_angular_speed"
            ),
        )
        if wall_angle_rad is not None:
            self.lane_tof_previous_angle_rad = float(wall_angle_rad)
            self.lane_tof_previous_angle_time_s = float(now_s)
        self.coverage_command = self.coverage_controller.external_command(
            command.linear_x,
            command.angular_z,
            command.phase,
        )
        self.set_control_mode(self.MODE_COVERAGE_SEARCH)
        if not command.reached:
            return (command.linear_x, command.angular_z)

        correction = Float64()
        correction.data = float(leg.target_x)
        if not self.dry_run:
            self.pose_x_correction_pub.publish(correction)
            self.pending_pose_x_correction = correction.data
            self.pending_pose_x_correction_time = self.get_clock().now()
            self.publish_pose_yaw_correction(
                desired_yaw_for_wall(wall_side)
            )
        self.reset_lane_tof_alignment()
        self.coverage_controller.complete_current_leg("SHIFT_TO_NEXT_LANE")
        self.coverage_command = self.coverage_controller.hold_command(
            "TOF_LANE_ALIGNED"
        )
        self.get_logger().info(
            "ToF lane alignment complete: "
            f"lane={leg.lane_number}, "
            f"measured_x={command.measured_robot_x:.3f}, "
            f"wall_angle={math.degrees(wall_angle_rad):.2f} deg, "
            f"wall={'east' if wall_side == 'right' else 'west'}, "
            f"pose_x->{correction.data:.3f}"
        )
        return (0.0, 0.0)

    def wall_measurement_age_s(self):
        if self.latest_wall_measurement_time is None:
            return None
        age = self.get_clock().now() - self.latest_wall_measurement_time
        return max(0.0, age.nanoseconds / 1_000_000_000.0)

    def waiting_for_pose_x_correction(self):
        if self.pending_pose_x_correction is None:
            return False
        tolerance_parameter = (
            "storage_tof_xy_tolerance_m"
            if self.mission.is_storage_phase()
            else "lane_tof_x_tolerance_m"
        )
        tolerance = min(0.01, self.get_float(tolerance_parameter))
        if abs(self.robot_x - self.pending_pose_x_correction) <= tolerance:
            self.pending_pose_x_correction = None
            self.pending_pose_x_correction_time = None
            return False
        age = self.get_clock().now() - self.pending_pose_x_correction_time
        if age.nanoseconds / 1_000_000_000.0 <= self.get_float("pose_timeout_s"):
            return True
        self.get_logger().warning(
            "Timed out waiting for pose tracker to publish the x correction"
        )
        self.pending_pose_x_correction = None
        self.pending_pose_x_correction_time = None
        return False

    def waiting_for_pose_y_correction(self):
        if self.pending_pose_y_correction is None:
            return False
        tolerance_parameter = (
            "storage_tof_xy_tolerance_m"
            if self.mission.is_storage_phase()
            else "main_road_tof_y_tolerance_m"
        )
        tolerance = min(0.01, self.get_float(tolerance_parameter))
        if abs(self.robot_y - self.pending_pose_y_correction) <= tolerance:
            self.pending_pose_y_correction = None
            self.pending_pose_y_correction_time = None
            return False
        age = self.get_clock().now() - self.pending_pose_y_correction_time
        if age.nanoseconds / 1_000_000_000.0 <= self.get_float("pose_timeout_s"):
            return True
        self.get_logger().warning(
            "Timed out waiting for pose tracker to publish the y correction"
        )
        self.pending_pose_y_correction = None
        self.pending_pose_y_correction_time = None
        return False

    def waiting_for_pose_yaw_correction(self):
        if self.pending_pose_yaw_correction is None:
            return False
        tolerance_parameter = (
            "storage_tof_wall_angle_tolerance_rad"
            if self.mission.is_storage_phase()
            else "lane_tof_wall_angle_tolerance_rad"
        )
        tolerance = min(0.01, self.get_float(tolerance_parameter))
        yaw_error = normalize_angle(
            self.robot_yaw - self.pending_pose_yaw_correction
        )
        if abs(yaw_error) <= tolerance:
            self.pending_pose_yaw_correction = None
            self.pending_pose_yaw_correction_time = None
            return False
        age = self.get_clock().now() - self.pending_pose_yaw_correction_time
        if age.nanoseconds / 1_000_000_000.0 <= self.get_float("pose_timeout_s"):
            return True
        self.get_logger().warning(
            "Timed out waiting for pose tracker to publish the yaw correction"
        )
        self.pending_pose_yaw_correction = None
        self.pending_pose_yaw_correction_time = None
        return False

    def set_control_mode(self, mode):
        if mode == self.control_mode:
            return
        previous_mode = self.control_mode
        if mode == self.MODE_TRACK_TARGET or previous_mode == self.MODE_TRACK_TARGET:
            self.pickup_controller.reset()
        self.control_mode = mode
        if mode == self.MODE_TRACK_TARGET:
            self.filtered_action = [0.0, 0.0]
        self.get_logger().info(f"Control mode: {previous_mode} -> {mode}")

    def current_target(self):
        if not self.target_data_is_fresh(self.latest_target_time):
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

    def update_near_target_candidate(self, target):
        if not bool(self.get_parameter("near_target_loss_enabled").value):
            self.clear_near_target_candidate()
            return
        if not self.latest_target_visible:
            return
        if target is None or not self.is_fresh(
            self.latest_target_time,
            "grab_detection_timeout_s",
        ):
            self.clear_near_target_candidate()
            return

        near_threshold = max(
            0.0,
            self.get_float("grab_area_ratio")
            - self.get_float("near_target_loss_margin"),
        )
        if abs(float(target.x)) > self.get_float("grab_center_tolerance") or float(
            target.y
        ) < near_threshold:
            self.clear_near_target_candidate()
            return

        self.near_target_candidate = self.make_point(target.x, target.y, target.z)
        self.near_target_candidate_time = self.get_clock().now()
        self.near_target_candidate_label = self.current_target_label() or "unknown"

    def near_target_loss_pickup_ready(self):
        if (
            not bool(self.get_parameter("near_target_loss_enabled").value)
            or self.near_target_candidate is None
            or self.near_target_candidate_time is None
            or self.near_target_missing_started_at is None
            or self.latest_target_visible
        ):
            return False

        now = self.get_clock().now()
        candidate_age_s = (
            now - self.near_target_candidate_time
        ).nanoseconds / 1_000_000_000.0
        missing_age_s = (
            now - self.near_target_missing_started_at
        ).nanoseconds / 1_000_000_000.0
        return (
            candidate_age_s <= self.get_float("near_target_loss_timeout_s")
            and missing_age_s >= self.get_float("near_target_loss_min_missing_s")
        )

    def clear_near_target_candidate(self):
        self.near_target_candidate = None
        self.near_target_candidate_time = None
        self.near_target_candidate_label = None
        self.near_target_missing_started_at = None

    def update_grab_sequence(self, target, allow_near_target_loss=False):
        if not bool(self.get_parameter("gripper_enabled").value):
            return None

        if self.grab_state == self.GRAB_TRACKING:
            regular_pickup_ready = (
                target is not None
                and self.is_fresh(
                    self.latest_target_time,
                    "grab_detection_timeout_s",
                )
                and pickup_is_ready(
                    target_x=target.x,
                    target_y=target.y,
                    center_tolerance=self.get_float("grab_center_tolerance"),
                    grab_area_ratio=self.get_float("grab_area_ratio"),
                )
            )
            if not regular_pickup_ready and not allow_near_target_loss:
                return None

            if allow_near_target_loss and not regular_pickup_ready:
                self.pickup_label = self.near_target_candidate_label or "unknown"
                self.pickup_target_x = float(self.near_target_candidate.x)
                self.get_logger().warning(
                    "Near centered target disappeared at the gripper edge; "
                    "continuing the pickup sequence"
                )
            else:
                self.pickup_label = self.current_target_label() or "unknown"
                self.pickup_target_x = float(target.x)
            self.clear_near_target_candidate()
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
                    self.resume_coverage_after_pickup()
            else:
                self.mission.onboard_objects.append(label)
                self.resume_coverage_after_pickup()
            self.pickup_label = None
            self.change_grab_state(self.GRAB_COMPLETE)
            if not self.full_mission_is_enabled() and bool(
                self.get_parameter("stop_after_grab").value
            ):
                self.active = False
                self.get_logger().info("Object grabbed; deterministic drive stopped")
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
        self.reset_target_reacquisition()
        self.clear_near_target_candidate()

    def resume_coverage_after_pickup(self):
        preferred_turn = self.coverage_controller.prepare_resume_after_pickup(
            self.pickup_target_x,
            self.robot_y,
        )
        self.clear_target_after_pickup()
        self.coverage_controller.begin_rejoin(self.robot_y)
        if preferred_turn:
            direction = (
                "counterclockwise"
                if self.pickup_target_x > 0.0
                else "clockwise"
            )
            self.get_logger().info(
                "Lane-end pickup recorded; the 180-degree turn will rotate "
                f"{direction} to expose the opposite side"
            )
        self.pickup_target_x = None

    def current_avoid_objects(self, target):
        if self.is_fresh(self.latest_avoid_objects_time, "avoid_timeout_s"):
            objects = list(self.latest_avoid_objects)
        elif (
            self.is_fresh(self.latest_avoid_time, "avoid_timeout_s")
            and self.latest_avoid is not None
        ):
            objects = [self.latest_avoid]
        else:
            objects = []

        return objects

    def pickup_avoid_objects(self, objects):
        if not bool(self.get_parameter("avoid_roi_enabled").value):
            return list(objects)
        return [obj for obj in objects if self.avoid_is_inside_roi(obj)]

    def should_avoid_target_path(self, objects, target):
        if not bool(self.get_parameter("avoid_enabled").value):
            return False

        for obj in objects:
            closeness = float(obj.y)
            if not self.avoid_is_on_active_path(obj, target):
                continue
            if closeness >= self.get_float("avoid_emergency_ratio"):
                return True
            if closeness < self.get_float("avoid_area_ratio"):
                continue
            if (
                bool(self.get_parameter("avoid_only_if_closer_than_target").value)
                and target is not None
                and closeness
                < float(target.y) * self.get_float("avoid_closer_ratio")
            ):
                continue
            return True
        return False

    def avoid_is_on_active_path(self, obj, target):
        x = float(obj.x)
        closeness = float(obj.y)
        if abs(x) <= self.get_float("avoid_center_corridor"):
            return True
        if closeness >= self.get_float("avoid_emergency_ratio"):
            return True
        if target is None:
            return True

        margin = self.get_float("avoid_path_margin")
        left_limit = min(0.0, float(target.x)) - margin
        right_limit = max(0.0, float(target.x)) + margin
        return left_limit <= x <= right_limit

    def avoid_is_inside_roi(self, obj):
        return point_is_inside_trapezoid_roi(
            self.clamp(float(obj.x), -1.0, 1.0),
            self.clamp(float(getattr(obj, "center_y", obj.y)), 0.0, 1.0),
            left_far_x=self.get_float("avoid_roi_left_far_x"),
            left_far_y=self.get_float("avoid_roi_left_far_y"),
            left_near_x=self.get_float("avoid_roi_left_near_x"),
            left_near_y=self.get_float("avoid_roi_left_near_y"),
            right_far_x=self.get_float("avoid_roi_right_far_x"),
            right_far_y=self.get_float("avoid_roi_right_far_y"),
            right_near_x=self.get_float("avoid_roi_right_near_x"),
            right_near_y=self.get_float("avoid_roi_right_near_y"),
        )

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
            danger = self.clamp(
                obj.y * obj.y * (1.0 + center_weight * centered), 0.0, 1.0
            )
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
            self.get_logger().info(
                f"Gripper {action} skipped because gripper_enabled is false"
            )
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
        return self.grab_state_elapsed_offset_s + elapsed.nanoseconds / 1_000_000_000.0

    def current_target_label(self):
        if not self.target_data_is_fresh(self.latest_target_label_time):
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
        storage_staging_x, storage_staging_y = self.active_storage_staging()
        msg = String()
        msg.data = json.dumps(
            {
                "active": self.active,
                "motion_paused": self.motion_paused,
                "dry_run": self.dry_run,
                "model_loaded": False,
                "controller_ready": True,
                "controller_type": "deterministic_roi_pickup",
                "rl_enabled": False,
                "control_mode": self.control_mode,
                "timer_rate_hz": self.get_float("timer_rate_hz"),
                "observation_dim": self.model_observation_dim,
                "observation_names": OBSERVATION_NAMES[: self.model_observation_dim],
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
                "target_reacquisition": {
                    "detection_count": self.target_reacquire_detection_count,
                    "confirmed": self.target_reacquire_confirmed,
                    "duration_s": self.target_reacquire_duration_s(),
                    "reverse_enabled": self.target_reacquire_confirmed,
                },
                "target_activation": {
                    "center_y": self.current_target_center_y(),
                    "minimum_center_y": self.get_float(
                        "target_tracking_center_y_min"
                        if self.control_mode == self.MODE_TRACK_TARGET
                        else "target_activation_center_y_min"
                    ),
                    "eligible": (
                        self.target_search_is_allowed()
                        and self.target_activation_is_met()
                    ),
                    "entry_minimum_center_y": self.get_float(
                        "target_activation_center_y_min"
                    ),
                    "tracking_minimum_center_y": self.get_float(
                        "target_tracking_center_y_min"
                    ),
                    "tracking_hysteresis_active": (
                        self.control_mode == self.MODE_TRACK_TARGET
                    ),
                    "active_timeout_s": self.get_float(
                        "target_tracking_timeout_s"
                        if self.control_mode == self.MODE_TRACK_TARGET
                        else "target_timeout_s"
                    ),
                    "storage_repickup_guard_active": (
                        self.storage_repickup_guard_is_active()
                    ),
                },
                "pickup_controller": {
                    "state": self.pickup_controller.state,
                    "avoiding": self.pickup_controller.is_avoiding,
                    "center_tolerance": self.get_float(
                        "approach_center_tolerance"
                    ),
                    "approach_max_linear_x": self.get_float(
                        "approach_max_linear_x"
                    ),
                    "approach_min_linear_x": self.get_float(
                        "approach_min_linear_x"
                    ),
                    "approach_angular_gain": self.get_float(
                        "approach_angular_gain"
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
                    "force_return_remaining_s": (self.mission.force_return_remaining_s),
                    "storage_capacity": self.mission.storage_capacity,
                    "target_object_count": self.mission.target_object_count,
                    "onboard_count": self.mission.onboard_count,
                    "delivered_count": self.mission.delivered_count,
                    "total_collected_count": (self.mission.total_collected_count),
                    "storage_visit_number": self.storage_visit_number,
                    "storage_staging": {
                        "x": round(storage_staging_x, 4),
                        "y": round(storage_staging_y, 4),
                    },
                    "storage_dash_yaw_deg": round(
                        math.degrees(self.storage_entry_dash_yaw()),
                        3,
                    ),
                    "storage_entry_dash_duration_s": (
                        self.active_storage_entry_dash_duration()
                    ),
                    "storage_exit_dash_duration_s": (
                        self.active_storage_exit_dash_duration()
                    ),
                    "storage_second_repush_speed": self.get_float(
                        "storage_second_repush_speed"
                    ),
                    "storage_second_side_shift_speed": self.get_float(
                        "storage_second_side_shift_speed"
                    ),
                    "storage_second_side_reverse_duration_s": self.get_float(
                        "storage_second_side_reverse_duration_s"
                    ),
                    "storage_second_side_target_x": self.get_float(
                        "storage_second_side_target_x"
                    ),
                    "storage_second_side_target_y": self.get_float(
                        "storage_second_side_target_y"
                    ),
                    "storage_second_side_slowdown_distance_m": self.get_float(
                        "storage_second_side_slowdown_distance_m"
                    ),
                    "storage_second_side_align_max_angular_speed": self.get_float(
                        "storage_second_side_align_max_angular_speed"
                    ),
                    "storage_second_side_curve_control_distance_m": self.get_float(
                        "storage_second_side_curve_control_distance_m"
                    ),
                    "storage_second_side_curve_lookahead_distance_m": self.get_float(
                        "storage_second_side_curve_lookahead_distance_m"
                    ),
                    "storage_second_repush_duration_s": self.get_float(
                        "storage_second_repush_duration_s"
                    ),
                    "waypoint": (
                        None
                        if mission_waypoint is None
                        else {
                            "x": round(float(mission_waypoint[0]), 4),
                            "y": round(float(mission_waypoint[1]), 4),
                        }
                    ),
                },
                "storage_avoidance": self.storage_curve_avoidance.status(now_s),
                "obs": [round(v, 4) for v in obs[: self.model_observation_dim]],
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
                "pose": (
                    None
                    if not pose_fresh
                    else {
                        "x": round(self.robot_x, 4),
                        "y": round(self.robot_y, 4),
                        "yaw": round(self.robot_yaw, 4),
                    }
                ),
                "lane_tof": {
                    "enabled": bool(
                        self.get_parameter("lane_tof_correction_enabled").value
                    ),
                    "active": self.lane_tof_alignment_active,
                    "coarse_heading_aligned": (
                        self.lane_tof_coarse_heading_aligned
                    ),
                    "distance_m": (
                        None
                        if self.latest_wall_distance_m is None
                        else round(self.latest_wall_distance_m, 4)
                    ),
                    "angle_rad": (
                        None
                        if self.latest_wall_angle_rad is None
                        else round(self.latest_wall_angle_rad, 4)
                    ),
                    "age_s": (
                        None
                        if self.wall_measurement_age_s() is None
                        else round(self.wall_measurement_age_s(), 4)
                    ),
                    "pending_pose_x": self.pending_pose_x_correction,
                },
                "main_road_tof": {
                    "enabled": bool(
                        self.get_parameter("main_road_tof_correction_enabled").value
                    ),
                    "active": self.main_road_tof_alignment_active,
                    "angle_alignment_active": (
                        self.main_road_tof_angle_alignment_active
                    ),
                    "distance_m": (
                        None
                        if self.latest_wall_distance_m is None
                        else round(self.latest_wall_distance_m, 4)
                    ),
                    "angle_rad": (
                        None
                        if self.latest_wall_angle_rad is None
                        else round(self.latest_wall_angle_rad, 4)
                    ),
                    "pending_pose_y": self.pending_pose_y_correction,
                    "pending_pose_yaw": self.pending_pose_yaw_correction,
                },
                "storage_tof": {
                    "enabled": bool(
                        self.get_parameter("storage_tof_correction_enabled").value
                    ),
                    "exit_angle_alignment_active": (
                        self.storage_exit_tof_angle_alignment_active
                    ),
                    "coarse_heading_aligned": (
                        self.storage_tof_coarse_heading_aligned
                    ),
                    "exit_coarse_heading_aligned": (
                        self.storage_exit_tof_coarse_heading_aligned
                    ),
                    "active_axis": (
                        "x"
                        if self.mission.phase
                        in (
                            MissionPhase.CORRECT_STORAGE_STAGING_X,
                            MissionPhase.CORRECT_STORAGE_X,
                            MissionPhase.CORRECT_STORAGE_EXIT_X,
                        )
                        else (
                            "y"
                            if self.mission.phase
                            in (
                                MissionPhase.CORRECT_STORAGE_STAGING_Y,
                                MissionPhase.CORRECT_STORAGE_Y,
                            )
                            else None
                        )
                    ),
                    "distance_m": (
                        None
                        if self.latest_wall_distance_m is None
                        else round(self.latest_wall_distance_m, 4)
                    ),
                    "age_s": (
                        None
                        if self.wall_measurement_age_s() is None
                        else round(self.wall_measurement_age_s(), 4)
                    ),
                    "pending_pose_x": self.pending_pose_x_correction,
                    "pending_pose_y": self.pending_pose_y_correction,
                },
                "coverage": {
                    "enabled": self.coverage_is_enabled(),
                    "phase": None if coverage is None else coverage.phase,
                    "leg_index": None if coverage is None else coverage.leg_index,
                    "leg_count": len(self.coverage_controller.legs),
                    "waypoint_x": (
                        None if coverage is None else round(coverage.waypoint_x, 4)
                    ),
                    "waypoint_y": (
                        None if coverage is None else round(coverage.waypoint_y, 4)
                    ),
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
        return bool(self.get_parameter("pose_observation_enabled").value)

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
    def make_point(x, y, z, center_y=None, bottom_y=None):
        normalized_y = max(0.0, min(1.0, float(y)))
        return SimpleNamespace(
            x=max(-1.0, min(1.0, float(x))),
            y=normalized_y,
            z=max(0.0, float(z)),
            center_y=max(
                0.0,
                min(1.0, normalized_y if center_y is None else float(center_y)),
            ),
            bottom_y=max(
                0.0,
                min(1.0, normalized_y if bottom_y is None else float(bottom_y)),
            ),
        )

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = DeterministicMissionControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
