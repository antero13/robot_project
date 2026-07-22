from dataclasses import dataclass
import math


class MissionPhase:
    IDLE = "IDLE"
    COLLECTING = "COLLECTING"
    RETURN_MAIN_ROAD = "RETURN_MAIN_ROAD"
    CORRECT_MAIN_ROAD_SOUTH = "CORRECT_MAIN_ROAD_SOUTH"
    REJOIN_STORAGE_LANE = "REJOIN_STORAGE_LANE"
    RETURN_STAGING = "RETURN_STAGING"
    CORRECT_STORAGE_STAGING_X = "CORRECT_STORAGE_STAGING_X"
    CORRECT_STORAGE_STAGING_Y = "CORRECT_STORAGE_STAGING_Y"
    MOVE_TO_STORAGE_Y = "MOVE_TO_STORAGE_Y"
    CORRECT_STORAGE_X = "CORRECT_STORAGE_X"
    CORRECT_STORAGE_Y = "CORRECT_STORAGE_Y"
    ALIGN_STORAGE_ENTRY = "ALIGN_STORAGE_ENTRY"
    OPEN_STORAGE_ENTRY = "OPEN_STORAGE_ENTRY"
    ALIGN_STORAGE_DASH = "ALIGN_STORAGE_DASH"
    ENTER_STORAGE = "ENTER_STORAGE"
    EXIT_STORAGE = "EXIT_STORAGE"
    CLOSE_STORAGE_REPUSH = "CLOSE_STORAGE_REPUSH"
    REPUSH_STORAGE = "REPUSH_STORAGE"
    EXIT_STORAGE_REPUSH = "EXIT_STORAGE_REPUSH"
    ALIGN_STORAGE_EXIT_WEST = "ALIGN_STORAGE_EXIT_WEST"
    ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH = (
        "ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH"
    )
    CORRECT_STORAGE_EXIT_X = "CORRECT_STORAGE_EXIT_X"
    CLOSE_STORAGE_EXIT = "CLOSE_STORAGE_EXIT"
    RETURN_FROM_STORAGE = "RETURN_FROM_STORAGE"
    COMPLETE = "COMPLETE"
    TIMEOUT = "TIMEOUT"
    STOPPED = "STOPPED"

    STORAGE_PHASES = frozenset(
        {
            REJOIN_STORAGE_LANE,
            RETURN_MAIN_ROAD,
            CORRECT_MAIN_ROAD_SOUTH,
            RETURN_STAGING,
            CORRECT_STORAGE_STAGING_X,
            CORRECT_STORAGE_STAGING_Y,
            MOVE_TO_STORAGE_Y,
            CORRECT_STORAGE_X,
            CORRECT_STORAGE_Y,
            ALIGN_STORAGE_ENTRY,
            OPEN_STORAGE_ENTRY,
            ALIGN_STORAGE_DASH,
            ENTER_STORAGE,
            EXIT_STORAGE,
            CLOSE_STORAGE_REPUSH,
            REPUSH_STORAGE,
            EXIT_STORAGE_REPUSH,
            ALIGN_STORAGE_EXIT_WEST,
            ALIGN_STORAGE_EXIT_WEST_AFTER_REPUSH,
            CORRECT_STORAGE_EXIT_X,
            CLOSE_STORAGE_EXIT,
            RETURN_FROM_STORAGE,
        }
    )


class ReturnReason:
    CAPACITY = "CAPACITY"
    TARGET_COUNT = "TARGET_COUNT"
    TIME_LIMIT = "TIME_LIMIT"
    MANUAL = "MANUAL"


def storage_return_start_phase():
    """Return to the southern main road before approaching storage."""
    return MissionPhase.RETURN_MAIN_ROAD


def storage_phase_after_staging_x(lane_number):
    """Choose the final storage staging step for the return lane."""
    if int(lane_number) in (1, 2):
        return MissionPhase.CORRECT_STORAGE_STAGING_Y
    return MissionPhase.OPEN_STORAGE_ENTRY


def storage_visit_number(delivered_count):
    """Use the first route before any deposit and the second route afterward."""
    if int(delivered_count) < 0:
        raise ValueError("delivered_count cannot be negative")
    return 1 if int(delivered_count) == 0 else 2


def storage_staging_coordinates(
    visit_number,
    first_x,
    first_y,
    second_x,
    second_y,
):
    """Return the configured pre-dash staging point for a storage visit."""
    if int(visit_number) <= 0:
        raise ValueError("storage visit number must be positive")
    if int(visit_number) == 1:
        return (float(first_x), float(first_y))
    return (float(second_x), float(second_y))


def storage_visit_dash_heading(visit_number, first_heading_deg, second_heading_deg):
    """Return the configured fixed storage dash heading for a visit."""
    if int(visit_number) <= 0:
        raise ValueError("storage visit number must be positive")
    heading_deg = (
        first_heading_deg if int(visit_number) == 1 else second_heading_deg
    )
    return (math.radians(float(heading_deg)) + math.pi) % (2.0 * math.pi) - math.pi


def storage_second_repush_required(visit_number):
    """Run the extra closed-gripper push only on the second storage visit."""
    return int(visit_number) == 2


def storage_pose_bounds_required(phase):
    """Only require bounded x/y while storage motion depends on waypoints."""
    return phase not in {
        MissionPhase.ENTER_STORAGE,
        MissionPhase.EXIT_STORAGE,
        MissionPhase.REPUSH_STORAGE,
        MissionPhase.EXIT_STORAGE_REPUSH,
    }


@dataclass(frozen=True)
class NavigationCommand:
    linear_x: float
    angular_z: float
    reached: bool


class StorageCurveAvoidanceController:
    """Latch one steering direction while curving around a storage-route obstacle."""

    def __init__(
        self,
        danger_threshold,
        release_threshold,
        linear_scale,
        angular_speed,
        direction_hold_s,
        max_duration_s=1.5,
        clear_samples=3,
        max_angular_speed=1.0,
    ):
        self.danger_threshold = float(danger_threshold)
        self.release_threshold = float(release_threshold)
        self.linear_scale = float(linear_scale)
        self.angular_speed = abs(float(angular_speed))
        self.direction_hold_s = max(0.0, float(direction_hold_s))
        self.max_duration_s = max(
            self.direction_hold_s,
            float(max_duration_s),
        )
        self.clear_samples = max(1, int(clear_samples))
        self.max_angular_speed = abs(float(max_angular_speed))
        self.reset()

    def reset(self):
        self.active = False
        self.direction = 0.0
        self.started_at_s = None
        self.clear_count = 0

    def command(
        self,
        *,
        now_s,
        base_command,
        nominal_speed,
        avoid_left,
        avoid_center,
        avoid_right,
        allow_start,
    ):
        now_s = float(now_s)
        avoid_left = float(avoid_left)
        avoid_center = float(avoid_center)
        avoid_right = float(avoid_right)

        if base_command.reached:
            self.reset()
            return base_command

        if not self.active:
            if not allow_start or avoid_center < self.danger_threshold:
                return base_command
            self.active = True
            self.direction = 1.0 if avoid_left <= avoid_right else -1.0
            self.started_at_s = now_s
            self.clear_count = 0

        remaining_danger = max(avoid_left, avoid_center, avoid_right)
        if remaining_danger <= self.release_threshold:
            self.clear_count += 1
        else:
            self.clear_count = 0

        elapsed_s = max(0.0, now_s - self.started_at_s)
        if elapsed_s >= self.max_duration_s:
            self.reset()
            return base_command
        if (
            elapsed_s >= self.direction_hold_s
            and self.clear_count >= self.clear_samples
        ):
            self.reset()
            return base_command

        linear_x = float(nominal_speed) * self.linear_scale
        angular_z = self.direction * self.angular_speed
        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z),
        )
        return NavigationCommand(linear_x, angular_z, False)

    def status(self, now_s):
        age_s = None
        if self.active and self.started_at_s is not None:
            age_s = max(0.0, float(now_s) - self.started_at_s)
        return {
            "active": self.active,
            "direction": self.direction,
            "age_s": age_s,
            "clear_count": self.clear_count,
        }


class MissionCoordinator:
    def __init__(
        self,
        storage_capacity=4,
        target_object_count=7,
        mission_duration_s=180.0,
        force_return_remaining_s=30.0,
    ):
        self.storage_capacity = int(storage_capacity)
        self.target_object_count = int(target_object_count)
        self.mission_duration_s = float(mission_duration_s)
        self.force_return_remaining_s = float(force_return_remaining_s)
        self._validate()
        self.reset()

    def _validate(self):
        if self.storage_capacity <= 0:
            raise ValueError("storage_capacity must be positive")
        if self.target_object_count <= 0:
            raise ValueError("target_object_count must be positive")
        if self.mission_duration_s <= 0.0:
            raise ValueError("mission_duration_s must be positive")
        if not 0.0 <= self.force_return_remaining_s < self.mission_duration_s:
            raise ValueError(
                "force_return_remaining_s must be within the mission duration"
            )

    def reset(self):
        self.phase = MissionPhase.IDLE
        self.started_at_s = None
        self.phase_started_at_s = None
        self.return_reason = None
        self.onboard_objects = []
        self.delivered_objects = []

    def start(self, now_s):
        self.reset()
        self.started_at_s = float(now_s)
        self.set_phase(MissionPhase.COLLECTING, now_s)

    def stop(self, now_s):
        self.set_phase(MissionPhase.STOPPED, now_s)

    def set_phase(self, phase, now_s):
        self.phase = str(phase)
        self.phase_started_at_s = float(now_s)

    def phase_age_s(self, now_s):
        if self.phase_started_at_s is None:
            return 0.0
        return max(0.0, float(now_s) - self.phase_started_at_s)

    def elapsed_s(self, now_s):
        if self.started_at_s is None:
            return 0.0
        return max(0.0, float(now_s) - self.started_at_s)

    def remaining_s(self, now_s):
        return max(0.0, self.mission_duration_s - self.elapsed_s(now_s))

    @property
    def onboard_count(self):
        return len(self.onboard_objects)

    @property
    def delivered_count(self):
        return len(self.delivered_objects)

    @property
    def total_collected_count(self):
        return self.onboard_count + self.delivered_count

    def is_storage_phase(self):
        return self.phase in MissionPhase.STORAGE_PHASES

    def record_pickup(self, label, now_s):
        self.onboard_objects.append(str(label or "unknown"))
        reason = self.return_reason_for_progress(now_s)
        if reason is not None:
            self.begin_return(reason, now_s)
        return reason

    def return_reason_for_progress(self, now_s):
        if self.total_collected_count >= self.target_object_count:
            return ReturnReason.TARGET_COUNT
        if self.onboard_count >= self.storage_capacity:
            return ReturnReason.CAPACITY
        if (
            self.onboard_count > 0
            and self.remaining_s(now_s) <= self.force_return_remaining_s
        ):
            return ReturnReason.TIME_LIMIT
        return None

    def update_time(self, now_s, defer_storage_return=False):
        if self.started_at_s is None:
            return None
        if self.remaining_s(now_s) <= 0.0 and self.phase not in (
            MissionPhase.COMPLETE,
            MissionPhase.TIMEOUT,
        ):
            self.set_phase(MissionPhase.TIMEOUT, now_s)
            return MissionPhase.TIMEOUT
        if self.phase == MissionPhase.COLLECTING and not defer_storage_return:
            reason = self.return_reason_for_progress(now_s)
            if reason is not None:
                self.begin_return(reason, now_s)
                return reason
        return None

    def begin_return(self, reason, now_s):
        if self.onboard_count <= 0:
            return False
        self.return_reason = str(reason)
        self.set_phase(MissionPhase.RETURN_MAIN_ROAD, now_s)
        return True

    def record_deposit(self, now_s):
        self.delivered_objects.extend(self.onboard_objects)
        self.onboard_objects.clear()
        self.set_phase(MissionPhase.EXIT_STORAGE, now_s)

    def finish_storage_exit(self, now_s):
        self.return_reason = None
        if self.delivered_count >= self.target_object_count:
            self.set_phase(MissionPhase.COMPLETE, now_s)
            return MissionPhase.COMPLETE
        self.set_phase(MissionPhase.COLLECTING, now_s)
        return MissionPhase.COLLECTING


def waypoint_command(
    robot_x,
    robot_y,
    robot_yaw,
    target_x,
    target_y,
    speed,
    waypoint_tolerance=0.10,
    heading_tolerance=0.14,
    heading_gain=1.5,
    max_angular_speed=0.40,
    final_yaw=None,
    final_yaw_tolerance=0.12,
):
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    distance = math.hypot(dx, dy)

    if distance <= float(waypoint_tolerance):
        if final_yaw is None:
            return NavigationCommand(0.0, 0.0, True)
        final_error = normalize_angle(float(final_yaw) - float(robot_yaw))
        if abs(final_error) <= float(final_yaw_tolerance):
            return NavigationCommand(0.0, 0.0, True)
        return NavigationCommand(
            0.0,
            clamp(
                float(heading_gain) * final_error,
                -float(max_angular_speed),
                float(max_angular_speed),
            ),
            False,
        )

    travel_heading = math.atan2(dy, dx)
    desired_yaw = (
        travel_heading
        if float(speed) >= 0.0
        else normalize_angle(travel_heading + math.pi)
    )
    heading_error = normalize_angle(desired_yaw - float(robot_yaw))
    angular_z = clamp(
        float(heading_gain) * heading_error,
        -float(max_angular_speed),
        float(max_angular_speed),
    )
    linear_x = 0.0 if abs(heading_error) > float(heading_tolerance) else float(speed)
    return NavigationCommand(linear_x, angular_z, False)


def waypoint_avoidance_required(
    robot_x,
    robot_y,
    target_x,
    target_y,
    waypoint_tolerance,
    linear_x,
    avoid_center,
    danger_threshold,
):
    """Use obstacle avoidance only while translating toward a waypoint."""
    distance = math.hypot(
        float(target_x) - float(robot_x),
        float(target_y) - float(robot_y),
    )
    return (
        abs(float(linear_x)) > 1e-9
        and distance > float(waypoint_tolerance)
        and float(avoid_center) >= float(danger_threshold)
    )


def storage_dash_heading(heading_deg):
    """Convert the configured fixed storage-dash IMU heading to radians."""
    return normalize_angle(math.radians(float(heading_deg)))


def fixed_heading_dash_command(
    robot_yaw,
    desired_yaw,
    speed,
    elapsed_s,
    duration_s,
    heading_gain=1.5,
    max_angular_speed=0.30,
):
    """Drive continuously for a fixed time while holding only IMU yaw."""
    if float(elapsed_s) >= max(0.0, float(duration_s)):
        return NavigationCommand(0.0, 0.0, True)

    heading_error = normalize_angle(float(desired_yaw) - float(robot_yaw))
    return NavigationCommand(
        float(speed),
        clamp(
            float(heading_gain) * heading_error,
            -float(max_angular_speed),
            float(max_angular_speed),
        ),
        False,
    )


def reverse_storage_x_exit_command(
    robot_x,
    robot_yaw,
    exit_x,
    desired_yaw,
    reverse_speed,
    x_tolerance=0.04,
    heading_gain=1.5,
    max_angular_speed=0.30,
):
    """Reverse east out of storage while the chassis remains facing west."""
    if float(robot_x) >= float(exit_x) - float(x_tolerance):
        return NavigationCommand(0.0, 0.0, True)
    heading_error = normalize_angle(float(desired_yaw) - float(robot_yaw))
    return NavigationCommand(
        -abs(float(reverse_speed)),
        clamp(
            float(heading_gain) * heading_error,
            -float(max_angular_speed),
            float(max_angular_speed),
        ),
        False,
    )


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))
