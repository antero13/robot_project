from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoverageLeg:
    target_x: float
    target_y: float
    speed: float
    phase: str


@dataclass(frozen=True)
class CoverageCommand:
    linear_x: float
    angular_z: float
    phase: str
    leg_index: int
    waypoint_x: float
    waypoint_y: float
    cycle_count: int


def generate_coverage_legs(
    min_x,
    max_x,
    main_road_y,
    scan_end_y,
    lane_spacing,
    scan_speed,
    transit_speed,
    return_speed,
):
    min_x = float(min_x)
    max_x = float(max_x)
    main_road_y = float(main_road_y)
    scan_end_y = float(scan_end_y)
    lane_spacing = float(lane_spacing)
    scan_speed = float(scan_speed)
    transit_speed = float(transit_speed)
    return_speed = float(return_speed)

    if min_x > max_x:
        raise ValueError("coverage min_x cannot be greater than max_x")
    if scan_end_y <= main_road_y:
        raise ValueError("coverage scan_end_y must be above main_road_y")
    if lane_spacing <= 0.0:
        raise ValueError("coverage lane_spacing must be positive")
    if scan_speed <= 0.0 or transit_speed <= 0.0 or return_speed <= 0.0:
        raise ValueError("coverage speeds must be positive")

    lane_x_positions = []
    lane_x = max_x
    while lane_x >= min_x - 1e-9:
        lane_x_positions.append(max(min_x, lane_x))
        lane_x -= lane_spacing
    if lane_x_positions[-1] > min_x + 1e-9:
        lane_x_positions.append(min_x)

    # Boustrophedon coverage avoids retracing every lane in reverse. The robot
    # scans north on one lane, shifts sideways, then scans south on the next.
    legs = [
        CoverageLeg(
            lane_x_positions[0],
            main_road_y,
            transit_speed,
            "ENTER_FIRST_LANE",
        )
    ]
    scan_y = scan_end_y
    for index, lane_x in enumerate(lane_x_positions):
        upward = scan_y == scan_end_y
        legs.append(
            CoverageLeg(
                lane_x,
                scan_y,
                scan_speed if upward else return_speed,
                "SCAN_LANE_UP" if upward else "SCAN_LANE_DOWN",
            )
        )
        if index + 1 < len(lane_x_positions):
            legs.append(
                CoverageLeg(
                    lane_x_positions[index + 1],
                    scan_y,
                    transit_speed,
                    "SHIFT_TO_NEXT_LANE",
                )
            )
        scan_y = main_road_y if upward else scan_end_y
    return legs


class CoverageController:
    def __init__(
        self,
        legs,
        waypoint_tolerance=0.10,
        heading_tolerance=0.12,
        heading_gain=1.4,
        max_angular_speed=0.40,
        avoid_danger_threshold=0.20,
        avoid_angular_speed=0.35,
        avoid_turn_angle=0.55,
        avoid_pass_distance=0.45,
        avoid_forward_speed=0.16,
        max_avoid_attempts_per_leg=2,
        arena_half_extent=2.0,
        boundary_clearance=0.22,
    ):
        self.legs = list(legs)
        if not self.legs:
            raise ValueError("coverage requires at least one leg")
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.heading_tolerance = float(heading_tolerance)
        self.heading_gain = float(heading_gain)
        self.max_angular_speed = float(max_angular_speed)
        self.avoid_danger_threshold = float(avoid_danger_threshold)
        self.avoid_angular_speed = float(avoid_angular_speed)
        self.avoid_turn_angle = float(avoid_turn_angle)
        self.avoid_pass_distance = float(avoid_pass_distance)
        self.avoid_forward_speed = float(avoid_forward_speed)
        self.max_avoid_attempts_per_leg = int(max_avoid_attempts_per_leg)
        self.arena_half_extent = float(arena_half_extent)
        self.boundary_clearance = float(boundary_clearance)
        if self.waypoint_tolerance <= 0.0:
            raise ValueError("waypoint_tolerance must be positive")
        if self.heading_tolerance <= 0.0:
            raise ValueError("heading_tolerance must be positive")
        if self.heading_gain <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError("heading controller values must be positive")
        if self.avoid_danger_threshold < 0.0 or self.avoid_angular_speed <= 0.0:
            raise ValueError("avoidance controller values are invalid")
        if (
            self.avoid_turn_angle <= 0.0
            or self.avoid_pass_distance <= 0.0
            or self.avoid_forward_speed <= 0.0
            or self.max_avoid_attempts_per_leg <= 0
            or self.arena_half_extent <= self.boundary_clearance
            or self.boundary_clearance < 0.0
        ):
            raise ValueError("avoidance bypass values must be positive")

        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.avoid_phase = None
        self.avoid_target_yaw = 0.0
        self.avoid_pass_start = None
        self.avoid_attempts = {}

    def reset(self):
        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.avoid_attempts.clear()
        self.cancel_avoidance()

    def cancel_avoidance(self):
        self.avoid_phase = None
        self.avoid_target_yaw = 0.0
        self.avoid_pass_start = None

    @property
    def current_leg(self):
        return self.legs[self.leg_index]

    def command(
        self,
        robot_x,
        robot_y,
        robot_yaw,
        avoid_left=0.0,
        avoid_center=0.0,
        avoid_right=0.0,
    ):
        robot_x = float(robot_x)
        robot_y = float(robot_y)
        robot_yaw = float(robot_yaw)
        self._advance_reached_legs(robot_x, robot_y)
        leg = self.current_leg

        if self.avoid_phase is not None:
            return self._avoidance_command(robot_x, robot_y, robot_yaw)

        if float(avoid_center) >= self.avoid_danger_threshold:
            direction = self._avoid_direction(
                float(avoid_left),
                float(avoid_right),
                robot_x,
                robot_y,
                robot_yaw,
            )
            attempts = self.avoid_attempts.get(self.leg_index, 0) + 1
            self.avoid_attempts[self.leg_index] = attempts
            if attempts > self.max_avoid_attempts_per_leg:
                self._advance_leg()
                return self._make_command(0.0, 0.0, "SKIP_BLOCKED_LEG")
            self.avoid_phase = "TURN"
            self.avoid_target_yaw = normalize_angle(
                robot_yaw + direction * self.avoid_turn_angle
            )
            return self._avoidance_command(robot_x, robot_y, robot_yaw)

        dx = leg.target_x - robot_x
        dy = leg.target_y - robot_y
        travel_heading = math.atan2(dy, dx)
        desired_yaw = (
            travel_heading
            if leg.speed >= 0.0
            else normalize_angle(travel_heading + math.pi)
        )
        heading_error = normalize_angle(desired_yaw - robot_yaw)
        angular_z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if abs(heading_error) > self.heading_tolerance:
            return self._make_command(0.0, angular_z, f"ALIGN_{leg.phase}")
        return self._make_command(leg.speed, angular_z, leg.phase)

    def _advance_reached_legs(self, robot_x, robot_y):
        checked = 0
        while checked < len(self.legs):
            leg = self.current_leg
            distance = math.hypot(leg.target_x - robot_x, leg.target_y - robot_y)
            if distance > self.waypoint_tolerance:
                return
            self._advance_leg()
            checked += 1

    def _advance_leg(self):
        previous_leg = self.leg_index
        self.leg_index += 1
        if self.leg_index >= len(self.legs):
            self.leg_index = 0
            self.cycle_count += 1
            self.avoid_attempts.clear()
        else:
            self.avoid_attempts.pop(previous_leg, None)
        self.cancel_avoidance()

    def _avoidance_command(self, robot_x, robot_y, robot_yaw):
        if self.avoid_phase == "TURN":
            heading_error = normalize_angle(self.avoid_target_yaw - robot_yaw)
            if abs(heading_error) > self.heading_tolerance:
                return self._make_command(
                    0.0,
                    clamp(
                        self.heading_gain * heading_error,
                        -self.avoid_angular_speed,
                        self.avoid_angular_speed,
                    ),
                    "AVOID_TURN",
                )
            self.avoid_phase = "PASS"
            self.avoid_pass_start = (robot_x, robot_y)

        start_x, start_y = self.avoid_pass_start
        distance = math.hypot(robot_x - start_x, robot_y - start_y)
        if distance >= self.avoid_pass_distance:
            self.cancel_avoidance()
            return self._make_command(0.0, 0.0, "AVOID_COMPLETE")
        heading_error = normalize_angle(self.avoid_target_yaw - robot_yaw)
        return self._make_command(
            self.avoid_forward_speed,
            clamp(
                self.heading_gain * heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            ),
            "AVOID_PASS",
        )

    def _avoid_direction(
        self,
        avoid_left,
        avoid_right,
        robot_x,
        robot_y,
        robot_yaw,
    ):
        if avoid_left < avoid_right:
            self.last_avoid_direction = 1.0
        elif avoid_right < avoid_left:
            self.last_avoid_direction = -1.0
        preferred = self.last_avoid_direction
        alternative = -preferred
        if self._bypass_boundary_score(
            alternative, robot_x, robot_y, robot_yaw
        ) > self._bypass_boundary_score(
            preferred, robot_x, robot_y, robot_yaw
        ) + 0.05:
            self.last_avoid_direction = alternative
        return self.last_avoid_direction

    def _bypass_boundary_score(self, direction, robot_x, robot_y, robot_yaw):
        bypass_yaw = robot_yaw + float(direction) * self.avoid_turn_angle
        projected_x = robot_x + self.avoid_pass_distance * math.cos(bypass_yaw)
        projected_y = robot_y + self.avoid_pass_distance * math.sin(bypass_yaw)
        safe_limit = self.arena_half_extent - self.boundary_clearance
        return min(
            safe_limit - abs(projected_x),
            safe_limit - abs(projected_y),
        )

    def _make_command(self, linear_x, angular_z, phase):
        leg = self.current_leg
        return CoverageCommand(
            linear_x=float(linear_x),
            angular_z=float(angular_z),
            phase=str(phase),
            leg_index=self.leg_index,
            waypoint_x=leg.target_x,
            waypoint_y=leg.target_y,
            cycle_count=self.cycle_count,
        )


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))
