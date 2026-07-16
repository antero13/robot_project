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
    reverse_order=False,
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
    if bool(reverse_order):
        lane_x_positions.reverse()

    # Each lane is centered between two object columns. The robot scans north,
    # turns around, scans south with its front camera, then shifts to the next
    # pair of columns along the obstacle-free lower road.
    legs = [
        CoverageLeg(
            lane_x_positions[0],
            main_road_y,
            transit_speed,
            "ENTER_FIRST_LANE",
        )
    ]
    for index, lane_x in enumerate(lane_x_positions):
        legs.append(
            CoverageLeg(
                lane_x,
                scan_end_y,
                scan_speed,
                "SCAN_LANE_UP",
            )
        )
        legs.append(
            CoverageLeg(
                lane_x,
                main_road_y,
                return_speed,
                "SCAN_LANE_DOWN",
            )
        )
        if index + 1 < len(lane_x_positions):
            legs.append(
                CoverageLeg(
                    lane_x_positions[index + 1],
                    main_road_y,
                    transit_speed,
                    "SHIFT_TO_NEXT_LANE",
                )
            )
    return legs


class CoverageController:
    def __init__(
        self,
        legs,
        waypoint_tolerance=0.10,
        heading_tolerance=0.12,
        heading_gain=1.4,
        max_angular_speed=0.40,
        turn_in_place_threshold=0.65,
        avoid_danger_threshold=0.20,
        avoid_angular_speed=0.35,
        avoid_linear_scale=0.65,
        rejoin_speed=0.20,
    ):
        self.legs = list(legs)
        if not self.legs:
            raise ValueError("coverage requires at least one leg")
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.heading_tolerance = float(heading_tolerance)
        self.heading_gain = float(heading_gain)
        self.max_angular_speed = float(max_angular_speed)
        self.turn_in_place_threshold = float(turn_in_place_threshold)
        self.avoid_danger_threshold = float(avoid_danger_threshold)
        self.avoid_angular_speed = float(avoid_angular_speed)
        self.avoid_linear_scale = float(avoid_linear_scale)
        self.rejoin_speed = float(rejoin_speed)
        if self.waypoint_tolerance <= 0.0:
            raise ValueError("waypoint_tolerance must be positive")
        if self.heading_tolerance <= 0.0:
            raise ValueError("heading_tolerance must be positive")
        if self.heading_gain <= 0.0 or self.max_angular_speed <= 0.0:
            raise ValueError("heading controller values must be positive")
        if self.turn_in_place_threshold <= self.heading_tolerance:
            raise ValueError("turn_in_place_threshold must exceed heading_tolerance")
        if self.avoid_danger_threshold < 0.0 or self.avoid_angular_speed <= 0.0:
            raise ValueError("avoidance controller values are invalid")
        if not 0.0 < self.avoid_linear_scale <= 1.0:
            raise ValueError("avoid_linear_scale must be in (0, 1]")
        if self.rejoin_speed <= 0.0:
            raise ValueError("rejoin_speed must be positive")

        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.rejoin_target_y = None

    def reset(self):
        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.rejoin_target_y = None

    def cancel_avoidance(self):
        # Continuous steering has no persistent avoidance state to reset.
        return None

    @property
    def rejoin_active(self):
        return self.rejoin_target_y is not None

    def begin_rejoin(self, robot_y):
        if not self.current_leg.phase.startswith("SCAN_LANE"):
            self.rejoin_target_y = None
            return False
        self.rejoin_target_y = float(robot_y)
        return True

    def cancel_rejoin(self):
        self.rejoin_target_y = None

    @property
    def current_leg(self):
        return self.legs[self.leg_index]

    def current_shift_wall_side(self):
        """Return the wall faced while moving toward the current lane center."""
        if self.current_leg.phase != "SHIFT_TO_NEXT_LANE":
            raise RuntimeError("current leg is not a lane shift")
        previous_leg = self.legs[(self.leg_index - 1) % len(self.legs)]
        if self.current_leg.target_x > previous_leg.target_x:
            return "right"
        return "left"

    def complete_current_leg(self, expected_phase=None):
        """Advance one leg after an external position reference reaches it."""
        if expected_phase is not None and self.current_leg.phase != str(expected_phase):
            return False
        self._advance_leg()
        return True

    def hold_command(self, phase):
        return self._make_command(0.0, 0.0, phase)

    def external_command(self, linear_x, angular_z, phase):
        """Describe a command produced by a leg-specific external controller."""
        return self._make_command(linear_x, angular_z, phase)

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
        if self.rejoin_active:
            return self._rejoin_command(robot_x, robot_y, robot_yaw)
        self._advance_reached_legs(robot_x, robot_y)
        leg = self.current_leg

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

        if abs(heading_error) > self.turn_in_place_threshold:
            return self._make_command(0.0, angular_z, f"ALIGN_{leg.phase}")

        heading_speed_scale = max(
            0.55,
            1.0 - 0.45 * abs(heading_error) / self.turn_in_place_threshold,
        )
        linear_x = leg.speed * heading_speed_scale
        phase = leg.phase
        avoid_danger = max(
            float(avoid_left),
            float(avoid_center),
            float(avoid_right),
        )
        if (
            leg.phase.startswith("SCAN_LANE")
            and avoid_danger >= self.avoid_danger_threshold
        ):
            direction = self._avoid_direction(
                float(avoid_left),
                float(avoid_right),
            )
            danger_span = max(1e-6, 1.0 - self.avoid_danger_threshold)
            danger = clamp(
                (avoid_danger - self.avoid_danger_threshold) / danger_span,
                0.0,
                1.0,
            )
            steer = direction * self.avoid_angular_speed * (0.45 + 0.55 * danger)
            angular_z = clamp(
                angular_z + steer,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            linear_x *= self.avoid_linear_scale
            phase = "CURVE_AVOID_LEFT" if direction > 0.0 else "CURVE_AVOID_RIGHT"
        return self._make_command(linear_x, angular_z, phase)

    def _rejoin_command(self, robot_x, robot_y, robot_yaw):
        target_x = self.current_leg.target_x
        target_y = self.rejoin_target_y
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        if distance <= self.waypoint_tolerance:
            self.rejoin_target_y = None
            return self.command(robot_x, robot_y, robot_yaw)

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - robot_yaw)
        angular_z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if abs(heading_error) > self.heading_tolerance:
            return self._make_rejoin_command(
                0.0,
                angular_z,
                "ALIGN_REJOIN_LANE",
                target_x,
                target_y,
            )
        return self._make_rejoin_command(
            self.rejoin_speed,
            angular_z,
            "REJOIN_LANE",
            target_x,
            target_y,
        )

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
        self.leg_index += 1
        if self.leg_index >= len(self.legs):
            self.leg_index = 0
            self.cycle_count += 1

    def _avoid_direction(self, avoid_left, avoid_right):
        if avoid_left < avoid_right:
            self.last_avoid_direction = 1.0
        elif avoid_right < avoid_left:
            self.last_avoid_direction = -1.0
        return self.last_avoid_direction

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

    def _make_rejoin_command(
        self,
        linear_x,
        angular_z,
        phase,
        waypoint_x,
        waypoint_y,
    ):
        return CoverageCommand(
            linear_x=float(linear_x),
            angular_z=float(angular_z),
            phase=str(phase),
            leg_index=self.leg_index,
            waypoint_x=float(waypoint_x),
            waypoint_y=float(waypoint_y),
            cycle_count=self.cycle_count,
        )


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))
