from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoverageLeg:
    target_x: float
    target_y: float
    speed: float
    phase: str
    lane_number: int = 0
    wall_side: str = ""


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
    numbered_lane_positions = list(enumerate(lane_x_positions, start=1))
    if bool(reverse_order):
        numbered_lane_positions.reverse()

    # Each lane is centered between two object columns. The robot scans north,
    # turns around, scans south with its front camera, then shifts to the next
    # pair of columns along the obstacle-free lower road.
    legs = [
        CoverageLeg(
            numbered_lane_positions[0][1],
            main_road_y,
            transit_speed,
            "ENTER_FIRST_LANE",
            lane_number=numbered_lane_positions[0][0],
        )
    ]
    for index, (lane_number, lane_x) in enumerate(numbered_lane_positions):
        legs.append(
            CoverageLeg(
                lane_x,
                scan_end_y,
                scan_speed,
                "SCAN_LANE_UP",
                lane_number=lane_number,
            )
        )
        legs.append(
            CoverageLeg(
                lane_x,
                main_road_y,
                return_speed,
                "SCAN_LANE_DOWN",
                lane_number=lane_number,
            )
        )
        if index + 1 < len(numbered_lane_positions):
            next_lane_number, next_lane_x = numbered_lane_positions[index + 1]
            wall_side = wall_side_for_lane_number(next_lane_number)
            legs.append(
                CoverageLeg(
                    next_lane_x,
                    main_road_y,
                    signed_lane_shift_speed(
                        current_x=lane_x,
                        target_x=next_lane_x,
                        wall_side=wall_side,
                        transit_speed=transit_speed,
                    ),
                    "SHIFT_TO_NEXT_LANE",
                    lane_number=next_lane_number,
                    wall_side=wall_side,
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
        avoid_heading_tolerance=0.14,
        avoid_angular_speed=0.35,
        avoid_linear_scale=0.65,
        rejoin_speed=0.20,
        rejoin_approach_angle=math.pi / 4.0,
        rejoin_blend_distance=0.45,
        rejoin_align_tolerance=0.12,
        rejoin_coordinate_limit=1.8,
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
        self.avoid_heading_tolerance = float(avoid_heading_tolerance)
        self.avoid_angular_speed = float(avoid_angular_speed)
        self.avoid_linear_scale = float(avoid_linear_scale)
        self.rejoin_speed = float(rejoin_speed)
        self.rejoin_approach_angle = float(rejoin_approach_angle)
        self.rejoin_blend_distance = float(rejoin_blend_distance)
        self.rejoin_align_tolerance = float(rejoin_align_tolerance)
        self.rejoin_coordinate_limit = float(rejoin_coordinate_limit)
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
        if not 0.0 < self.avoid_heading_tolerance <= self.turn_in_place_threshold:
            raise ValueError(
                "avoid_heading_tolerance must be in (0, turn_in_place_threshold]"
            )
        if not 0.0 < self.avoid_linear_scale <= 1.0:
            raise ValueError("avoid_linear_scale must be in (0, 1]")
        if self.rejoin_speed <= 0.0:
            raise ValueError("rejoin_speed must be positive")
        if not 0.0 < self.rejoin_approach_angle <= math.pi / 2.0:
            raise ValueError("rejoin_approach_angle must be in (0, pi/2]")
        if self.rejoin_blend_distance <= self.waypoint_tolerance:
            raise ValueError("rejoin_blend_distance must exceed waypoint_tolerance")
        if self.rejoin_align_tolerance <= 0.0:
            raise ValueError("rejoin_align_tolerance must be positive")
        if self.rejoin_coordinate_limit <= 0.0:
            raise ValueError("rejoin_coordinate_limit must be positive")

        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.rejoin_target_y = None
        self.rejoin_entry_aligned = False
        self.rejoin_perpendicular_active = False

    def reset(self):
        self.leg_index = 0
        self.cycle_count = 0
        self.last_avoid_direction = 1.0
        self.rejoin_target_y = None
        self.rejoin_entry_aligned = False
        self.rejoin_perpendicular_active = False

    def cancel_avoidance(self):
        # Continuous steering has no persistent avoidance state to reset.
        return None

    @property
    def rejoin_active(self):
        return self.rejoin_target_y is not None

    def begin_rejoin(self, robot_y):
        if not self.current_leg.phase.startswith("SCAN_LANE"):
            self.rejoin_target_y = None
            self.rejoin_entry_aligned = False
            self.rejoin_perpendicular_active = False
            return False
        self.rejoin_target_y = float(robot_y)
        self.rejoin_entry_aligned = False
        self.rejoin_perpendicular_active = False
        return True

    def cancel_rejoin(self):
        self.rejoin_target_y = None
        self.rejoin_entry_aligned = False
        self.rejoin_perpendicular_active = False

    @property
    def current_leg(self):
        return self.legs[self.leg_index]

    def current_leg_reached(self, robot_x, robot_y):
        """Check the current waypoint without advancing the coverage route."""
        leg = self.current_leg
        return (
            math.hypot(leg.target_x - robot_x, leg.target_y - robot_y)
            <= self.waypoint_tolerance
        )

    def current_shift_wall_side(self):
        """Return the wall assigned to the destination lane.

        Lanes are numbered from east to west. Lanes 1 and 2 use the east
        (right, +x) wall; lanes 3 and 4 use the west (left, -x) wall.
        """
        if self.current_leg.phase != "SHIFT_TO_NEXT_LANE":
            raise RuntimeError("current leg is not a lane shift")
        if self.current_leg.wall_side not in ("left", "right"):
            raise RuntimeError("current lane shift has no assigned wall")
        return self.current_leg.wall_side

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
            abs(linear_x) > 1e-9
            and abs(heading_error) <= self.avoid_heading_tolerance
            and leg.phase.startswith("SCAN_LANE")
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
        x_error = target_x - robot_x
        if abs(x_error) <= self.waypoint_tolerance:
            self.rejoin_target_y = None
            self.rejoin_entry_aligned = False
            self.rejoin_perpendicular_active = False
            return self.command(robot_x, robot_y, robot_yaw)

        scan_direction = 1.0 if self.current_leg.phase == "SCAN_LANE_UP" else -1.0
        if (
            not self.rejoin_perpendicular_active
            and curved_rejoin_exceeds_coordinate_limit(
                robot_x=robot_x,
                robot_y=robot_y,
                target_x=target_x,
                scan_direction=scan_direction,
                approach_angle=self.rejoin_approach_angle,
                blend_distance=self.rejoin_blend_distance,
                waypoint_tolerance=self.waypoint_tolerance,
                coordinate_limit=self.rejoin_coordinate_limit,
            )
        ):
            self.rejoin_perpendicular_active = True
            self.rejoin_entry_aligned = False

        if self.rejoin_perpendicular_active:
            return self._perpendicular_rejoin_command(
                robot_x,
                robot_y,
                robot_yaw,
                target_x,
                target_y,
            )

        scan_yaw = scan_direction * math.pi / 2.0
        robot_side = 1.0 if robot_x > target_x else -1.0
        approach_offset = (
            scan_direction * robot_side * self.rejoin_approach_angle
        )
        approach_yaw = normalize_angle(scan_yaw + approach_offset)

        if self.rejoin_entry_aligned:
            blend = clamp(
                abs(x_error) / self.rejoin_blend_distance,
                0.0,
                1.0,
            )
            desired_yaw = normalize_angle(scan_yaw + approach_offset * blend)
        else:
            desired_yaw = approach_yaw

        heading_error = normalize_angle(desired_yaw - robot_yaw)
        angular_z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if (
            not self.rejoin_entry_aligned
            and abs(heading_error) > self.rejoin_align_tolerance
        ):
            return self._make_rejoin_command(
                0.0,
                angular_z,
                "ALIGN_CURVED_REJOIN",
                target_x,
                target_y,
            )
        self.rejoin_entry_aligned = True
        return self._make_rejoin_command(
            self.rejoin_speed,
            angular_z,
            "CURVE_REJOIN_LANE",
            target_x,
            target_y,
        )

    def _perpendicular_rejoin_command(
        self,
        robot_x,
        robot_y,
        robot_yaw,
        target_x,
        target_y,
    ):
        desired_yaw = 0.0 if target_x > robot_x else math.pi
        heading_error = normalize_angle(desired_yaw - robot_yaw)
        angular_z = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if (
            not self.rejoin_entry_aligned
            and abs(heading_error) > self.rejoin_align_tolerance
        ):
            return self._make_rejoin_command(
                0.0,
                angular_z,
                "ALIGN_PERPENDICULAR_REJOIN",
                target_x,
                target_y,
            )
        self.rejoin_entry_aligned = True
        return self._make_rejoin_command(
            self.rejoin_speed,
            angular_z,
            "PERPENDICULAR_REJOIN_LANE",
            target_x,
            target_y,
        )

    def _advance_reached_legs(self, robot_x, robot_y):
        checked = 0
        while checked < len(self.legs):
            if not self.current_leg_reached(robot_x, robot_y):
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


def curved_rejoin_exceeds_coordinate_limit(
    *,
    robot_x,
    robot_y,
    target_x,
    scan_direction,
    approach_angle,
    blend_distance,
    waypoint_tolerance,
    coordinate_limit,
):
    """Predict whether the ideal curved rejoin reaches the arena safety limit."""
    robot_x = float(robot_x)
    robot_y = float(robot_y)
    target_x = float(target_x)
    scan_direction = 1.0 if float(scan_direction) >= 0.0 else -1.0
    approach_angle = float(approach_angle)
    blend_distance = float(blend_distance)
    waypoint_tolerance = float(waypoint_tolerance)
    coordinate_limit = float(coordinate_limit)
    if coordinate_limit <= 0.0:
        raise ValueError("coordinate_limit must be positive")
    if not 0.0 < approach_angle <= math.pi / 2.0:
        raise ValueError("approach_angle must be in (0, pi/2]")
    if blend_distance <= waypoint_tolerance or waypoint_tolerance <= 0.0:
        raise ValueError("blend_distance must exceed positive waypoint_tolerance")

    if max(abs(robot_x), abs(robot_y), abs(target_x)) >= coordinate_limit:
        return True

    x_error = abs(target_x - robot_x)
    if x_error <= waypoint_tolerance:
        return False

    straight_error = max(0.0, x_error - blend_distance)
    y_travel = straight_error / math.tan(approach_angle)

    curved_upper = min(x_error, blend_distance)
    if curved_upper > waypoint_tolerance:
        angle_per_metre = approach_angle / blend_distance
        upper_sine = math.sin(angle_per_metre * curved_upper)
        lower_sine = math.sin(angle_per_metre * waypoint_tolerance)
        y_travel += math.log(upper_sine / lower_sine) / angle_per_metre

    projected_y = robot_y + scan_direction * y_travel
    return abs(projected_y) >= coordinate_limit


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def wall_side_for_lane_number(lane_number):
    """Assign east wall to lanes 1-2 and west wall to lanes 3 onward."""
    lane_number = int(lane_number)
    if lane_number <= 0:
        raise ValueError("lane_number must be positive")
    return "right" if lane_number <= 2 else "left"


def signed_lane_shift_speed(
    *,
    current_x,
    target_x,
    wall_side,
    transit_speed,
):
    """Choose forward/reverse while keeping the assigned wall in front."""
    current_x = float(current_x)
    target_x = float(target_x)
    transit_speed = abs(float(transit_speed))
    wall_side = str(wall_side).strip().lower()
    if transit_speed <= 0.0:
        raise ValueError("transit_speed must be positive")
    if wall_side == "right":
        facing_x = 1.0  # East wall, yaw 0.
    elif wall_side == "left":
        facing_x = -1.0  # West wall, yaw pi.
    else:
        raise ValueError("wall_side must be 'left' or 'right'")
    shift_x = target_x - current_x
    if abs(shift_x) <= 1e-9:
        raise ValueError("lane shift requires different x coordinates")
    return math.copysign(transit_speed, shift_x * facing_x)


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))
