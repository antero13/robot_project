from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PickupCommand:
    linear_x: float
    angular_z: float
    state: str


class DeterministicPickupController:
    """ROI-avoidance and target-approach controller used after search."""

    TRACK = "TRACK"
    AVOID_TURN = "AVOID_TURN"
    AVOID_FORWARD = "AVOID_FORWARD"

    def __init__(
        self,
        *,
        center_tolerance=0.12,
        approach_max_linear_x=0.10,
        approach_min_linear_x=0.03,
        approach_angular_gain=0.8,
        approach_max_angular_z=0.45,
        grab_area_ratio=0.70,
        avoid_turn_duration_s=0.55,
        avoid_turn_angular_z=0.65,
        avoid_forward_duration_s=0.85,
        avoid_forward_linear_x=0.05,
        avoid_forward_angular_z=0.25,
        avoid_vfh_target_weight=0.60,
        avoid_vfh_switch_penalty=0.25,
        avoid_direction_hold_s=0.8,
    ):
        self.center_tolerance = max(0.0, float(center_tolerance))
        self.approach_max_linear_x = max(0.0, float(approach_max_linear_x))
        self.approach_min_linear_x = max(0.0, float(approach_min_linear_x))
        self.approach_angular_gain = max(0.0, float(approach_angular_gain))
        self.approach_max_angular_z = max(0.0, float(approach_max_angular_z))
        self.grab_area_ratio = max(1e-6, float(grab_area_ratio))
        self.avoid_turn_duration_s = max(0.0, float(avoid_turn_duration_s))
        self.avoid_turn_angular_z = max(0.0, float(avoid_turn_angular_z))
        self.avoid_forward_duration_s = max(0.0, float(avoid_forward_duration_s))
        self.avoid_forward_linear_x = max(0.0, float(avoid_forward_linear_x))
        self.avoid_forward_angular_z = max(0.0, float(avoid_forward_angular_z))
        self.avoid_vfh_target_weight = max(0.0, float(avoid_vfh_target_weight))
        self.avoid_vfh_switch_penalty = max(0.0, float(avoid_vfh_switch_penalty))
        self.avoid_direction_hold_s = max(0.0, float(avoid_direction_hold_s))
        self.reset()

    def reset(self):
        self.state = self.TRACK
        self.state_started_s = None
        self.avoid_turn_direction = 1.0
        self.last_avoid_direction = None
        self.last_avoid_started_s = None

    @property
    def is_avoiding(self):
        return self.state in (self.AVOID_TURN, self.AVOID_FORWARD)

    def command(
        self,
        *,
        now_s,
        target_x=None,
        target_y=None,
        avoid_required=False,
        avoid_bins=(0.0, 0.0, 0.0),
    ):
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")

        maneuver = self._avoidance_command(now_s)
        if maneuver is not None:
            return maneuver

        if target_x is None or target_y is None:
            return PickupCommand(0.0, 0.0, self.TRACK)

        target_x = self._clamp(float(target_x), -1.0, 1.0)
        target_y = self._clamp(float(target_y), 0.0, 1.0)
        if avoid_required:
            self.avoid_turn_direction = self._choose_turn_direction(
                avoid_bins, target_x, now_s
            )
            self.state = self.AVOID_TURN
            self.state_started_s = now_s
            self.last_avoid_direction = self.avoid_turn_direction
            self.last_avoid_started_s = now_s
            return PickupCommand(
                0.0,
                self.avoid_turn_direction * self.avoid_turn_angular_z,
                self.state,
            )

        angular_z = self._clamp(
            -self.approach_angular_gain * target_x,
            -self.approach_max_angular_z,
            self.approach_max_angular_z,
        )
        if abs(target_x) > self.center_tolerance:
            return PickupCommand(0.0, angular_z, self.TRACK)

        closeness_scale = self._clamp(target_y / self.grab_area_ratio, 0.0, 1.0)
        linear_x = max(
            self.approach_min_linear_x,
            self.approach_max_linear_x * (1.0 - closeness_scale),
        )
        return PickupCommand(linear_x, angular_z, self.TRACK)

    def _avoidance_command(self, now_s):
        if self.state == self.AVOID_TURN:
            elapsed_s = max(0.0, now_s - self.state_started_s)
            if elapsed_s < self.avoid_turn_duration_s:
                return PickupCommand(
                    0.0,
                    self.avoid_turn_direction * self.avoid_turn_angular_z,
                    self.state,
                )
            self.state = self.AVOID_FORWARD
            self.state_started_s = now_s

        if self.state == self.AVOID_FORWARD:
            elapsed_s = max(0.0, now_s - self.state_started_s)
            if elapsed_s < self.avoid_forward_duration_s:
                return PickupCommand(
                    self.avoid_forward_linear_x,
                    self.avoid_turn_direction * self.avoid_forward_angular_z,
                    self.state,
                )
            self.state = self.TRACK
            self.state_started_s = None
        return None

    def _choose_turn_direction(self, bins, target_x, now_s):
        left, center, right = (float(value) for value in bins)
        left_cost = left + center * 0.45
        right_cost = right + center * 0.45

        if target_x < -self.center_tolerance:
            right_cost += self.avoid_vfh_target_weight
        elif target_x > self.center_tolerance:
            left_cost += self.avoid_vfh_target_weight

        if (
            self.last_avoid_direction is not None
            and self.last_avoid_started_s is not None
            and now_s - self.last_avoid_started_s <= self.avoid_direction_hold_s
        ):
            if self.last_avoid_direction != 1.0:
                left_cost += self.avoid_vfh_switch_penalty
            if self.last_avoid_direction != -1.0:
                right_cost += self.avoid_vfh_switch_penalty

        return 1.0 if left_cost <= right_cost else -1.0

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))
