from dataclasses import dataclass


@dataclass(frozen=True)
class TargetAlignmentCommand:
    angular_z: float
    error: float
    derivative: float


class TargetAlignmentPD:
    def __init__(
        self,
        proportional_gain=0.8,
        derivative_gain=0.12,
        derivative_limit=0.25,
        center_deadband=0.06,
        max_angular_z=0.45,
        minimum_dt_s=0.02,
    ):
        self.proportional_gain = float(proportional_gain)
        self.derivative_gain = float(derivative_gain)
        self.derivative_limit = float(derivative_limit)
        self.center_deadband = float(center_deadband)
        self.max_angular_z = float(max_angular_z)
        self.minimum_dt_s = float(minimum_dt_s)
        self._validate()
        self.reset()

    def _validate(self):
        if self.proportional_gain < 0.0:
            raise ValueError("proportional_gain must not be negative")
        if self.derivative_gain < 0.0:
            raise ValueError("derivative_gain must not be negative")
        if self.derivative_limit <= 0.0:
            raise ValueError("derivative_limit must be positive")
        if self.center_deadband < 0.0:
            raise ValueError("center_deadband must not be negative")
        if self.max_angular_z <= 0.0:
            raise ValueError("max_angular_z must be positive")
        if self.minimum_dt_s <= 0.0:
            raise ValueError("minimum_dt_s must be positive")

    def reset(self):
        self.previous_error = None
        self.previous_time_s = None

    def command(self, error, now_s, maximum_dt_s):
        error = float(error)
        now_s = float(now_s)
        maximum_dt_s = float(maximum_dt_s)
        if maximum_dt_s <= 0.0:
            raise ValueError("maximum_dt_s must be positive")

        previous_error = self.previous_error
        previous_time_s = self.previous_time_s
        self.previous_error = error
        self.previous_time_s = now_s

        derivative = 0.0
        if previous_error is not None and previous_time_s is not None:
            dt = now_s - previous_time_s
            if self.minimum_dt_s <= dt <= maximum_dt_s:
                derivative = clamp(
                    (error - previous_error) / dt,
                    -self.derivative_limit,
                    self.derivative_limit,
                )

        if abs(error) <= self.center_deadband:
            return TargetAlignmentCommand(0.0, error, derivative)

        control = (
            self.proportional_gain * error
            + self.derivative_gain * derivative
        )
        return TargetAlignmentCommand(
            angular_z=clamp(-control, -self.max_angular_z, self.max_angular_z),
            error=error,
            derivative=derivative,
        )


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))
