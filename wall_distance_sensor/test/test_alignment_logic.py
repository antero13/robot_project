import math

from wall_distance_sensor.alignment_logic import alignment_angular_command


def command(angle_deg):
    return alignment_angular_command(
        wall_angle_rad=math.radians(angle_deg),
        tolerance_rad=math.radians(2.0),
        gain=1.2,
        minimum_angular_z=0.05,
        maximum_angular_z=0.20,
    )


def test_alignment_command_stops_inside_tolerance():
    assert command(1.5) == 0.0
    assert command(-2.0) == 0.0


def test_alignment_command_preserves_correction_direction():
    assert command(5.0) > 0.0
    assert command(-5.0) < 0.0


def test_alignment_command_respects_minimum_and_maximum():
    assert math.isclose(command(2.1), 0.05)
    assert math.isclose(command(30.0), 0.20)
