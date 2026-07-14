def reacquire_angular_velocity(
    last_target_direction,
    elapsed_s,
    reverse_after_s,
    angular_speed,
):
    """Return a two-phase search turn around the last visible target side."""
    if elapsed_s < 0.0:
        raise ValueError("elapsed_s must be non-negative")
    if reverse_after_s < 0.0:
        raise ValueError("reverse_after_s must be non-negative")
    if angular_speed < 0.0:
        raise ValueError("angular_speed must be non-negative")

    direction = 1.0 if float(last_target_direction) >= 0.0 else -1.0
    if elapsed_s >= reverse_after_s:
        direction *= -1.0
    return -direction * float(angular_speed)
