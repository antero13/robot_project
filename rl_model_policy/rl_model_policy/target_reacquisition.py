def reacquire_angular_velocity(
    last_target_direction,
    elapsed_s,
    reverse_after_s,
    angular_speed,
    reverse_enabled=True,
):
    """Return a two-phase search turn around the last visible target side."""
    if elapsed_s < 0.0:
        raise ValueError("elapsed_s must be non-negative")
    if reverse_after_s < 0.0:
        raise ValueError("reverse_after_s must be non-negative")
    if angular_speed < 0.0:
        raise ValueError("angular_speed must be non-negative")

    direction = 1.0 if float(last_target_direction) >= 0.0 else -1.0
    if bool(reverse_enabled) and elapsed_s >= reverse_after_s:
        direction *= -1.0
    return -direction * float(angular_speed)


def reacquire_duration_for_evidence(
    detection_count,
    confirmed,
    single_detection_duration_s,
    two_detection_duration_s,
    confirmed_duration_s,
):
    """Choose a short one-way search unless the target was confirmed."""
    detection_count = int(detection_count)
    durations = (
        float(single_detection_duration_s),
        float(two_detection_duration_s),
        float(confirmed_duration_s),
    )
    if detection_count < 0:
        raise ValueError("detection_count must be non-negative")
    if any(duration < 0.0 for duration in durations):
        raise ValueError("reacquisition durations must be non-negative")
    if bool(confirmed):
        return durations[2]
    if detection_count >= 2:
        return durations[1]
    if detection_count == 1:
        return durations[0]
    return 0.0
