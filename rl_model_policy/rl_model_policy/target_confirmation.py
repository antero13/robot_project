def target_is_confirmed(
    visibility_history,
    window_size=5,
    minimum_detections=3,
):
    if window_size <= 0:
        raise ValueError("window_size must be positive")
    if minimum_detections <= 0 or minimum_detections > window_size:
        raise ValueError("minimum_detections must be between 1 and window_size")
    if len(visibility_history) < window_size:
        return False
    recent = list(visibility_history)[-window_size:]
    return sum(bool(value) for value in recent) >= minimum_detections
