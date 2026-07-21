def parse_class_ids(value) -> set[int]:
    """Parse a ROS parameter containing comma-separated final class IDs."""
    if value is None:
        return set()

    items = value if isinstance(value, (list, tuple)) else str(value).split(",")
    class_ids = set()
    for item in items:
        text = str(item).strip()
        if not text:
            continue
        try:
            class_id = int(text)
        except ValueError as exc:
            raise ValueError(
                "Class filters accept final numeric class IDs only; "
                f"received {text!r}"
            ) from exc
        if class_id < 0:
            raise ValueError(f"Class IDs must be non-negative; received {class_id}")
        class_ids.add(class_id)
    return class_ids


def is_target_class_id(class_id: int, target_ids: set[int], avoid_ids: set[int]) -> bool:
    if not target_ids:
        return class_id not in avoid_ids
    return class_id in target_ids


def is_avoid_class_id(class_id: int, target_ids: set[int], avoid_ids: set[int]) -> bool:
    if avoid_ids:
        return class_id in avoid_ids
    return bool(target_ids) and class_id not in target_ids
