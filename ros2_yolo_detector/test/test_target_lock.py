from types import SimpleNamespace

from ros2_yolo_detector.target_lock import (
    lock_from_candidate,
    select_locked_candidate,
)


def candidate(name, x, center_y, bbox, closeness=0.5):
    point = SimpleNamespace(point=SimpleNamespace(x=x, y=closeness, z=0.9))
    return (closeness, name, point, bbox, center_y)


def test_lock_keeps_same_spatial_target_when_another_is_closer():
    original = candidate("apple", -0.2, 0.5, (100, 100, 200, 200), 0.5)
    target_lock = lock_from_candidate(original)
    same_target = candidate("orange", -0.18, 0.52, (105, 103, 205, 203), 0.52)
    other_target = candidate("apple", 0.6, 0.7, (400, 300, 520, 440), 0.9)

    selected = select_locked_candidate(
        [same_target, other_target],
        target_lock,
        score=lambda item: item[0],
        iou_threshold=0.1,
        center_distance_threshold=0.2,
    )

    assert selected is same_target


def test_lock_does_not_jump_when_locked_target_disappears():
    original = candidate("apple", -0.2, 0.5, (100, 100, 200, 200))
    other_target = candidate("apple", 0.7, 0.7, (420, 300, 520, 420), 0.9)

    selected = select_locked_candidate(
        [other_target],
        lock_from_candidate(original),
        score=lambda item: item[0],
        iou_threshold=0.1,
        center_distance_threshold=0.2,
    )

    assert selected is None


def test_no_lock_selects_highest_scoring_candidate():
    first = candidate("apple", 0.0, 0.4, (100, 100, 200, 200), 0.4)
    second = candidate("orange", 0.1, 0.6, (200, 200, 300, 300), 0.8)

    selected = select_locked_candidate(
        [first, second],
        None,
        score=lambda item: item[0],
        iou_threshold=0.1,
        center_distance_threshold=0.2,
    )

    assert selected is second
