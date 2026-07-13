import math

from mission_manager_2.mission_logic import (
    Pose2D,
    angular_error,
    pose_distance,
    select_target,
    target_is_large_enough,
    wall_matches_expected,
)


def detection_payload(detections):
    return {
        'image_width': 640,
        'image_height': 480,
        'detections': detections,
    }


def detection(class_name, confidence, xyxy, class_id=0):
    return {
        'class_id': class_id,
        'class_name': class_name,
        'confidence': confidence,
        'bbox_xyxy': dict(zip(('x1', 'y1', 'x2', 'y2'), xyxy)),
    }


def test_angular_error_wraps_across_pi():
    error = angular_error(math.radians(-179.0), math.radians(179.0))
    assert math.isclose(error, math.radians(2.0), abs_tol=1e-9)


def test_pose_distance_is_euclidean():
    assert pose_distance(Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 4.0, 1.0)) == 5.0


def test_select_target_prefers_near_centered_detection():
    payload = detection_payload([
        detection('apple', 0.9, (0, 100, 120, 260), class_id=1),
        detection('banana', 0.8, (260, 180, 380, 400), class_id=2),
    ])
    target = select_target(payload, set(), min_confidence=0.3)
    assert target.class_name == 'banana'
    assert math.isclose(target.x_error, 0.0, abs_tol=1e-9)
    assert math.isclose(target.bottom_y_ratio, 400 / 480)


def test_select_target_applies_allow_list_and_class_lock():
    payload = detection_payload([
        detection('apple', 0.9, (250, 100, 390, 350), class_id=1),
        detection('banana', 0.9, (250, 150, 390, 420), class_id=2),
    ])
    target = select_target(payload, {'apple'}, 0.3)
    assert target.class_name == 'apple'
    assert select_target(payload, set(), 0.3, locked_class='1').class_name == 'apple'


def test_target_size_requires_both_area_and_height():
    target = select_target(
        detection_payload([detection('apple', 0.9, (270, 200, 370, 320))]),
        set(),
        0.3,
    )
    assert target_is_large_enough(target, 0.008, 0.10)
    assert not target_is_large_enough(target, 0.05, 0.10)


def test_wall_measurement_must_match_expected_distance():
    assert wall_matches_expected(1.05, 1.0, 0.1)
    assert not wall_matches_expected(0.30, 1.0, 0.1)
    assert not wall_matches_expected(float('nan'), 1.0, 0.1)


def test_malformed_detection_payload_is_ignored():
    assert select_target('{not-json', set(), 0.3) is None
    assert select_target({'detections': []}, set(), 0.3) is None
