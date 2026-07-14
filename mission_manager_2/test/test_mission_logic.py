import math

from mission_manager_2.mission_logic import (
    Pose2D,
    angular_error,
    confirmed_side_target,
    main_road_remaining_distance,
    pose_distance,
    select_target,
    select_side_targets,
    target_observations,
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


def test_side_targets_only_use_left_and_right_cells_of_middle_row():
    observations = target_observations(
        detection_payload([
            detection('left', 0.9, (40, 170, 180, 310), class_id=1),
            detection('center', 0.9, (260, 170, 380, 310), class_id=2),
            detection('right', 0.9, (470, 170, 610, 310), class_id=3),
            detection('top', 0.9, (40, 20, 180, 120), class_id=4),
        ]),
        set(),
        0.3,
    )
    left, right = select_side_targets(observations, 0.008, 0.10)
    assert left.class_name == 'left'
    assert right.class_name == 'right'


def test_side_target_requires_three_votes_in_the_same_side_over_five_frames():
    observations = target_observations(
        detection_payload([
            detection('left', 0.9, (40, 170, 180, 310), class_id=1),
            detection('right', 0.9, (470, 170, 610, 310), class_id=2),
        ]),
        set(),
        0.3,
    )
    left, right = select_side_targets(observations, 0.008, 0.10)
    history = [
        (left, None),
        (None, right),
        (left, None),
        (None, None),
        (left, None),
    ]
    assert confirmed_side_target(history, 3, 5).class_name == 'left'

    split_votes = [
        (left, None),
        (None, right),
        (left, None),
        (None, right),
        (None, None),
    ]
    assert confirmed_side_target(split_votes, 3, 5) is None


def test_side_target_waits_until_five_frames_are_available():
    target = select_target(
        detection_payload([detection('left', 0.9, (40, 170, 180, 310))]),
        set(),
        0.3,
    )
    assert confirmed_side_target([(target, None)] * 3, 3, 5) is None


def test_wall_measurement_must_match_expected_distance():
    assert wall_matches_expected(1.05, 1.0, 0.1)
    assert not wall_matches_expected(0.30, 1.0, 0.1)
    assert not wall_matches_expected(float('nan'), 1.0, 0.1)


def test_main_road_distance_prefers_tof_and_falls_back_to_pose():
    remaining_from_wall = main_road_remaining_distance(
        pose_x=2.50,
        goal_x=2.25,
        front_sensor_offset=0.15,
        wall_distance=2.15,
    )
    remaining_from_pose = main_road_remaining_distance(
        pose_x=2.50,
        goal_x=2.25,
        front_sensor_offset=0.15,
        wall_distance=None,
    )
    assert math.isclose(remaining_from_wall, 0.05, abs_tol=1e-9)
    assert math.isclose(remaining_from_pose, 0.25, abs_tol=1e-9)


def test_malformed_detection_payload_is_ignored():
    assert select_target('{not-json', set(), 0.3) is None
    assert select_target({'detections': []}, set(), 0.3) is None
