import pytest

from rl_model_policy.target_confirmation import target_is_confirmed


def test_requires_a_complete_window():
    assert not target_is_confirmed([True, True, True], 5, 3)


def test_confirms_three_detections_in_five_frames():
    assert target_is_confirmed([True, False, True, False, True], 5, 3)


def test_rejects_two_detections_in_five_frames():
    assert not target_is_confirmed([True, False, False, False, True], 5, 3)


def test_uses_only_the_most_recent_window():
    history = [True, True, True, False, False, False, False, True]
    assert not target_is_confirmed(history, 5, 3)


@pytest.mark.parametrize(
    ("window_size", "minimum_detections"),
    [(0, 1), (5, 0), (5, 6)],
)
def test_rejects_invalid_configuration(window_size, minimum_detections):
    with pytest.raises(ValueError):
        target_is_confirmed([], window_size, minimum_detections)
