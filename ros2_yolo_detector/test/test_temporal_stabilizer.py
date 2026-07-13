from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros2_yolo_detector.temporal_stabilizer import TemporalDetectionStabilizer


def make_detection(
    class_id=0,
    class_name="pineapple",
    confidence=0.8,
    stable_track_id=1,
    bbox=(100.0, 100.0, 200.0, 200.0),
):
    detection = {
        "class_id": class_id,
        "class_name": class_name,
        "confidence": confidence,
        "bbox_xyxy": {
            "x1": bbox[0],
            "y1": bbox[1],
            "x2": bbox[2],
            "y2": bbox[3],
        },
    }
    if stable_track_id is not None:
        detection["stable_track_id"] = stable_track_id
    return detection


class TemporalDetectionStabilizerTest(unittest.TestCase):
    def test_locks_class_after_three_consistent_observations(self):
        stabilizer = TemporalDetectionStabilizer()

        stabilizer.update([make_detection()], 0.0)
        stabilizer.update([make_detection(confidence=0.7)], 0.1)
        detections = stabilizer.update([make_detection(confidence=0.6)], 0.2)

        self.assertTrue(detections[0]["class_locked"])
        self.assertEqual(detections[0]["class_name"], "pineapple")
        self.assertAlmostEqual(detections[0]["temporal_class_confidence"], 0.7)

    def test_single_high_confidence_class_flicker_does_not_change_lock(self):
        stabilizer = TemporalDetectionStabilizer()
        for timestamp in (0.0, 0.1, 0.2):
            stabilizer.update([make_detection()], timestamp)

        detections = stabilizer.update(
            [make_detection(class_id=1, class_name="banana", confidence=0.95)],
            0.3,
        )

        self.assertEqual(detections[0]["class_id"], 0)
        self.assertEqual(detections[0]["class_name"], "pineapple")
        self.assertEqual(detections[0]["observed_class_name"], "banana")
        self.assertTrue(detections[0]["class_stabilized"])

    def test_sustained_strong_alternative_class_can_replace_bad_lock(self):
        stabilizer = TemporalDetectionStabilizer()
        for timestamp in (0.0, 0.1, 0.2):
            stabilizer.update([make_detection()], timestamp)

        for timestamp in (0.3, 0.4, 0.5):
            detections = stabilizer.update(
                [make_detection(class_id=1, class_name="banana", confidence=0.9)],
                timestamp,
            )
            self.assertEqual(detections[0]["class_name"], "pineapple")

        detections = stabilizer.update(
            [make_detection(class_id=1, class_name="banana", confidence=0.9)],
            0.6,
        )

        self.assertEqual(detections[0]["class_id"], 1)
        self.assertEqual(detections[0]["class_name"], "banana")
        self.assertFalse(detections[0]["class_stabilized"])

    def test_holds_last_detection_for_brief_miss_then_expires(self):
        stabilizer = TemporalDetectionStabilizer(detection_hold_s=0.35)
        stabilizer.update([make_detection()], 1.0)

        held = stabilizer.update([], 1.2)
        expired = stabilizer.update([], 1.36)

        self.assertEqual(len(held), 1)
        self.assertTrue(held[0]["temporally_held"])
        self.assertAlmostEqual(held[0]["hold_age_s"], 0.2)
        self.assertEqual(expired, [])

    def test_class_lock_survives_gap_longer_than_detection_hold(self):
        stabilizer = TemporalDetectionStabilizer(
            detection_hold_s=0.35,
            track_retention_s=1.0,
        )
        for timestamp in (0.0, 0.1, 0.2):
            stabilizer.update([make_detection()], timestamp)

        self.assertEqual(stabilizer.update([], 0.7), [])
        detections = stabilizer.update(
            [make_detection(class_id=1, class_name="banana", confidence=0.9)],
            0.8,
        )

        self.assertEqual(detections[0]["class_name"], "pineapple")
        self.assertTrue(detections[0]["class_stabilized"])

    def test_does_not_hold_detection_without_a_track_id(self):
        stabilizer = TemporalDetectionStabilizer()
        current = stabilizer.update([make_detection(stable_track_id=None)], 0.0)

        self.assertFalse(current[0]["class_locked"])
        self.assertEqual(stabilizer.update([], 0.1), [])

    def test_suppresses_held_duplicate_when_new_track_overlaps(self):
        stabilizer = TemporalDetectionStabilizer()
        stabilizer.update([make_detection(stable_track_id=1)], 0.0)

        detections = stabilizer.update([make_detection(stable_track_id=2)], 0.1)

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["stable_track_id"], 2)

    def test_rejects_invalid_settings(self):
        invalid_settings = [
            {"history_size": 0},
            {"history_size": 3, "lock_min_votes": 4},
            {"lock_min_confidence": 1.1},
            {"switch_min_confidence": -0.1},
            {"detection_hold_s": -0.1},
            {"track_retention_s": 0.0},
        ]
        for settings in invalid_settings:
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    TemporalDetectionStabilizer(**settings)


if __name__ == "__main__":
    unittest.main()
