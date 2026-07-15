import cv2
import numpy as np
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros2_yolo_detector.frame_correction import FrameCorrector, LetterboxTransform


class FrameCorrectorTest(unittest.TestCase):
    def test_disabled_correction_returns_original_frame(self):
        frame = np.full((20, 30, 3), 80, dtype=np.uint8)
        corrector = FrameCorrector(enabled=False)

        self.assertIs(corrector.apply(frame), frame)

    def test_correction_preserves_shape_dtype_and_neutral_color(self):
        frame = np.full((80, 120, 3), 160, dtype=np.uint8)
        corrector = FrameCorrector()

        corrected = corrector.apply(frame)

        self.assertEqual(corrected.shape, frame.shape)
        self.assertEqual(corrected.dtype, frame.dtype)
        channel_spread = corrected.max(axis=2) - corrected.min(axis=2)
        self.assertLessEqual(int(channel_spread.max()), 3)

    def test_chroma_gain_increases_color_saturation(self):
        frame = np.full((80, 120, 3), (70, 90, 180), dtype=np.uint8)
        baseline = FrameCorrector(chroma_gain=1.0).apply(frame)
        enhanced = FrameCorrector(chroma_gain=1.3).apply(frame)

        baseline_saturation = cv2.cvtColor(baseline, cv2.COLOR_BGR2HSV)[..., 1].mean()
        enhanced_saturation = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)[..., 1].mean()

        self.assertGreater(enhanced_saturation, baseline_saturation)

    def test_invalid_settings_are_rejected(self):
        invalid_settings = [
            {"gamma": 0},
            {"clahe_clip_limit": -1},
            {"clahe_tile_grid": 0},
            {"chroma_gain": 0},
        ]
        for settings in invalid_settings:
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    FrameCorrector(**settings)

    def test_letterbox_bbox_is_restored_to_original_coordinates(self):
        transform = LetterboxTransform.for_square(
            original_width=800,
            original_height=600,
            image_size=640,
        )

        restored = transform.to_original_bbox([80.0, 160.0, 400.0, 400.0])

        self.assertEqual(transform.resized_width, 640)
        self.assertEqual(transform.resized_height, 480)
        self.assertEqual(transform.pad_left, 0)
        self.assertEqual(transform.pad_top, 80)
        np.testing.assert_allclose(restored, [100.0, 100.0, 500.0, 400.0])

    def test_letterbox_bbox_is_clamped_to_original_frame(self):
        transform = LetterboxTransform.for_square(800, 600, 640)

        restored = transform.to_original_bbox([-10.0, 0.0, 700.0, 700.0])

        np.testing.assert_allclose(restored, [0.0, 0.0, 800.0, 600.0])


if __name__ == "__main__":
    unittest.main()
