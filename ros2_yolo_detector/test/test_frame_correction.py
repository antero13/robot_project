import cv2
import numpy as np
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros2_yolo_detector.frame_correction import FrameCorrector


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


if __name__ == "__main__":
    unittest.main()
