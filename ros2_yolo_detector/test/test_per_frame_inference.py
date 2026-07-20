import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "ros2_yolo_detector" / "yolo_camera_node.py"


class PerFrameInferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def test_inference_uses_predict_without_track(self):
        calls = {
            node.func.attr
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertIn("predict", calls)
        self.assertNotIn("track", calls)

    def test_inference_enables_configurable_class_agnostic_nms(self):
        self.assertIn('"agnostic_nms": self.agnostic_nms', self.source)

    def test_detector_has_no_cross_frame_tracking_state(self):
        forbidden_names = {
            "stable_tracks",
            "temporal_stabilizer",
            "tracker_enabled",
            "tracker_persist",
        }
        assigned_attributes = {
            node.attr
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "self"
        }
        self.assertTrue(forbidden_names.isdisjoint(assigned_attributes))

    def test_tracking_module_was_removed(self):
        tracking_module = PACKAGE_ROOT / "ros2_yolo_detector" / "temporal_stabilizer.py"
        self.assertFalse(tracking_module.exists())

    def test_secondary_inference_uses_single_crop_for_fixed_batch_engine(self):
        self.assertIn('"source": crop', self.source)
        self.assertNotIn('"source": [crop for _, crop in candidates]', self.source)

    def test_secondary_default_size_matches_exported_engine(self):
        self.assertIn('declare_parameter("secondary_imgsz", 800)', self.source)


if __name__ == "__main__":
    unittest.main()
