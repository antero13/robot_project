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
        self.assertIn('"source": inference_crop', self.source)
        self.assertNotIn('"source": [crop for _, crop in candidates]', self.source)

    def test_secondary_default_size_matches_exported_engine(self):
        self.assertIn('declare_parameter("secondary_imgsz", 800)', self.source)

    def test_final_class_names_match_dataset_order(self):
        expected_names = {
            0: "12",
            1: "20",
            2: "6",
            3: "8",
            4: "apple",
            5: "banana",
            6: "orange",
            7: "pineapple",
        }
        assignments = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "FINAL_CLASS_NAMES"
        }
        self.assertEqual(assignments["FINAL_CLASS_NAMES"], expected_names)

    def test_secondary_name_uses_final_class_mapping(self):
        self.assertIn(
            'detection["class_name"] = FINAL_CLASS_NAMES[final_class_id]',
            self.source,
        )

    def test_secondary_crop_uses_its_own_frame_correction(self):
        self.assertIn(
            "self.secondary_frame_corrector = self._create_frame_corrector(\n"
            "            self.secondary_imgsz",
            self.source,
        )
        self.assertIn(
            "corrected = self.secondary_frame_corrector.apply(crop)",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
