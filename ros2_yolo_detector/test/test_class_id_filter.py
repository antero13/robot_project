import unittest

from ros2_yolo_detector.class_id_filter import (
    is_avoid_class_id,
    is_target_class_id,
    parse_class_ids,
)


class ClassIdFilterTest(unittest.TestCase):
    def test_parse_comma_separated_class_ids(self):
        self.assertEqual(parse_class_ids("0, 2,6"), {0, 2, 6})

    def test_parse_list_of_class_ids(self):
        self.assertEqual(parse_class_ids([0, "7"]), {0, 7})

    def test_rejects_class_names_and_prefixed_ids(self):
        for value in ("orange", "id:6", "6,apple"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "numeric class IDs only"):
                    parse_class_ids(value)

    def test_rejects_negative_ids(self):
        with self.assertRaisesRegex(ValueError, "non-negative"):
            parse_class_ids("-1")

    def test_final_id_six_selects_orange_not_geometric_six(self):
        target_ids = parse_class_ids("6")
        self.assertTrue(is_target_class_id(6, target_ids, set()))
        self.assertFalse(is_target_class_id(2, target_ids, set()))
        self.assertTrue(is_avoid_class_id(2, target_ids, set()))

    def test_final_id_two_selects_geometric_six_not_orange(self):
        target_ids = parse_class_ids("2")
        self.assertTrue(is_target_class_id(2, target_ids, set()))
        self.assertFalse(is_target_class_id(6, target_ids, set()))


if __name__ == "__main__":
    unittest.main()
