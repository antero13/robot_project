import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rl_model_policy" / "rl_model_policy_node.py"
POLICY_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rl_model_policy.launch.py"
AUTONOMOUS_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rl_autonomous_drive.launch.py"


class DeterministicRuntimeConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_source = NODE_PATH.read_text(encoding="utf-8")
        cls.policy_launch_source = POLICY_LAUNCH_PATH.read_text(encoding="utf-8")
        cls.autonomous_launch_source = AUTONOMOUS_LAUNCH_PATH.read_text(
            encoding="utf-8"
        )

    @staticmethod
    def declared_launch_arguments(source):
        tree = ast.parse(source)
        return {
            ast.literal_eval(node.args[0])
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
        }

    @staticmethod
    def declared_launch_defaults(source):
        tree = ast.parse(source)
        defaults = {}
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "DeclareLaunchArgument"
                and node.args
            ):
                name = ast.literal_eval(node.args[0])
                default_node = next(
                    (
                        keyword.value
                        for keyword in node.keywords
                        if keyword.arg == "default_value"
                    ),
                    None,
                )
                if default_node is not None:
                    try:
                        defaults[name] = ast.literal_eval(default_node)
                    except (TypeError, ValueError):
                        pass
        return defaults

    def test_runtime_has_no_rl_model_loading_or_inference(self):
        for forbidden in (
            "import torch",
            "class PolicyNetwork",
            "def load_model",
            "def infer_action",
            'get_parameter("model_path")',
        ):
            self.assertNotIn(forbidden, self.node_source)

    def test_launches_have_no_rl_checkpoint_argument(self):
        policy_arguments = self.declared_launch_arguments(self.policy_launch_source)
        autonomous_arguments = self.declared_launch_arguments(
            self.autonomous_launch_source
        )
        self.assertNotIn("model_path", policy_arguments)
        self.assertNotIn("rl_model_path", autonomous_arguments)

    def test_launches_expose_deterministic_approach_controls(self):
        required = {
            "approach_center_tolerance",
            "approach_max_linear_x",
            "approach_min_linear_x",
            "approach_angular_gain",
            "approach_max_angular_z",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )
        self.assertIn(
            "executable='deterministic_mission_controller'",
            self.policy_launch_source,
        )

    def test_launches_expose_lane_avoidance_heading_tolerance(self):
        required = {"coverage_avoid_heading_tolerance"}
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )

    def test_launches_expose_candidate_reacquisition_durations(self):
        required = {
            "coverage_reacquire_single_detection_duration_s",
            "coverage_reacquire_two_detection_duration_s",
            "coverage_reacquire_duration_s",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )

    def test_launches_split_initial_entry_y_from_the_main_road(self):
        required = {
            "coverage_first_entry_y",
            "coverage_main_road_y",
            "storage_main_road_y",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )
        self.assertIn(
            'declare_parameter("coverage_first_entry_y", -1.3343)',
            self.node_source,
        )
        self.assertIn(
            'declare_parameter("coverage_main_road_y", -1.40)',
            self.node_source,
        )
        self.assertIn(
            'declare_parameter("storage_main_road_y", -1.40)',
            self.node_source,
        )

    def test_launches_expose_lane_tof_angle_pd_controls(self):
        required = {
            "lane_tof_angle_kp",
            "lane_tof_angle_kd",
            "lane_tof_angle_max_angular_speed",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )

    def test_launches_expose_storage_repickup_guard(self):
        required = {
            "storage_repickup_guard_enabled",
            "storage_repickup_guard_start_y",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )

    def test_storage_tof_completion_resets_yaw_for_the_corrected_axis(self):
        self.assertIn(
            "corrected_yaw = desired_yaw_for_storage_axis(axis)",
            self.node_source,
        )
        self.assertIn(
            "self.publish_pose_yaw_correction(corrected_yaw)",
            self.node_source,
        )

    def test_storage_rotation_limit_is_reduced_to_point_eight(self):
        self.assertIn(
            'declare_parameter("storage_max_angular_speed", 0.80)',
            self.node_source,
        )

    def test_all_tof_alignment_thresholds_are_four_degrees(self):
        names = (
            "lane_tof_wall_angle_tolerance_rad",
            "main_road_tof_angle_trigger_rad",
            "main_road_tof_angle_release_rad",
            "storage_tof_wall_angle_tolerance_rad",
            "storage_exit_tof_angle_trigger_rad",
            "storage_exit_tof_angle_release_rad",
        )
        for name in names:
            self.assertIn(f'"{name}", math.radians(4.0)', self.node_source)
        for source in (self.policy_launch_source, self.autonomous_launch_source):
            defaults = self.declared_launch_defaults(source)
            for name in names:
                self.assertEqual(defaults[name], "0.0698131701")

    def test_second_storage_repush_speed_is_point_thirteen(self):
        self.assertIn(
            'declare_parameter("storage_second_repush_speed", 0.13)',
            self.node_source,
        )
        for source in (
            self.policy_launch_source,
            self.autonomous_launch_source,
        ):
            tree = ast.parse(source)
            declarations = [
                node
                for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "DeclareLaunchArgument"
                and node.args
                and ast.literal_eval(node.args[0])
                == "storage_second_repush_speed"
            ]
            self.assertEqual(len(declarations), 1)
            default_value = next(
                keyword.value
                for keyword in declarations[0].keywords
                if keyword.arg == "default_value"
            )
            self.assertEqual(ast.literal_eval(default_value), "0.13")

    def test_launches_expose_second_storage_visit_route(self):
        required = {
            "storage_second_staging_x",
            "storage_second_staging_y",
            "storage_entry_dash_heading_deg",
            "storage_second_entry_dash_heading_deg",
            "storage_second_entry_dash_duration_s",
            "storage_second_exit_dash_duration_s",
            "storage_second_repush_speed",
            "storage_second_repush_duration_s",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )

    def test_launches_expose_pickup_vfh_motion_parameters(self):
        required = {
            "avoid_forward_linear_x",
            "avoid_escape_duration_s",
            "avoid_escape_linear_x",
            "avoid_escape_angular_z",
        }
        self.assertTrue(
            required.issubset(self.declared_launch_arguments(self.policy_launch_source))
        )
        self.assertTrue(
            required.issubset(
                self.declared_launch_arguments(self.autonomous_launch_source)
            )
        )


if __name__ == "__main__":
    unittest.main()
