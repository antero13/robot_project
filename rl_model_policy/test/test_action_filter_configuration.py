import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "rl_model_policy" / "rl_model_policy_node.py"
POLICY_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rl_model_policy.launch.py"
AUTONOMOUS_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rl_autonomous_drive.launch.py"


class ActionFilterConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_source = NODE_PATH.read_text(encoding="utf-8")
        cls.node_tree = ast.parse(cls.node_source)
        cls.policy_launch_source = POLICY_LAUNCH_PATH.read_text(encoding="utf-8")
        cls.autonomous_launch_source = AUTONOMOUS_LAUNCH_PATH.read_text(
            encoding="utf-8"
        )

    def _declared_default(self, parameter_name):
        for node in ast.walk(self.node_tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != "declare_parameter" or len(node.args) < 2:
                continue
            if ast.literal_eval(node.args[0]) == parameter_name:
                return ast.literal_eval(node.args[1])
        self.fail(f"parameter is not declared: {parameter_name}")

    @staticmethod
    def _declared_launch_arguments(source):
        tree = ast.parse(source)
        return {
            ast.literal_eval(node.args[0])
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
        }

    def test_angular_filter_reverses_within_one_second_at_ten_hz(self):
        max_delta = self._declared_default("max_angular_action_delta")
        alpha = self._declared_default("angular_action_filter_alpha")
        reversal_time_s = 2.0 / (max_delta * alpha * 10.0)
        self.assertLess(reversal_time_s, 1.0)

    def test_linear_filter_keeps_its_existing_alpha(self):
        self.assertEqual(self._declared_default("action_filter_alpha"), 0.55)

    def test_autonomous_launch_exposes_angular_filter_controls(self):
        autonomous_arguments = self._declared_launch_arguments(
            self.autonomous_launch_source
        )
        policy_arguments = self._declared_launch_arguments(self.policy_launch_source)
        for name in ("max_angular_action_delta", "angular_action_filter_alpha"):
            self.assertIn(name, autonomous_arguments)
            self.assertIn(name, policy_arguments)
            self.assertIn(f'"{name}": {name}', self.autonomous_launch_source)


if __name__ == "__main__":
    unittest.main()
