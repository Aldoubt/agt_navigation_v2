import importlib.util
from pathlib import Path

from agt_interfaces.action import Relocalize


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "automatic_relocalization.py"
SPEC = importlib.util.spec_from_file_location("automatic_relocalization", SCRIPT)
AUTO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUTO)


def test_startup_goal_is_bounded_auto_search():
    goal = AUTO.make_startup_goal(12.5, 7, True)

    assert goal.mode == Relocalize.Goal.MODE_AUTO_SEARCH
    assert goal.use_last_valid_pose is True
    assert goal.use_configured_candidates is True
    assert goal.use_external_coarse_pose is True
    assert goal.max_candidates == 7
    assert goal.timeout_s == 12.5
    assert goal.publish_debug is True


def test_startup_goal_keeps_zero_as_node_default_candidate_limit():
    goal = AUTO.make_startup_goal(5.0, 0, False)
    assert goal.max_candidates == 0
    assert goal.timeout_s == 5.0
