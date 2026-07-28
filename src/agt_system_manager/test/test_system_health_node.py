import importlib.util
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from agt_interfaces.msg import MapVersionSummary


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "system_health_node.py"
SPEC = importlib.util.spec_from_file_location("system_health_node", SCRIPT)
HEALTH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HEALTH)


def _status(values, name="agt_safety/tracked_controller"):
    status = DiagnosticStatus(name=name)
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    message = DiagnosticArray()
    message.status = [status]
    return message


def test_safety_gate_accepts_controller_clear_and_ready_status():
    assert HEALTH._safety_gate_from_status(
        _status(
            {
                "motion_enabled": "true",
                "estop_latched": "false",
                "emergency_stop": "false",
                "navigation_ready": "true",
            }
        )
    ) == (True, True, False, True)


def test_safety_gate_rejects_missing_or_foreign_status():
    assert HEALTH._safety_gate_from_status(_status({"motion_enabled": "true"})) == (
        True,
        False,
        True,
        False,
    )
    assert HEALTH._safety_gate_from_status(
        _status({"motion_enabled": "true"}, name="other/node")
    ) == (False, False, True, False)


def test_topic_freshness_expires_cached_readiness_inputs():
    stats = {"/status": {"count": 1, "last_seen": 10.0}}
    assert HEALTH._topic_is_fresh(stats, "/status", 1.0, 11.0)
    assert not HEALTH._topic_is_fresh(stats, "/status", 1.0, 11.001)


def test_active_map_gate_uses_typed_manager_context_only():
    message = MapVersionSummary()
    message.active = True
    message.state = MapVersionSummary.STATE_READY
    message.valid = True
    message.map_id = "map_a"
    message.map_version_id = "version_a"
    message.map_hash = "sha256:" + "1" * 64
    message.navigation_yaml = "/managed/navigation/map.yaml"
    message.localization_pcd = "/managed/pointcloud/map.pcd"
    message.processing_record = "/managed/pointcloud/map.processing.yaml"
    assert HEALTH._active_map_gate(message) == (
        "map_a", "version_a", message.map_hash, True, True, True
    )
    message.active = False
    assert HEALTH._active_map_gate(message) == ("", "", "", False, False, False)
