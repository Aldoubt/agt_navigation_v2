from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from agt_system_manager.robot_state import (
    mode_value, nav2_state_from_health, parse_chassis_status, parse_safety_status,
)
from agt_interfaces.msg import ComponentHealth, SystemHealth


def _diagnostic(name, values, level=DiagnosticStatus.OK):
    status = DiagnosticStatus(name=name, level=level)
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    return DiagnosticArray(status=[status])


def test_unknown_mode_remains_unknown():
    assert mode_value("") == 0
    assert mode_value("NOT_A_MODE") == 0
    assert mode_value("navigation") == 5


def test_safety_requires_complete_authoritative_diagnostics():
    incomplete = _diagnostic("agt_safety/tracked_controller", {"motion_enabled": "true"})
    assert parse_safety_status(incomplete)["known"] is False
    clear = _diagnostic(
        "agt_safety/tracked_controller",
        {
            "motion_enabled": "true",
            "emergency_stop": "false",
            "estop_latched": "false",
            "navigation_ready": "true",
        },
    )
    parsed = parse_safety_status(clear)
    assert parsed == {
        "known": True,
        "motion_enabled": True,
        "emergency_stop": False,
        "estop_latched": False,
        "navigation_ready": True,
    }


def test_chassis_control_mode_and_connection_are_independent():
    message = _diagnostic(
        "agt_chassis/bunker", {"control_mode": "1"}, level=DiagnosticStatus.ERROR
    )
    assert parse_chassis_status(message) == {
        "known": True, "connected": False, "control_mode": 2
    }


def test_nav2_active_requires_lifecycle_component_evidence():
    health = SystemHealth()
    component = ComponentHealth(
        component_id="nav2", state=ComponentHealth.STATE_OK, present=True
    )
    health.components = [component]
    assert nav2_state_from_health(health, "NAVIGATION") == 2
    component.lifecycle_failures = ["planner_server=inactive"]
    assert nav2_state_from_health(health, "NAVIGATION") == 3
    assert nav2_state_from_health(health, "IDLE") == 1

