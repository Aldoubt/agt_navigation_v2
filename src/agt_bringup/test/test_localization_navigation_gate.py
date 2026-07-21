import importlib.util
from pathlib import Path

from agt_interfaces.msg import LocalizationStatus


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "localization_navigation_gate.py"
)
SPEC = importlib.util.spec_from_file_location("localization_navigation_gate", SCRIPT)
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)


def test_gate_requires_accepted_tracking_status():
    status = LocalizationStatus()
    assert not GATE.localization_status_is_ready(status)

    status.state = LocalizationStatus.STATE_TRACKING
    status.pose_valid = True
    status.localization_accepted = True
    status.error_code = LocalizationStatus.ERROR_NONE
    assert GATE.localization_status_is_ready(status)

    status.status_stale = True
    assert not GATE.localization_status_is_ready(status)
