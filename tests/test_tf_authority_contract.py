from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
LOCALIZATION = ROOT / "src" / "agt_localization"
NAVIGATION = ROOT / "src" / "agt_navigation"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_global_correction_manager_is_production_map_odom_authority():
    manager = _read(LOCALIZATION / "src" / "global_correction_manager.cpp")
    launch = _read(LOCALIZATION / "launch" / "relocalization.launch.py")
    params = yaml.safe_load(
        _read(LOCALIZATION / "config" / "relocalization.yaml")
    )["/**"]["ros__parameters"]

    assert "TransformBroadcaster" in manager
    assert "latest_map_from_odom_" in manager
    assert 'global_frame_, odom_frame_' in manager
    assert 'executable="global_correction_manager"' in launch

    # Legacy relocalization code retains a compatibility publish_tf switch, but
    # the production config and production launch both force it off. It therefore
    # produces global-pose evidence rather than a competing TF stream.
    assert params["publish_tf"] is False
    assert '"publish_tf": False' in launch


def test_correction_manager_is_canonical_localization_status_owner():
    manager = _read(LOCALIZATION / "src" / "global_correction_manager.cpp")
    launch = _read(LOCALIZATION / "launch" / "relocalization.launch.py")
    correction = yaml.safe_load(
        _read(LOCALIZATION / "config" / "global_correction.yaml")
    )["/**"]["ros__parameters"]

    assert correction["evidence_status_topic"] == "/agt/localization/evidence_status"
    assert correction["canonical_status_topic"] == "/agt/localization/status"
    assert '"/agt/localization/status"' in launch
    assert '"/agt/localization/evidence_status"' in launch
    assert "canonical_status_pub_->publish" in manager
    assert "localization_accepted = false" in manager
    assert "STATE_RECOVERING" in manager


def test_mapping_navigation_and_recovery_do_not_claim_map_odom_authority():
    route_runtime = _read(NAVIGATION / "agt_navigation" / "route_runtime.py")
    route_adapter = _read(NAVIGATION / "agt_navigation" / "nav2_follow_path_adapter.py")
    recovery = _read(LOCALIZATION / "src" / "recovery_trigger_manager.cpp")

    assert "TransformBroadcaster" not in route_runtime
    assert "TransformBroadcaster" not in route_adapter
    assert "TransformBroadcaster" not in recovery


def test_v25_10_uses_existing_localization_evidence_without_new_public_message():
    manager = _read(LOCALIZATION / "src" / "global_correction_manager.cpp")
    action = _read(ROOT / "src" / "agt_interfaces" / "action" / "Relocalize.action")

    assert "LocalizationStatus" in manager
    assert "status->global_pose" in manager
    assert "status->map_id" in manager
    assert "status->map_hash" in manager
    assert "final_status" in action
    assert not (ROOT / "src" / "agt_interfaces" / "msg" / "GlobalCorrection.msg").exists()
