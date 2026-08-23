import json

from agt_offline_assets.vehicle_feasibility_audit import VehicleFeasibilityAudit


def test_profile_sweep_contains_required_profiles():
    audit = VehicleFeasibilityAudit({}, {}, {})
    result = audit.profile_sweep()
    assert set(result["profiles"].keys()) == {
        "profile_0",
        "profile_1",
        "profile_2",
        "profile_3",
        "profile_4",
    }


def test_counterfactual_is_review_only():
    audit = VehicleFeasibilityAudit({}, {}, {})
    result = audit.obstacle_counterfactual()
    assert result["authority"] == "EXPERIMENTAL_REVIEW_EVIDENCE"
    assert result["map_authority"] == "NOT_NAVIGATION_MAP_AUTHORITY"


def test_root_cause_classification():
    audit = VehicleFeasibilityAudit(
        {},
        {},
        {"aisles": [{"aisle_id": "a1", "root_cause": "WIDTH_LIMITED"}]},
    )
    result = audit.classify()
    assert result["root_cause"]["a1"] == "WIDTH_LIMITED"
