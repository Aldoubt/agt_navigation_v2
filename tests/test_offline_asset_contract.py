import csv
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
INTERFACES = ROOT / "docs/interfaces"
EXAMPLES = INTERFACES / "examples"
OFFLINE_PACKAGE = ROOT / "src/agt_offline_assets"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_offline_lineage_contract_is_explicit():
    calibration = _read(INTERFACES / "calibration_dataset_contract.md")
    derivation = _read(INTERFACES / "map_derivation_contract.md")
    workflow = _read(ROOT / "docs/workflows/bag_to_route_asset.md")

    for token in (
        "dataset_sha256",
        "calibration_sha256",
        "platform_profile_sha256",
        "recipe_sha256",
    ):
        assert token in calibration

    assert "EVALUATION" in calibration
    assert "OPERATIONAL" in calibration
    assert "site_id" in derivation and "epoch_id" in derivation
    assert "canonical site frame" in derivation
    assert "Bag/Experiment" in _read(ROOT / "docs/architecture/offline_asset_pipeline.md")
    assert "Generate\n  -> Visualize" in workflow


def test_map_derivation_examples_bind_dataset_calibration_and_site():
    example = EXAMPLES / "map_derivation"
    dataset = _yaml(example / "dataset_binding.yaml")
    recipe = _yaml(example / "recipe.yaml")
    alignment = _yaml(example / "alignment.yaml")

    assert dataset["site_id"] == alignment["site_id"]
    assert dataset["epoch_id"] == alignment["epoch_id"]
    assert dataset["purpose"] in {"EVALUATION", "OPERATIONAL"}
    assert recipe["source_dataset_id"] == dataset["dataset_id"]
    assert recipe["calibration_id"] == dataset["calibration"]["calibration_id"]
    assert alignment["map_frame"] == "map"
    assert alignment["method"] in {"SITE_CONTROL_POINTS", "ENU_GEOREFERENCE", "REFERENCE_MAP"}


def test_route_asset_binds_map_semantics_vehicle_policy_and_feasibility():
    example = EXAMPLES / "route_asset"
    route = _yaml(example / "route.yaml")
    policy = _yaml(example / "policy.yaml")
    report = json.loads((example / "feasibility_report.json").read_text(encoding="utf-8"))

    assert route["frame_id"] == "map"
    assert route["map_binding"]["manifest_sha256"].startswith("sha256:")
    assert route["semantic_binding"]["sha256"].startswith("sha256:")
    assert route["semantic_binding"]["coverage_sha256"].startswith("sha256:")
    assert route["semantic_binding"]["path"].startswith("../../../semantic/")
    assert route["vehicle_binding"]["platform_profile_sha256"].startswith("sha256:")
    assert route["policy_binding"]["sha256"].startswith("sha256:")
    assert route["feasibility_report_sha256"].startswith("sha256:")
    assert route["preview_sha256"].startswith("sha256:")
    assert policy["constraints"]["direction_change_requires_stop"] is True
    assert report["status"] == "PASS"
    assert report["metrics"]["footprint_collision_count"] == 0
    assert report["metrics"]["curvature_violation_count"] == 0


def test_route_csv_has_direction_clearance_and_business_reference_without_executing_it():
    csv_path = EXAMPLES / "route_asset" / "route.csv"
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert rows
    required = {
        "seq",
        "segment_id",
        "x",
        "y",
        "yaw",
        "direction",
        "v_ref",
        "curvature",
        "clearance",
        "semantic_ref",
        "event_ref",
    }
    assert required.issubset(rows[0])
    assert {row["direction"] for row in rows}.issubset({"F", "R"})
    assert any(row["event_ref"] for row in rows)

    contract = _read(INTERFACES / "route_asset_contract.md")
    assert "Route Executor 不自行执行采摘、喷药、拍照等业务" in contract
    assert "重新 footprint sweep" in contract
    assert "READY revision 之后不得原地" in contract


def test_vehicle_geometry_stays_in_platform_profile_and_tracker_is_adapter_only():
    route_contract = _read(INTERFACES / "route_asset_contract.md")
    tracker = _read(INTERFACES / "vehicle_tracker_adapter.md")

    assert "profiles/platforms/<platform>.yaml" in route_contract
    assert "不负责" in tracker
    assert "map -> odom TF ownership" in tracker
    assert "Mission/BT task execution" in tracker
    assert "Tracker tuning" in tracker
    assert "不得维护第二份 vehicle geometry truth" in tracker


def test_offline_asset_package_implements_workspace_route_feasibility_and_tuning():
    required = (
        "agt_offline_assets/contracts.py",
        "agt_offline_assets/workspace.py",
        "agt_offline_assets/map_validation.py",
        "agt_offline_assets/route_asset.py",
        "agt_offline_assets/grid_io.py",
        "agt_offline_assets/feasibility.py",
        "agt_offline_assets/preview.py",
        "agt_offline_assets/tuning.py",
        "scripts/agt_offline_assets.py",
        "test/test_offline_assets.py",
    )
    for relative in required:
        assert (OFFLINE_PACKAGE / relative).is_file(), relative

    cli = _read(OFFLINE_PACKAGE / "scripts/agt_offline_assets.py")
    for command in ("init-map", "refresh-map", "validate-map", "derive-route", "validate-route", "tune-route"):
        assert command in cli
    assert "ExecuteRouteTask" not in cli
    assert "cmd_vel" not in cli


def test_v25_09_asset_contracts_do_not_add_public_route_actions():
    action_dir = ROOT / "src/agt_interfaces/action"
    assert not (action_dir / "ExecuteRouteTask.action").exists()
    assert not (action_dir / "ExecuteNavigationTask.action").exists()

    route_contract = _read(INTERFACES / "route_asset_contract.md")
    assert "Route Asset 本身不发布速度、不拥有 TF" in route_contract
