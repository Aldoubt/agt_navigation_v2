from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SEMANTICS = ROOT / "docs/architecture/navigation_semantics.md"
ARCHITECTURE = ROOT / "docs/architecture/system_architecture.md"
OFFLINE = ROOT / "docs/architecture/offline_asset_pipeline.md"
BT = ROOT / "docs/architecture/behavior_tree_execution.md"
SITE_REQUIREMENTS = ROOT / "docs/v2.5/V25_12_SITE_WORKFLOW_REQUIREMENTS.md"
SITE_CONTRACT = ROOT / "docs/interfaces/site_package_manifest.md"
ROADMAP = ROOT / "docs/roadmap/v2_5.md"
TOPICS = ROOT / "docs/interfaces/topic_contract.md"
MISSION = ROOT / "docs/interfaces/mission_schema.md"
AGENTS = ROOT / "AGENTS.md"
INTERFACES_README = ROOT / "src/agt_interfaces/README.md"
SITE_PACKAGE_CODE = ROOT / "src/agt_offline_assets/agt_offline_assets/site_package.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _flat(text: str) -> str:
    """Normalize Markdown line wrapping without weakening semantic token checks."""
    return " ".join(text.split())


def test_navigation_modes_remain_semantically_distinct_under_v25_12_capability_architecture():
    semantics = _read(SEMANTICS)
    architecture = _read(ARCHITECTURE)
    for mode in ("MAP", "ROUTE", "LOCAL"):
        assert f"### {mode}" in semantics
    assert "Project Navigation Capability Plane" in architecture
    assert "ExecuteRoute / ExecuteWaypointTask" in architecture
    assert "NavigateToSemanticTarget" in architecture
    assert "ExecuteCoverageTask" in architecture
    assert "Semantic Task Intent" in architecture
    assert "Route Asset" in architecture
    assert "Runtime Path" in architecture


def test_map_products_keep_distinct_lifetimes_and_site_package_bindings():
    semantics = _read(SEMANTICS)
    offline = _read(OFFLINE)
    site_contract = _read(SITE_CONTRACT)
    topics = _read(TOPICS)
    flat_topics = _flat(topics)
    assert "Global Navigation Map" in semantics
    assert "Localization Prior" in semantics
    assert "Semantic Map" in semantics
    assert "Global Navigation Map != Localization Prior != Semantic Map" in semantics
    for token in (
        "Localization Map",
        "Localization Prior",
        "Navigation Map",
        "Semantic Map",
        "Route Asset",
        "READY Site Package",
    ):
        assert token in offline
    assert "Site Package is not a second map registry" in site_contract

    local_row = next(
        line for line in topics.splitlines()
        if line.startswith("| `/agt/map/local_occupancy` |")
    )
    assert "`odom`" in local_row
    assert "transient rolling" in local_row
    assert "reserved" in local_row
    assert "not versioned global-map truth" in flat_topics


def test_esdf_remains_optional_instead_of_defining_v25_12():
    semantics = _read(SEMANTICS)
    roadmap = _read(ROADMAP)
    requirements = _read(SITE_REQUIREMENTS)
    assert "Optional ESDF" in semantics
    assert "ESDF 是可选派生表达" in semantics
    assert "V25-12 is no longer defined as Optional ESDF" in roadmap
    assert "ESDF remains an optional local-planning product" in roadmap
    assert "local ESDF" in requirements


def test_task_route_path_semantics_are_not_collapsed():
    token = "SemanticWaypoint != WaypointTask != Route != Runtime Path"
    assert token in _read(SEMANTICS)
    assert token in _read(TOPICS)
    assert "WaypointTask != Route != Runtime Path" in _read(MISSION)
    assert token in _read(AGENTS)
    architecture = _read(ARCHITECTURE)
    assert "Semantic Task Intent" in architecture
    assert "Route Asset" in architecture
    assert "Runtime Path" in architecture
    assert "Mission/BT 描述业务流程" in architecture


def test_tf_authority_allows_only_one_selected_map_to_odom_publisher():
    semantics = _flat(_read(SEMANTICS))
    topics = _flat(_read(TOPICS))
    agents = _flat(_read(AGENTS))
    architecture = _flat(_read(ARCHITECTURE))
    for text in (semantics, topics, agents, architecture):
        assert "odom -> base_footprint" in text or "odom → base_footprint" in text
        assert "map -> odom" in text or "map → odom" in text
    assert "只能有一个被选中的 TF publisher" in semantics
    assert "selected TF publisher" in topics
    assert "exactly one selected runtime publisher" in agents
    assert "Localization Authority" in architecture
    assert "Authoritative map → odom" in architecture
    assert "agt_localization_fusion" in semantics


def test_project_navigation_capability_stays_above_backend_native_interfaces():
    semantics = _flat(_read(SEMANTICS))
    interfaces = _flat(_read(INTERFACES_README))
    mission = _flat(_read(MISSION))
    agents = _flat(_read(AGENTS))
    bt = _flat(_read(BT))
    architecture = _flat(_read(ARCHITECTURE))
    assert "ExecuteWaypointTask" in semantics
    assert "Nav2 是其中一个 内部 backend" in semantics
    assert "waypoint navigation capability" in interfaces
    assert "Mission WAYPOINT_TASK -> project ExecuteWaypointTask capability" in mission
    assert "Navigation is a project capability, not a synonym for Nav2" in agents
    for token in (
        "EnsureLocalization",
        "ExecuteRoute",
        "NavigateToSemanticTarget",
        "ExecuteCoverageTask",
        "RecordBenchmark",
    ):
        assert token in bt
    assert "Replaceable Odometry Backend" in architecture
    assert "Replaceable Global Planner" in architecture
    assert "Replaceable Local Controller" in architecture


def test_bt_business_tree_does_not_encode_backend_packages_as_capabilities():
    bt = _read(BT)
    assert "CallFastLivo2Node" in bt
    assert "RunNDTWithTheseParameters" in bt
    assert "CallNavfn" in bt
    assert "CallMPPI" in bt
    assert "Incorrect Mission-level semantics" in bt
    assert "BT nodes do not publish chassis/navigation velocity or TF" in _read(AGENTS)
    assert "Changing one backend must not require editing the business-level BT tree" in bt


def test_v25_12a_site_package_is_pure_offline_and_adds_no_ros_interface():
    action_dir = ROOT / "src/agt_interfaces/action"
    service_dir = ROOT / "src/agt_interfaces/srv"
    assert not (action_dir / "LoadSitePackage.action").exists()
    assert not (action_dir / "ExecuteRouteTask.action").exists()
    assert not (action_dir / "ExecuteNavigationTask.action").exists()
    assert not (service_dir / "LoadSitePackage.srv").exists()

    code = _read(SITE_PACKAGE_CODE)
    assert "rclpy" not in code
    assert "create_publisher" not in code
    assert "create_subscription" not in code
    assert "create_service" not in code
    assert "ActionServer" not in code
    assert "agt_site_package/v1" in code
    assert "compute_site_package_content_sha256" in code
    assert "ready_site_package_immutable" in code


def test_v25_12_site_workflow_and_architecture_are_frozen_in_authoritative_docs():
    requirements = _read(SITE_REQUIREMENTS)
    architecture = _read(ARCHITECTURE)
    agents = _read(AGENTS)
    roadmap = _read(ROADMAP)
    assert "FROZEN REQUIREMENTS BASELINE" in requirements
    for token in (
        "Site acquisition",
        "Point-cloud cleaning and editing",
        "Localization prior authoring",
        "Semantic task annotation",
        "Benchmark dataset and truth binding",
        "READY Site Package export",
        "Mission / BT execution",
    ):
        assert token in requirements
    assert "Offline Site Production & Evaluation Plane" in architecture
    assert "Project Navigation Capability Plane" in architecture
    assert "Mission / BehaviorTree Capability Orchestration" in architecture
    assert "Current stage: `AGT Navigation V2.5 Site Workflow & BT Capability Architecture (V25-12)`" in agents
    assert "V25-12A  Site Manifest & Asset Lineage" in roadmap
