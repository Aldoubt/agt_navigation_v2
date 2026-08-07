from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SEMANTICS = ROOT / "docs/architecture/navigation_semantics.md"
ARCHITECTURE = ROOT / "docs/architecture/system_architecture.md"
TOPICS = ROOT / "docs/interfaces/topic_contract.md"
MISSION = ROOT / "docs/interfaces/mission_schema.md"
AGENTS = ROOT / "AGENTS.md"
INTERFACES_README = ROOT / "src/agt_interfaces/README.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _flat(text: str) -> str:
    """Normalize Markdown line wrapping without weakening semantic token checks."""
    return " ".join(text.split())


def test_navigation_modes_are_defined_without_claiming_route_runtime():
    semantics = _read(SEMANTICS)
    architecture = _read(ARCHITECTURE)
    for mode in ("MAP", "ROUTE", "LOCAL"):
        assert f"### {mode}" in semantics
        assert f"{mode} Navigation" in architecture
    assert "ROUTE | RESERVED" in architecture
    assert "LOCAL | RESERVED" in architecture
    assert "ROUTE 是 V25-09 及以后实现的目标能力" in semantics
    assert "LOCAL 是预留能力，当前未实现" in semantics


def test_map_products_keep_distinct_lifetimes_and_frames():
    semantics = _read(SEMANTICS)
    topics = _read(TOPICS)
    flat_topics = _flat(topics)
    assert "Global Navigation Map" in semantics
    assert "Localization Prior" in semantics
    assert "Semantic Map" in semantics
    assert "Global Navigation Map != Localization Prior != Semantic Map" in semantics

    local_row = next(
        line for line in topics.splitlines()
        if line.startswith("| `/agt/map/local_occupancy` |")
    )
    assert "`odom`" in local_row
    assert "transient rolling" in local_row
    assert "reserved" in local_row
    assert "not versioned global-map truth" in flat_topics


def test_esdf_is_optional_derived_representation():
    semantics = _read(SEMANTICS)
    architecture = _read(ARCHITECTURE)
    topics = _flat(_read(TOPICS))
    assert "Optional ESDF" in semantics
    assert "ESDF 是可选派生表达" in semantics
    assert 'ESDF["Optional ESDF"]' in architecture
    assert "ESDF | OPTIONAL" in architecture
    assert "optional derived representation" in topics


def test_task_route_path_semantics_are_not_collapsed():
    token = "SemanticWaypoint != WaypointTask != Route != Runtime Path"
    assert token in _read(SEMANTICS)
    assert token in _read(TOPICS)
    assert "WaypointTask != Route != Runtime Path" in _read(MISSION)
    assert token in _read(AGENTS)


def test_tf_authority_allows_only_one_selected_map_to_odom_publisher():
    semantics = _flat(_read(SEMANTICS))
    topics = _flat(_read(TOPICS))
    agents = _flat(_read(AGENTS))
    for text in (semantics, topics, agents):
        assert "odom -> base_footprint" in text or "odom → base_footprint" in text
        assert "map -> odom" in text or "map → odom" in text
    assert "只能有一个被选中的 TF publisher" in semantics
    assert "selected TF publisher" in topics
    assert "exactly one selected runtime publisher" in agents
    assert "publish_tf=false" in semantics
    assert "agt_localization_fusion" in semantics


def test_project_navigation_capability_stays_above_nav2_native_interfaces():
    semantics = _flat(_read(SEMANTICS))
    interfaces = _flat(_read(INTERFACES_README))
    mission = _flat(_read(MISSION))
    agents = _flat(_read(AGENTS))
    assert "ExecuteWaypointTask" in semantics
    assert "Nav2 是其中一个 内部 backend" in semantics
    assert "waypoint navigation capability" in interfaces
    assert "Mission WAYPOINT_TASK -> project ExecuteWaypointTask capability" in mission
    assert "Navigation is a project capability, not a synonym for Nav2" in agents


def test_v25_08_does_not_smuggle_route_policy_into_versioned_ros_interfaces():
    action_dir = ROOT / "src/agt_interfaces/action"
    assert not (action_dir / "ExecuteRouteTask.action").exists()
    assert not (action_dir / "ExecuteNavigationTask.action").exists()
    mission = _read(MISSION)
    semantics = _flat(_read(SEMANTICS))
    assert "不新增 `navigation_mode`" in mission
    assert "本阶段不公开 `ExecuteRouteTask` 或 `ExecuteNavigationTask`" in semantics
