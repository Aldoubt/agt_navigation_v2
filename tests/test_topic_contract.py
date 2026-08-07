from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "docs/interfaces/topic_contract.md"


def test_topic_contract_declares_required_canonical_topics_once():
    text = CONTRACT.read_text(encoding="utf-8")
    required = (
        "/agt/sensors/lidar/custom",
        "/agt/sensors/lidar/custom_filtered",
        "/agt/sensors/imu/data",
        "/agt/sensors/camera/image",
        "/agt/sensors/camera/camera_info",
        "/agt/sensors/gnss/fix",
        "/agt/mapping/odometry",
        "/agt/mapping/registered_points",
        "/agt/chassis/odometry",
        "/agt/perception/ground_cloud",
        "/agt/perception/obstacle_cloud",
        "/agt/perception/ground_plane",
        "/agt/map/local_occupancy",
        "/agt/map/global_occupancy",
        "/agt/map/waypoints",
        "/agt/system/health",
        "/agt/system/task_readiness",
    )
    table_rows = tuple(line for line in text.splitlines() if line.startswith("| `/agt/"))
    assert all(sum(line.startswith("| `" + topic + "` |") for line in table_rows) == 1 for topic in required)


def test_historical_registered_point_names_are_not_runtime_interfaces():
    forbidden = ("/agt/mapping/registered_points_lidar", "/agt/mapping/registered_cloud")
    roots = (ROOT / "src", ROOT / "profiles", ROOT / "launch", ROOT / "tests")
    offenders = []
    for base in roots:
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.is_file() and path != Path(__file__) and ".git" not in path.parts and "__pycache__" not in path.parts:
                body = path.read_text(encoding="utf-8", errors="ignore")
                if any(name in body for name in forbidden):
                    offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []


def test_v25_entry_documents_do_not_reintroduce_migration_narrative():
    for relative in ("README.md", "AGENTS.md", "docs/architecture/system_architecture.md", "docs/interfaces/topic_contract.md", "docs/roadmap/v2_5.md"):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "Bunker Qt5 FAST-LIO Navigation Baseline Integration" not in text
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "Phase 0" not in readme
    assert "TASK-" not in readme


def test_authoritative_tf_and_cloud_contract_is_explicit():
    architecture = (ROOT / "docs/architecture/system_architecture.md").read_text(encoding="utf-8")
    topics = (ROOT / "docs/interfaces/topic_contract.md").read_text(encoding="utf-8")
    assert "/agt/mapping/registered_points" in architecture
    assert "frame=odom" in architecture
    assert "`odom` frame" in topics
    assert "agt_localization" in architecture and "map → odom" in architecture
    assert "agt_mapping_fast_livo2_adapter" in architecture and "odom → base_footprint" in architecture


def test_current_markdown_relative_links_resolve():
    link_pattern = re.compile(r"!?\[[^]]*\]\(([^)]+)\)")
    ignored_roots = {"docs/archive", "third_party", "build", "install", "log"}
    for path in ROOT.rglob("*.md"):
        relative = path.relative_to(ROOT).as_posix()
        if any(relative == root or relative.startswith(root + "/") for root in ignored_roots):
            continue
        for target in link_pattern.findall(path.read_text(encoding="utf-8", errors="ignore")):
            target = target.split("#", 1)[0].strip().strip("<>")
            target = re.sub(r":\d+$", "", target)
            if not target or target.startswith(("http://", "https://", "mailto:", "/")):
                continue
            assert (path.parent / target).exists(), f"broken link in {relative}: {target}"
