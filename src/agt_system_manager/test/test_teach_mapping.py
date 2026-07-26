from pathlib import Path
import signal
import subprocess

import pytest
import yaml

from agt_teach_repeat.path_io import (
    atomic_write_yaml,
    sha256_file,
    write_reference_paths,
)
from agt_system_manager.teach_mapping import (
    ProcessGroupSupervisor,
    TeachMappingError,
    _parser,
    candidate_commands,
    create_report,
    extract_session,
    init_session,
    load_nav2_grid,
    load_session,
    map_metrics,
    mark_failed,
    register_rescan,
    transition,
    validate_candidate_assets,
)
from agt_teach_repeat.path_types import PathPose


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "profiles" / "platforms" / "bunker.yaml"


def write_pgm(path, width=4, height=3, pixels=None):
    pixels = pixels or [254] * (width * height)
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    )


def write_pcd(path, points=2):
    rows = "\n".join(f"{index}.0 0.0 0.0" for index in range(points))
    path.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {points}\n"
        f"DATA ascii\n{rows}\n",
        encoding="ascii",
    )


def bag_metadata(topics):
    return {
        "rosbag2_bagfile_information": {
            "storage_identifier": "sqlite3",
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": name, "type": topic_type},
                    "message_count": count,
                }
                for name, topic_type, count in topics
            ],
        }
    }


def make_bootstrap(tmp_path, *, ready=True, recorded_hash=True):
    tmp_path.mkdir(parents=True, exist_ok=True)
    image = tmp_path / "map.pgm"
    write_pgm(image)
    map_yaml = tmp_path / "map.yaml"
    atomic_write_yaml(
        map_yaml,
        {
            "image": "map.pgm",
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        },
    )
    pcd = tmp_path / "localization_map.pcd"
    write_pcd(pcd)
    record = tmp_path / "localization_map.processing.yaml"
    value = {
        "state": "ready" if ready else "processing",
        "map_file": pcd.name,
    }
    if recorded_hash:
        value["pcd_sha256"] = sha256_file(pcd)
    atomic_write_yaml(record, value)
    bag = tmp_path / "teach_bag"
    bag.mkdir(exist_ok=True)
    atomic_write_yaml(
        bag / "metadata.yaml",
        bag_metadata(
            [("/agt/mapping/odometry", "nav_msgs/msg/Odometry", 10)]
        ),
    )
    return map_yaml, pcd, record, bag


def init_valid(tmp_path, **overrides):
    assets = overrides.pop("_assets", None)
    map_yaml, pcd, record, bag = assets or make_bootstrap(tmp_path / "assets")
    values = {
        "session_id": "greenhouse_001",
        "runtime_root": tmp_path / "runtime",
        "platform_profile": PROFILE,
        "map_id": "greenhouse_bootstrap",
        "bootstrap_map_yaml": map_yaml,
        "bootstrap_localization_pcd": pcd,
        "bootstrap_processing_record": record,
        "teach_bag": bag,
        "map_from_teach_odom_x": 0.0,
        "map_from_teach_odom_y": 0.0,
        "map_from_teach_odom_z": 0.0,
        "map_from_teach_odom_yaw": 0.0,
    }
    values.update(overrides)
    return init_session(**values)


def test_valid_session_creation_is_bound_and_atomic(tmp_path):
    path = init_valid(tmp_path)
    loaded_path, session = load_session(path)
    assert loaded_path == path.resolve()
    assert session["stage"] == "BOOTSTRAP_READY"
    assert session["last_successful_stage"] == "BOOTSTRAP_READY"
    assert session["bootstrap"]["teach_bag_sha256"].startswith("sha256:")
    assert session["platform"]["profile_sha256"] == sha256_file(PROFILE)
    assert not list(path.parent.glob(".*.tmp"))
    assert {item.name for item in path.parent.iterdir()} >= {
        "bootstrap",
        "teach_route",
        "rescan",
        "candidate_map",
        "reports",
        "session.yaml",
    }


def test_extract_allows_the_initialized_empty_route_directory(tmp_path, monkeypatch):
    path = init_valid(tmp_path)
    observed = {}

    def fake_extract_demo(**kwargs):
        observed.update(kwargs)
        raise TeachMappingError("expected_stop", "stop after checking overwrite")

    monkeypatch.setattr(
        "agt_system_manager.teach_mapping.extract_demo", fake_extract_demo
    )
    with pytest.raises(TeachMappingError, match="stop after checking overwrite"):
        extract_session(path)

    assert observed["output_demo_dir"] == path.parent / "teach_route"
    assert observed["overwrite"] is True


def test_duplicate_session_and_unsafe_id_are_rejected(tmp_path):
    init_valid(tmp_path)
    with pytest.raises(TeachMappingError, match="already exists"):
        init_valid(tmp_path)
    (tmp_path / "other").mkdir()
    with pytest.raises(TeachMappingError) as error:
        init_valid(tmp_path / "other", session_id="../escape")
    assert error.value.code == "invalid_session_id"


@pytest.mark.parametrize("missing", ["map", "pcd", "record", "bag"])
def test_missing_bootstrap_assets_are_rejected_and_record_failure(tmp_path, missing):
    map_yaml, pcd, record, bag = make_bootstrap(tmp_path)
    paths = {"map": map_yaml, "pcd": pcd, "record": record, "bag": bag}
    target = paths[missing]
    if target.is_dir():
        (target / "metadata.yaml").unlink()
    else:
        target.unlink()
    with pytest.raises(TeachMappingError):
        init_session(
            session_id="failed_01",
            runtime_root=tmp_path / "runtime",
            platform_profile=PROFILE,
            map_id="map_01",
            bootstrap_map_yaml=map_yaml,
            bootstrap_localization_pcd=pcd,
            bootstrap_processing_record=record,
            teach_bag=bag,
            map_from_teach_odom_x=0.0,
            map_from_teach_odom_y=0.0,
            map_from_teach_odom_z=0.0,
            map_from_teach_odom_yaw=0.0,
        )
    _, session = load_session(tmp_path / "runtime" / "failed_01" / "session.yaml")
    assert session["stage"] == "FAILED"
    assert session["last_successful_stage"] == "CREATED"


def test_nonready_and_pcd_hash_mismatch_are_rejected(tmp_path):
    map_yaml, pcd, record, bag = make_bootstrap(tmp_path, ready=False)
    with pytest.raises(TeachMappingError) as error:
        init_valid(
            tmp_path,
            _assets=(map_yaml, pcd, record, bag),
        )
    assert error.value.code == "pcd_not_ready"

    second = tmp_path / "second"
    second.mkdir()
    map_yaml, pcd, record, bag = make_bootstrap(second)
    data = yaml.safe_load(record.read_text())
    data["pcd_sha256"] = "sha256:" + "0" * 64
    atomic_write_yaml(record, data)
    with pytest.raises(TeachMappingError) as error:
        init_valid(
            second,
            _assets=(map_yaml, pcd, record, bag),
        )
    assert error.value.code == "pcd_hash_mismatch"


def test_transform_arguments_are_required_by_cli():
    with pytest.raises(SystemExit):
        _parser().parse_args(
            [
                "init",
                "--session-id", "s",
                "--runtime-root", "/tmp/r",
                "--platform-profile", "/tmp/p",
                "--map-id", "m",
                "--bootstrap-map-yaml", "/tmp/m",
                "--bootstrap-localization-pcd", "/tmp/p",
                "--bootstrap-processing-record", "/tmp/r",
                "--teach-bag", "/tmp/b",
            ]
        )


def test_state_machine_rejects_jumps_and_failure_preserves_success():
    session = {"stage": "CREATED", "last_successful_stage": "CREATED"}
    ready = transition(session, "BOOTSTRAP_READY")
    extracted = transition(ready, "PATH_EXTRACTED")
    assert extracted["last_successful_stage"] == "PATH_EXTRACTED"
    with pytest.raises(TeachMappingError):
        transition(session, "RESCAN_RECORDED")
    failed = mark_failed(extracted, "test", "failure")
    assert failed["stage"] == "FAILED"
    assert failed["last_successful_stage"] == "PATH_EXTRACTED"
    assert failed["last_error"] == {"code": "test", "message": "failure"}


def test_candidate_commands_are_offline_raw_only_and_separate(tmp_path):
    path = init_valid(tmp_path)
    _, session = load_session(path)
    session["stage"] = "RESCAN_RECORDED"
    session["last_successful_stage"] = "RESCAN_RECORDED"
    session["rescan"].update({"bag": "/tmp/rescan", "bag_sha256": "sha256:x", "completed": True})
    atomic_write_yaml(path, session)
    commands = candidate_commands(path, "candidate_v1")
    mapping = " ".join(commands["mapping"])
    assert "mode:=mapping" in mapping
    assert "use_sim_time:=true" in mapping
    assert "start_sensor:=false" in mapping
    assert "start_chassis:=false" in mapping
    assert "start_chassis_monitor:=false" in mapping
    assert "start_rviz:=false" in mapping
    assert "start_mapping_gui:=false" in mapping
    assert "record_bag:=false" in mapping
    assert "--clock" in commands["play"]
    assert commands["play"][-2:] == [
        "/agt/sensors/lidar/custom",
        "/agt/sensors/imu/data",
    ]
    bootstrap = Path(session["bootstrap"]["map_yaml"])
    assert commands["root"] != bootstrap.parent
    assert path.parent / "candidate_map" in commands["root"].parents


def test_rescan_registration_rejects_missing_required_topic(tmp_path, monkeypatch):
    path = init_valid(tmp_path)
    _, session = load_session(path)
    session["stage"] = "PATH_EXTRACTED"
    session["last_successful_stage"] = "PATH_EXTRACTED"
    atomic_write_yaml(path, session)
    bag = tmp_path / "rescan"
    bag.mkdir()
    atomic_write_yaml(
        bag / "metadata.yaml",
        bag_metadata(
            [
                ("/agt/sensors/lidar/custom", "livox_ros_driver2/msg/CustomMsg", 10),
                ("/agt/sensors/imu/data", "sensor_msgs/msg/Imu", 10),
            ]
        ),
    )
    monkeypatch.setattr(
        "agt_system_manager.teach_mapping.validate_session_bindings",
        lambda _session: (None, None),
    )
    with pytest.raises(TeachMappingError) as error:
        register_rescan(path, bag)
    assert error.value.code == "bag_topic_missing"
    _, failed = load_session(path)
    assert failed["stage"] == "FAILED"
    assert failed["last_successful_stage"] == "PATH_EXTRACTED"


def test_candidate_failure_preserves_bootstrap_and_allows_new_name(
    tmp_path, monkeypatch
):
    from agt_system_manager import teach_mapping

    path = init_valid(tmp_path)
    _, session = load_session(path)
    session["stage"] = "RESCAN_RECORDED"
    session["last_successful_stage"] = "RESCAN_RECORDED"
    session["rescan"].update(
        {"bag": "/tmp/rescan", "bag_sha256": "sha256:registered", "completed": True}
    )
    atomic_write_yaml(path, session)
    bootstrap_before = dict(session["bootstrap"])
    monkeypatch.setattr(teach_mapping, "validate_session_bindings", lambda _session: (None, None))
    monkeypatch.setattr(teach_mapping, "sha256_path", lambda _path: "sha256:registered")

    class Process:
        pid = 123

        def poll(self):
            return None

    class Supervisor:
        def __init__(self):
            self.cleaned = False

        def start(self, _command, _log):
            return Process()

        def cleanup(self):
            self.cleaned = True

    supervisor = Supervisor()

    def fail_readiness(_process, _timeout):
        raise TeachMappingError("mapping_start_timeout", "not ready")

    with pytest.raises(TeachMappingError) as error:
        teach_mapping.build_candidate(
            path,
            "candidate_v1",
            supervisor=supervisor,
            readiness_waiter=fail_readiness,
        )
    assert error.value.code == "mapping_start_timeout"
    assert supervisor.cleaned is True
    _, failed = load_session(path)
    assert failed["stage"] == "FAILED"
    assert failed["last_successful_stage"] == "RESCAN_RECORDED"
    assert failed["bootstrap"] == bootstrap_before
    assert candidate_commands(path, "candidate_v2")["root"].name == "candidate_v2"


def test_process_cleanup_uses_only_sigint_and_sigterm():
    class Process:
        pid = 42

        def __init__(self):
            self.returncode = None
            self.wait_count = 0

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            self.wait_count += 1
            if self.wait_count == 1:
                raise subprocess.TimeoutExpired("fake", timeout)
            self.returncode = -signal.SIGTERM
            return self.returncode

    process = Process()
    signals = []
    supervisor = ProcessGroupSupervisor(kill_group=lambda pid, sig: signals.append((pid, sig)))
    supervisor._active.append(process)
    supervisor.cleanup()
    assert signals == [(42, signal.SIGINT), (42, signal.SIGTERM)]
    assert signal.SIGKILL not in [item[1] for item in signals]


def test_incomplete_candidate_never_validates_ready(tmp_path):
    root = tmp_path / "candidate"
    root.mkdir()
    with pytest.raises(TeachMappingError, match="candidate map YAML"):
        validate_candidate_assets(root, "candidate_v1")


def test_trinary_grid_origin_yaw_and_different_geometry_metrics(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    for root, dimensions, yaw in ((first, (4, 3), 0.0), (second, (5, 2), 1.57079632679)):
        width, height = dimensions
        pixels = [254] * (width * height)
        pixels[0] = 0
        pixels[1] = 205
        write_pgm(root / "map.pgm", width, height, pixels)
        atomic_write_yaml(
            root / "map.yaml",
            {
                "image": "map.pgm",
                "resolution": 1.0,
                "origin": [10.0, 20.0, yaw],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
        )
        write_pcd(root / "map.pcd", 3)
        atomic_write_yaml(
            root / "processing.yaml", {"state": "ready", "map_file": "map.pcd"}
        )
    grid, _ = load_nav2_grid(second / "map.yaml")
    assert (grid.width, grid.height, grid.origin_yaw) == (5, 2, pytest.approx(1.57079632679))
    poses = (
        PathPose(1, 9.5, 20.5, frame_id="map"),
        PathPose(2, 9.5, 23.5, frame_id="map"),
    )
    metrics = map_metrics(
        second / "map.yaml",
        second / "map.pcd",
        second / "processing.yaml",
        poses,
        [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]],
        0.0,
        path_sample_distance_m=0.5,
    )
    assert metrics["map"]["width"] == 5
    assert metrics["map"]["occupied_cells"] == 1
    assert metrics["map"]["unknown_cells"] == 1
    assert metrics["teach_path"]["sample_count"] == 7
    assert metrics["teach_path"]["center_cells"] == {
        "free": 7,
        "occupied": 0,
        "unknown": 0,
        "outside": 0,
    }
    assert metrics["teach_path"]["swept_audited_cells"] > 0


def test_report_supports_different_geometry_and_never_selects_winner(tmp_path):
    path = init_valid(tmp_path)
    _, session = load_session(path)
    teach_route = path.parent / "teach_route"
    reference = write_reference_paths(
        teach_route / "processed",
        session["teach_route"]["demo_id"],
        (
            PathPose(1, 0.1, 0.1, frame_id="map"),
            PathPose(2, 0.2, 0.1, frame_id="map"),
        ),
    )["yaml"]
    bootstrap = session["bootstrap"]
    manifest = {
        "schema_version": 1,
        "demo_id": session["teach_route"]["demo_id"],
        "source": {
            "bag_path": bootstrap["teach_bag"],
            "bag_sha256": bootstrap["teach_bag_sha256"],
            "odometry_topic": "/agt/mapping/odometry",
        },
        "map": {
            "map_id": bootstrap["map_id"],
            "map_yaml": bootstrap["map_yaml"],
            "map_yaml_sha256": bootstrap["map_yaml_sha256"],
            "localization_pcd": bootstrap["localization_pcd"],
            "localization_pcd_sha256": bootstrap["localization_pcd_sha256"],
            "processing_record": bootstrap["processing_record"],
            "processing_record_sha256": bootstrap["processing_record_sha256"],
        },
        "platform": {"profile": session["platform"]["profile"]},
        "frames": {
            "execution_frame": "map",
            "map_from_teach_odom": session["transform"]["map_from_teach_odom"],
        },
        "processing": {},
        "execution": {"maximum_linear_speed_mps": 0.2},
        "assets": {
            "reference_path": "processed/reference_path.yaml",
            "reference_path_sha256": sha256_file(reference),
        },
    }
    manifest_path = teach_route / "manifest.yaml"
    atomic_write_yaml(manifest_path, manifest)

    candidate_root = path.parent / "candidate_map" / "maps" / "candidate_v1"
    candidate_root.mkdir(parents=True)
    write_pgm(candidate_root / "candidate_v1.pgm", 5, 4)
    atomic_write_yaml(
        candidate_root / "candidate_v1.yaml",
        {
            "image": "candidate_v1.pgm",
            "resolution": 0.2,
            "origin": [-0.1, -0.1, 0.2],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        },
    )
    (candidate_root / "pcd").mkdir()
    write_pcd(candidate_root / "pcd" / "localization_map.pcd", 4)
    atomic_write_yaml(
        candidate_root / "pcd" / "localization_map.processing.yaml",
        {"state": "ready", "map_file": "localization_map.pcd"},
    )
    candidate = validate_candidate_assets(candidate_root, "candidate_v1")
    session["stage"] = "CANDIDATE_MAP_READY"
    session["last_successful_stage"] = "CANDIDATE_MAP_READY"
    session["teach_route"].update(
        {
            "manifest": str(manifest_path),
            "manifest_sha256": sha256_file(manifest_path),
        }
    )
    session["candidate_map"].update(
        {"map_name": "candidate_v1", **candidate, "ready": True}
    )
    atomic_write_yaml(path, session)

    json_path, markdown_path = create_report(path)
    report = yaml.safe_load(json_path.read_text(encoding="utf-8"))
    assert report["bootstrap"]["map"]["width"] == 4
    assert report["candidate"]["map"]["width"] == 5
    assert report["automatic_winner_selected"] is False
    assert report["operator_decision_required"] is True
    assert any("map_geometry_differs" in warning for warning in report["warnings"])
    assert "does not select or publish" in markdown_path.read_text(encoding="utf-8")
