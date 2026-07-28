import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from agt_map_manager.registry import MapRegistry
from agt_system_manager.mapping_session import (
    MappingSessionError,
    MappingSessionRepository,
    mapping_session_timeout,
)


def _write_grid(map_url: Path) -> None:
    image = map_url.with_suffix(".pgm")
    image.write_bytes(b"P5\n3 2\n255\n" + bytes([254, 254, 0, 254, 205, 254]))
    map_url.with_suffix(".yaml").write_text(
        yaml.safe_dump(
            {
                "image": image.name,
                "mode": "trinary",
                "resolution": 0.05,
                "origin": [-1.0, -2.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def _write_unknown_grid(map_url: Path) -> None:
    _write_grid(map_url)
    map_url.with_suffix(".pgm").write_bytes(b"P5\n3 2\n255\n" + bytes([205] * 6))


def _write_shutdown_assets(session: dict) -> None:
    root = Path(session["root"])
    pcd = root / "pcd" / "localization_map.pcd"
    pcd.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )
    (root / "pcd" / "localization_map.processing.yaml").write_text(
        yaml.safe_dump({"state": "ready", "map_file": pcd.name}),
        encoding="utf-8",
    )
    bag = root / "rosbag" / "mapping_capture"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")


def _build_candidate(_session, paths, baseline_yaml: Path, _timeout: float):
    output = paths.root / "generated_static_candidate"
    output.mkdir(exist_ok=True)
    metadata = yaml.safe_load(baseline_yaml.read_text(encoding="utf-8"))
    source_image = baseline_yaml.parent / metadata["image"]
    candidate_image = output / "ground_temporal.pgm"
    candidate_yaml = output / "ground_temporal.yaml"
    candidate_image.write_bytes(source_image.read_bytes())
    metadata["image"] = candidate_image.name
    candidate_yaml.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    report = {
        "schema_version": 2,
        "eligible_for_candidate": True,
        "selected_candidate": "ground_temporal",
        "clouds": 1,
        "odometry_poses": 1,
        "pose_mismatches": 0,
        "ground_plane_failures": 0,
        "empty_clouds": 0,
        "parameters": {"resolution": 0.05},
        "canvas": {
            "width": 3,
            "height": 2,
            "origin": [-1.0, -2.0],
            "padding_m": 0.0,
        },
        "raytrace": {
            "enabled": True,
            "selected_clouds": 1,
            "rays": 1,
            "source_free_pixels": 4,
            "free_pixels": 4,
            "new_free_pixels": 0,
        },
        "variants": {
            "ground_temporal": {
                "evidence_cells_clipped": 0,
                "swept_cells_clipped": 0,
                "occupied_pixels": 1,
                "known_edge_margin_cells": {
                    "left": 0,
                    "top": 0,
                    "right": 0,
                    "bottom": 0,
                },
            }
        },
    }
    report_path = output / "comparison_report.json"
    report_path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
    return {
        "map_yaml": candidate_yaml,
        "map_image": candidate_image,
        "report_path": report_path,
        "report": report,
    }


def _mapping_session(repository: MappingSessionRepository, map_id: str = "greenhouse_01"):
    session, launch_arguments = repository.prepare(
        map_id,
        {"platform_profile": "bunker", "start_rviz": "true"},
    )
    return repository.update(session, "MAPPING"), launch_arguments


def test_action_timeout_is_finite_and_bounded():
    assert mapping_session_timeout(0.0) == 120.0
    assert mapping_session_timeout(0.25) == 0.25
    assert mapping_session_timeout(300.0) == 300.0
    for value in (-1.0, 300.1, float("inf"), float("nan")):
        with pytest.raises(MappingSessionError) as error:
            mapping_session_timeout(value)
        assert error.value.code == "invalid_timeout"


def test_prepare_owns_artifact_paths_and_rejects_unsafe_overrides(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, arguments = repository.prepare(
        "greenhouse_01", {"start_sensor": "false", "use_sim_time": "true"}
    )

    assert session["state"] == "PREPARED"
    assert arguments["runtime_dir"] == session["root"]
    assert arguments["map_name"] == "greenhouse_01"
    assert arguments["mapping_output_dir"] == session["pcd_output_dir"]
    assert arguments["record_bag"] == "true"
    assert arguments["bag_profile"] == "mapping"
    assert arguments["start_sensor"] == "false"

    with pytest.raises(MappingSessionError) as error:
        repository.prepare("second", {"runtime_dir": "/tmp/escape"})
    assert error.value.code == "owned_argument"

    with pytest.raises(MappingSessionError) as error:
        repository.prepare("second", {"not_a_launch_argument": "true"})
    assert error.value.code == "unknown_argument"

    with pytest.raises(MappingSessionError) as error:
        repository.prepare("second")
    assert error.value.code == "session_active"


def test_finalize_saves_live_grid_before_stopping_and_waits_for_all_assets(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository)
    calls = []

    def save_grid(map_url: Path, _timeout: float) -> None:
        calls.append("save")
        _write_grid(map_url)

    def stop_mapping(_timeout: float) -> None:
        calls.append("stop")
        assert (Path(session["root"]) / "greenhouse_01.yaml").is_file()
        _write_shutdown_assets(session)

    result = repository.finalize_capture(
        session["session_id"],
        save_grid=save_grid,
        stop_mapping=stop_mapping,
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )

    assert calls == ["save", "stop"]
    assert result["state"] == "CANDIDATE_READY"
    assert result["grid_statistics"] == {
        "width": 3,
        "height": 2,
        "total_cells": 6,
        "free_cells": 4,
        "occupied_cells": 1,
        "unknown_cells": 1,
        "free_ratio": 4 / 6,
        "occupied_ratio": 1 / 6,
        "unknown_ratio": 1 / 6,
    }
    record = yaml.safe_load(repository.paths(result).processing_record.read_text(encoding="utf-8"))
    assert record["pcd_sha256"].startswith("sha256:")
    assert not list(Path(result["root"]).rglob("*.tmp"))


def test_grid_save_failure_keeps_mapping_running(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository)
    stopped = False

    def stop_mapping(_timeout: float) -> None:
        nonlocal stopped
        stopped = True

    with pytest.raises(MappingSessionError) as error:
        repository.finalize_capture(
            session["session_id"],
            save_grid=lambda _path, _timeout: (_ for _ in ()).throw(RuntimeError("save failed")),
            stop_mapping=stop_mapping,
            build_candidate=_build_candidate,
            timeout_s=1.0,
        )

    assert error.value.code == "grid_save_failed"
    assert stopped is False
    assert repository.load(session["session_id"])["state"] == "MAPPING"


def test_all_unknown_grid_keeps_mapping_running_before_normal_stop(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository)
    stopped = False

    def stop_mapping(_timeout: float) -> None:
        nonlocal stopped
        stopped = True

    with pytest.raises(MappingSessionError) as error:
        repository.finalize_capture(
            session["session_id"],
            save_grid=lambda path, _timeout: _write_unknown_grid(path),
            stop_mapping=stop_mapping,
            build_candidate=_build_candidate,
            timeout_s=1.0,
        )

    assert error.value.code == "grid_save_failed"
    assert "free=0, occupied=0, unknown=6" in str(error.value)
    assert stopped is False
    failed = repository.load(session["session_id"])
    assert failed["state"] == "MAPPING"
    assert failed["last_error_code"] == "grid_save_failed"


def test_missing_shutdown_assets_fail_closed_after_normal_stop(tmp_path):
    now = [0.0]

    def sleep(duration: float) -> None:
        now[0] += duration

    repository = MappingSessionRepository(
        tmp_path / "runtime", clock=lambda: now[0], sleep=sleep
    )
    session, _ = _mapping_session(repository)

    with pytest.raises(MappingSessionError) as error:
        repository.finalize_capture(
            session["session_id"],
            save_grid=lambda path, _timeout: _write_grid(path),
            stop_mapping=lambda _timeout: None,
            build_candidate=_build_candidate,
            timeout_s=1.0,
        )

    assert error.value.code == "asset_timeout"
    assert repository.load(session["session_id"])["state"] == "CAPTURE_FAILED"


def test_static_candidate_failure_is_retryable_without_restarting_mapping(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository)

    with pytest.raises(MappingSessionError) as error:
        repository.finalize_capture(
            session["session_id"],
            save_grid=lambda path, _timeout: _write_grid(path),
            stop_mapping=lambda _timeout: _write_shutdown_assets(session),
            build_candidate=lambda *_values: (_ for _ in ()).throw(
                RuntimeError("offline failed")
            ),
            timeout_s=2.0,
        )

    assert error.value.code == "offline_candidate_failed"
    failed = repository.load(session["session_id"])
    assert failed["state"] == "CANDIDATE_BUILD_FAILED"
    assert Path(failed["online_preview_map_yaml"]).is_file()

    retried = repository.finalize_capture(
        session["session_id"],
        save_grid=lambda *_values: pytest.fail("retry must not save the online grid"),
        stop_mapping=lambda *_values: pytest.fail("retry must not stop mapping again"),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )

    assert retried["state"] == "CANDIDATE_READY"
    assert retried["candidate_quality"]["raytrace_clouds"] == 1


def test_static_candidate_report_rejects_clipped_evidence(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository)

    def clipped_builder(*values):
        generated = _build_candidate(*values)
        generated["report"]["variants"]["ground_temporal"][
            "evidence_cells_clipped"
        ] = 1
        Path(generated["report_path"]).write_text(
            json.dumps(generated["report"], sort_keys=True), encoding="utf-8"
        )
        return generated

    with pytest.raises(MappingSessionError) as error:
        repository.finalize_capture(
            session["session_id"],
            save_grid=lambda path, _timeout: _write_grid(path),
            stop_mapping=lambda _timeout: _write_shutdown_assets(session),
            build_candidate=clipped_builder,
            timeout_s=2.0,
        )

    assert error.value.code == "candidate_quality_failed"
    assert repository.load(session["session_id"])["state"] == "CANDIDATE_BUILD_FAILED"


def test_candidate_edit_commits_as_a_new_immutable_map_version(tmp_path):
    runtime = tmp_path / "runtime"
    repository = MappingSessionRepository(runtime)
    session, _ = _mapping_session(repository)
    candidate = repository.finalize_capture(
        session["session_id"],
        save_grid=lambda path, _timeout: _write_grid(path),
        stop_mapping=lambda _timeout: _write_shutdown_assets(session),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )
    candidate_image = repository.paths(candidate).map_image
    edited_bytes = candidate_image.read_bytes()[:-1] + bytes([0])
    candidate_image.write_bytes(edited_bytes)

    registry = MapRegistry(runtime / "maps")
    committed = repository.commit(
        session["session_id"], map_registry=registry, activate=True
    )

    assert committed["state"] == "REGISTERED"
    assert committed["map_version_id"].startswith("map_")
    versions = registry.list_versions(map_id="greenhouse_01")
    assert len(versions) == 1
    assert versions[0]["active"] == 1
    assert Path(committed["registered_map_yaml"]).is_file()
    assert Path(committed["tasks_directory"]).is_dir()
    manifest = yaml.safe_load(Path(versions[0]["manifest_path"]).read_text(encoding="utf-8"))
    registered_image = Path(versions[0]["manifest_path"]).parent / manifest["assets"]["navigation_pgm"]["path"]
    assert registered_image.read_bytes() == edited_bytes
    assert candidate_image.read_bytes() == edited_bytes


def test_commit_restores_trinary_mode_omitted_by_qt_candidate_save(tmp_path):
    runtime = tmp_path / "runtime"
    repository = MappingSessionRepository(runtime)
    session, _ = _mapping_session(repository)
    candidate = repository.finalize_capture(
        session["session_id"],
        save_grid=lambda path, _timeout: _write_grid(path),
        stop_mapping=lambda _timeout: _write_shutdown_assets(session),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )
    candidate_yaml = Path(candidate["map_url"]).with_suffix(".yaml")
    metadata = yaml.safe_load(candidate_yaml.read_text(encoding="utf-8"))
    metadata.pop("mode")
    candidate_yaml.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )

    committed = repository.commit(
        session["session_id"], map_registry=MapRegistry(runtime / "maps")
    )

    assert committed["state"] == "REGISTERED"
    assert committed["candidate_mode_recovered"] is True
    assert yaml.safe_load(candidate_yaml.read_text(encoding="utf-8"))["mode"] == "trinary"
    registered_yaml = Path(committed["registered_map_yaml"])
    assert yaml.safe_load(registered_yaml.read_text(encoding="utf-8"))["mode"] == "trinary"


def test_commit_rejects_explicit_non_trinary_candidate_mode(tmp_path):
    runtime = tmp_path / "runtime"
    repository = MappingSessionRepository(runtime)
    session, _ = _mapping_session(repository)
    candidate = repository.finalize_capture(
        session["session_id"],
        save_grid=lambda path, _timeout: _write_grid(path),
        stop_mapping=lambda _timeout: _write_shutdown_assets(session),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )
    candidate_yaml = Path(candidate["map_url"]).with_suffix(".yaml")
    metadata = yaml.safe_load(candidate_yaml.read_text(encoding="utf-8"))
    metadata["mode"] = "scale"
    candidate_yaml.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )

    with pytest.raises(MappingSessionError) as error:
        repository.commit(
            session["session_id"], map_registry=MapRegistry(runtime / "maps")
        )

    assert error.value.code == "candidate_grid_invalid"
    failed = repository.load(session["session_id"])
    assert failed["state"] == "COMMIT_FAILED"
    assert failed["candidate_mode_recovered"] is False
    assert yaml.safe_load(candidate_yaml.read_text(encoding="utf-8"))["mode"] == "scale"


def test_commit_revalidates_edited_candidate_grid_content(tmp_path):
    runtime = tmp_path / "runtime"
    repository = MappingSessionRepository(runtime)
    session, _ = _mapping_session(repository)
    candidate = repository.finalize_capture(
        session["session_id"],
        save_grid=lambda path, _timeout: _write_grid(path),
        stop_mapping=lambda _timeout: _write_shutdown_assets(session),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )
    _write_unknown_grid(Path(candidate["map_url"]))

    with pytest.raises(MappingSessionError) as error:
        repository.commit(
            session["session_id"], map_registry=MapRegistry(runtime / "maps")
        )

    assert error.value.code == "candidate_grid_invalid"
    failed = repository.load(session["session_id"])
    assert failed["state"] == "COMMIT_FAILED"
    assert failed["last_error_code"] == "candidate_grid_invalid"
    assert not (runtime / "maps" / "greenhouse_01").exists()


def test_discard_is_recoverable_and_does_not_block_the_next_session(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = repository.prepare("temporary")
    discarded = repository.discard(session["session_id"])

    assert discarded["state"] == "DISCARDED"
    assert Path(discarded["session_file"]).is_file()
    assert ".trash" in Path(discarded["root"]).parts
    with pytest.raises(MappingSessionError) as error:
        repository.load(session["session_id"])
    assert error.value.code == "session_not_found"
    replacement, _ = repository.prepare("replacement")
    assert replacement["state"] == "PREPARED"


def test_failed_registry_version_is_recorded_for_recoverable_cleanup(tmp_path):
    repository = MappingSessionRepository(tmp_path / "runtime")
    session, _ = _mapping_session(repository, "invalid_candidate")
    repository.finalize_capture(
        session["session_id"],
        save_grid=lambda path, _timeout: _write_grid(path),
        stop_mapping=lambda _timeout: _write_shutdown_assets(session),
        build_candidate=_build_candidate,
        timeout_s=2.0,
    )

    class InvalidRegistry:
        root = tmp_path / "runtime" / "maps"

        @staticmethod
        def import_legacy(**_values):
            return SimpleNamespace(
                valid=False,
                errors=("candidate validation failed",),
                map_version_id="map_20260727_120000_deadbeef",
            )

    with pytest.raises(MappingSessionError) as error:
        repository.commit(session["session_id"], map_registry=InvalidRegistry())

    assert error.value.code == "map_registration_failed"
    failed = repository.load(session["session_id"])
    assert failed["state"] == "COMMIT_FAILED"
    assert failed["failed_map_version_id"] == "map_20260727_120000_deadbeef"
