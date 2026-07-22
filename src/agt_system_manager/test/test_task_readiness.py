from agt_system_manager.readiness import ReadinessInputs, evaluate_task_readiness


def ready_inputs() -> ReadinessInputs:
    return ReadinessInputs(
        active_mode="NAVIGATION",
        map_id="greenhouse_01",
        map_version_id="map_20260722_120000_ab12cd34",
        map_ready=True,
        navigation_map_valid=True,
        localization_pcd_valid=True,
        active_map_hash="sha256:" + "a" * 64,
        localization_map_id="greenhouse_01",
        localization_map_hash="sha256:" + "a" * 64,
        localization_state="TRACKING",
        pose_valid=True,
        localization_accepted=True,
        status_stale=False,
        emergency_stop=False,
        chassis_connected=True,
        safety_allows_navigation=True,
        nav2_active=True,
        tf_chain_fresh=True,
        task_valid=True,
        health_revision=7,
    )


def test_readiness_is_ready_only_when_all_contracts_hold():
    result = evaluate_task_readiness(ready_inputs())
    assert result.ready
    assert result.blocker_codes == []


def test_readiness_reports_independent_blockers():
    inputs = ready_inputs()
    inputs.active_mode = "MAPPING"
    inputs.localization_map_hash = "sha256:" + "b" * 64
    inputs.emergency_stop = True
    inputs.task_valid = False
    result = evaluate_task_readiness(inputs)
    assert not result.ready
    assert result.blocker_codes == [
        "MODE_NOT_NAVIGATION",
        "LOCALIZATION_MAP_MISMATCH",
        "EMERGENCY_STOP",
        "TASK_INVALID",
    ]


def test_stale_localization_and_missing_runtime_inputs_fail_closed():
    inputs = ReadinessInputs()
    result = evaluate_task_readiness(inputs)
    assert not result.ready
    assert "MODE_NOT_NAVIGATION" in result.blocker_codes
    assert "LOCALIZATION_STATUS_STALE" in result.blocker_codes
    assert "SAFETY_NOT_READY" in result.blocker_codes
