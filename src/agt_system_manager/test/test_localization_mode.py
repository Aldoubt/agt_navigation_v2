from agt_system_manager.localization_mode import AUTO_ON_START, AUTO_RECOVERY, MANUAL_ONLY, RelocalizationPolicy
from agt_system_manager.localization_thresholds import evaluate_localization_display


def test_manual_mode_never_triggers():
    policy = RelocalizationPolicy()
    assert policy.mode == MANUAL_ONLY
    assert not policy.should_trigger(now=0.0, localization_state="DEGRADED", map_ready=True, cloud_healthy=True)


def test_startup_mode_is_one_shot_even_after_failure():
    policy = RelocalizationPolicy(mode=AUTO_ON_START, max_attempts=3)
    assert policy.should_trigger(now=0.0, localization_state="LOST", map_ready=True, cloud_healthy=True)
    policy.start_attempt(0.0)
    policy.finish_attempt(False)
    assert not policy.should_trigger(now=100.0, localization_state="LOST", map_ready=True, cloud_healthy=True)
    assert policy.exhausted


def test_recovery_has_cooldown_and_attempt_bound():
    policy = RelocalizationPolicy(mode=AUTO_RECOVERY, max_attempts=2, retry_cooldown_s=10.0, total_timeout_s=30.0)
    assert policy.should_trigger(now=0.0, localization_state="DEGRADED", map_ready=True, cloud_healthy=True)
    policy.start_attempt(0.0)
    policy.finish_attempt(False)
    assert not policy.should_trigger(now=5.0, localization_state="DEGRADED", map_ready=True, cloud_healthy=True)
    assert policy.should_trigger(now=10.0, localization_state="DEGRADED", map_ready=True, cloud_healthy=True)
    policy.start_attempt(10.0)
    policy.finish_attempt(False)
    assert not policy.should_trigger(now=30.0, localization_state="DEGRADED", map_ready=True, cloud_healthy=True)
    assert policy.exhausted


def test_display_uses_supplied_runtime_thresholds():
    result = evaluate_localization_display(
        {"pose_valid": True, "localization_accepted": True, "status_stale": False, "ambiguous_result": False, "error_code": 0, "fitness_score": 1.0, "translation_innovation": 0.5, "yaw_innovation": 0.2, "ambiguity_score": 0.01},
        {"/**": {"ros__parameters": {"fitness_score_threshold": 2.0, "max_translation_innovation": 5.0, "max_yaw_innovation": 1.0, "ambiguity_ratio": 0.1}}},
    )
    assert result["thresholds"]["yaw_innovation"] == 1.0
    assert result["margins"]["yaw_innovation"] == 0.8
    assert result["level"] in {"EXCELLENT", "ACCEPTABLE"}
