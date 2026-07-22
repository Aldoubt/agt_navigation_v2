from agt_system_manager.health import ERROR, OK, WARN, HealthEvaluator, TopicObservation


CONTRACT = {
    "health": {
        "components": [
            {
                "component_id": "required_cloud",
                "display_name": "Cloud",
                "required_in_modes": ["MAPPING"],
                "required_topics": [
                    {"name": "/cloud", "min_rate_hz": 2.0, "max_age_sec": 1.0}
                ],
                "required_frames": ["odom->base_footprint"],
            },
            {
                "component_id": "optional_gui",
                "display_name": "GUI",
                "required_in_modes": ["MAPPING"],
                "optional": True,
                "conditions": [{"name": "gui.alive", "expected": True, "severity": "WARN"}],
            },
        ]
    }
}


def test_normal_rate_and_frame_is_ok():
    evaluator = HealthEvaluator(CONTRACT)
    result = evaluator.evaluate(
        "MAPPING",
        {"/cloud": TopicObservation(5, 0.0, 2.0)},
        now=2.2,
        frames={"odom->base_footprint"},
        conditions={"gui.alive": True},
    )
    assert result.overall_state == OK
    assert result.components[0].observed_rate_hz == 2.0


def test_low_rate_and_expired_message_are_reported():
    evaluator = HealthEvaluator(CONTRACT)
    result = evaluator.evaluate(
        "MAPPING",
        {"/cloud": TopicObservation(2, 0.0, 1.0)},
        now=3.0,
        frames={"odom->base_footprint"},
        conditions={"gui.alive": False},
    )
    assert result.overall_state == ERROR
    assert "rate_low" in result.components[0].detail
    assert "message_expired" in result.components[0].detail
    assert result.components[1].state == WARN


def test_missing_required_topic_frame_and_optional_component():
    evaluator = HealthEvaluator(CONTRACT)
    result = evaluator.evaluate("MAPPING", now=0.0, conditions={"gui.alive": False})
    assert result.overall_state == ERROR
    assert result.components[0].state == ERROR
    assert "/cloud" in result.components[0].missing_topics
    assert "odom->base_footprint" in result.components[0].missing_frames
    assert result.components[1].state == WARN


def test_health_recovers_after_observation_returns():
    evaluator = HealthEvaluator(CONTRACT)
    evaluator.evaluate("MAPPING", now=0.0, conditions={"gui.alive": False})
    result = evaluator.evaluate(
        "MAPPING",
        {"/cloud": TopicObservation(5, 1.0, 3.0)},
        now=3.2,
        frames={"odom->base_footprint"},
        conditions={"gui.alive": True},
    )
    assert result.overall_state == OK


def test_components_not_required_in_current_mode_do_not_make_health_unknown():
    evaluator = HealthEvaluator(
        {"health": {"components": [
            {"component_id": "sensor", "required_in_modes": ["SENSOR_ONLY"]},
            {"component_id": "navigation", "required_in_modes": ["NAVIGATION"]},
        ]}}
    )
    result = evaluator.evaluate("SENSOR_ONLY")
    assert result.overall_state == OK
    assert result.components[1].state == "UNKNOWN"
