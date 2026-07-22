from pathlib import Path

import pytest

from agt_web_console.service import WebConsoleConfig, WebConsoleService
from agt_web_console.offline_backend import OfflineConsoleBackend


class ModeController:
    def __init__(self):
        self.calls = []

    def start(self, profile, arguments):
        self.calls.append(("start", profile, arguments))
        return {"profile": profile, "started": True}

    def stop_all(self):
        self.calls.append(("stop",))
        return []

    def status(self):
        return {"active_mode": "IDLE", "processes": []}

    def relocalize(self, values):
        self.calls.append(("relocalize", values))
        return {"success": False, "failure_reason": "test backend"}


class Experiments:
    def create(self, **values):
        self.values = values
        return "exp_20260722_120000_trial_ab12cd"

    def list(self, **_filters):
        return []


def test_remote_listener_requires_token_and_loopback_is_default():
    with pytest.raises(ValueError):
        WebConsoleConfig(host="0.0.0.0").validate()
    WebConsoleConfig(host="0.0.0.0", token="local-test-token").validate()


def test_offline_backend_validates_profiles_and_never_opens_task_gate(tmp_path):
    backend = OfflineConsoleBackend(
        {
            "mapping": {
                "mode": "MAPPING",
                "description": "offline mapping",
                "command": ["ros2", "launch", "test", "mapping.launch.py"],
                "allowed_argument_keys": ["runtime_dir", "use_sim_time"],
            }
        },
        runtime_dir=tmp_path,
    )
    result = backend.start("mapping", {"runtime_dir": str(tmp_path), "use_sim_time": "false"})
    assert result["offline"] is True
    assert backend.readiness()["ready"] is False
    assert backend.readiness()["blocker_codes"] == ["OFFLINE_MODE"]
    relocalization = backend.relocalize({"mode": "AUTO_SEARCH", "max_candidates": 8, "timeout_s": 2})
    assert relocalization["success"] is True
    assert relocalization["offline"] is True
    assert backend.localization()["simulated"] is True
    backend.stop_all()


def test_service_can_switch_between_configured_backends(tmp_path):
    ros = ModeController()
    offline = OfflineConsoleBackend(
        {
            "sensor_only": {
                "mode": "SENSOR_ONLY",
                "command": ["ros2", "launch", "test", "sensor.launch.py"],
                "allowed_argument_keys": ["use_sim_time"],
            }
        },
        runtime_dir=tmp_path,
    )
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=ros,
        localization_controller=ros,
        experiment_manager=Experiments(),
        backends={
            "ros": {"mode_controller": ros, "localization_controller": ros},
            "offline": {"health_provider": offline.health, "readiness_provider": offline.readiness, "mode_controller": offline, "localization_controller": offline},
        },
    )
    assert service.set_backend("offline")["offline"] is True
    assert service.set_mode("sensor_only", {"use_sim_time": "false"})["offline"] is True
    with pytest.raises(RuntimeError, match="离线测试模式"):
        service.experiment_action("missing", "start_bag")
    service.stop_mode()
    assert service.set_backend("ros")["offline"] is False


def test_console_delegates_only_configured_profile_and_audits_writes(tmp_path):
    controller = ModeController()
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        health_provider=lambda: {"overall_state": "OK"},
        readiness_provider=lambda: {"ready": False, "blocker_codes": ["MODE_NOT_NAVIGATION"]},
        mode_controller=controller,
        experiment_manager=Experiments(),
    )
    result = service.set_mode("mapping", {"map_name": "greenhouse_01"})
    assert result["started"]
    assert controller.calls == [("start", "mapping", {"map_name": "greenhouse_01"})]
    assert service.overview()["task_readiness"]["ready"] is False
    service.stop_mode()
    audit = (tmp_path / "logs" / "web_console_audit.jsonl").read_text(encoding="utf-8")
    assert "system_mode_start" in audit and "system_mode_stop" in audit


def test_logs_are_restricted_to_managed_roots(tmp_path):
    service = WebConsoleService(WebConsoleConfig(runtime_dir=str(tmp_path)))
    with pytest.raises(ValueError):
        service.logs("/etc")


def test_relocalization_is_a_structured_action_request(tmp_path):
    controller = ModeController()
    service = WebConsoleService(WebConsoleConfig(runtime_dir=str(tmp_path)), localization_controller=controller)
    result = service.relocalize({"mode": "AUTO_SEARCH"})
    assert result["success"] is False
    assert controller.calls == [("relocalize", {"mode": "AUTO_SEARCH"})]
    assert "localization_relocalize" in (tmp_path / "logs" / "web_console_audit.jsonl").read_text(encoding="utf-8")


def test_static_console_is_chinese_and_exposes_ordered_workflow_controls():
    static_root = Path(__file__).parents[1] / "static"
    html = (static_root / "index.html").read_text(encoding="utf-8")
    javascript = (static_root / "app.js").read_text(encoding="utf-8")

    assert '<html lang="zh-CN">' in html
    assert "从系统检查到任务执行" in html
    assert "runtime-backend" in html
    assert "switch-backend" in html
    assert "relocalize-action-mode" in html
    assert 'action: "relocalize"' in javascript
    assert '"/api/v1/localization/relocalize"' in javascript
    assert "timeout_s:" in javascript
    assert "timeout_sec:" not in javascript
    assert '"/api/v1/runtime/backend"' in javascript
    assert "profileArguments" in javascript
