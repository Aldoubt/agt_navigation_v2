import hashlib
import threading
from types import SimpleNamespace
from pathlib import Path

import pytest
import yaml

from agt_map_manager.registry import MapRegistry
from agt_experiment_manager.manager import ExperimentManager
from agt_web_console.instance_lock import WebConsoleInstanceLock
from agt_web_console.ros_bridge import RosConsoleBridge
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


class MappingController(ModeController):
    def __init__(self):
        super().__init__()
        self.active_mode = "IDLE"

    def start(self, profile, arguments):
        result = super().start(profile, arguments)
        self.active_mode = "MAPPING"
        return result

    def stop_all(self):
        self.active_mode = "IDLE"
        return super().stop_all()

    def status(self):
        return {"active_mode": self.active_mode, "processes": []}


class MappingSessionController:
    def __init__(self, root: Path, *, fail_finalize: bool = False):
        self.root = root
        self.calls = []
        self.session = None
        self.fail_finalize = fail_finalize

    def manage_mapping_session(self, operation, **values):
        self.calls.append((operation, values))
        if operation == "status":
            return dict(self.session) if self.session else {
                "success": False,
                "available": False,
                "state": "IDLE",
            }
        if operation == "start":
            map_id = values["map_id"]
            self.session = {
                "success": True,
                "available": True,
                "state": "MAPPING",
                "session_id": "mapping_test_0001",
                "map_id": map_id,
                "map_name": map_id,
                "root": str(self.root),
                "message": "mapping capture is running",
            }
        elif operation == "finalize":
            if self.fail_finalize:
                self.session["state"] = "CAPTURE_FAILED"
                raise RuntimeError("capture assets are incomplete")
            self.session.update(
                {
                    "state": "CANDIDATE_READY",
                    "candidate_map_yaml": str(self.root / "candidate.yaml"),
                    "candidate_map_image": str(self.root / "candidate.pgm"),
                    "localization_pcd": str(self.root / "localization_map.pcd"),
                    "processing_record": str(self.root / "localization_map.processing.yaml"),
                    "bag_directory": str(self.root / "rosbag"),
                    "message": "candidate ready",
                }
            )
        elif operation == "commit":
            self.session.update(
                {
                    "state": "REGISTERED",
                    "map_version_id": "map_20260727_120000_1234abcd",
                    "version_id": "map_20260727_120000_1234abcd",
                    "message": "registered",
                }
            )
        elif operation == "discard":
            self.session.update(
                {"state": "DISCARDED", "available": False, "message": "discarded"}
            )
        return dict(self.session)


def test_remote_listener_requires_token_and_loopback_is_default():
    with pytest.raises(ValueError):
        WebConsoleConfig(host="0.0.0.0").validate()
    WebConsoleConfig(host="0.0.0.0", token="local-test-token").validate()
    with pytest.raises(ValueError):
        WebConsoleConfig(can_interface="can0;sudo").validate()


def test_web_console_runtime_allows_only_one_instance(tmp_path):
    first = WebConsoleInstanceLock(tmp_path)
    second = WebConsoleInstanceLock(tmp_path)
    first.acquire()
    try:
        with pytest.raises(RuntimeError, match="Web 控制台已经在运行"):
            second.acquire()
    finally:
        first.release()
    second.acquire()
    second.release()


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


def test_offline_backend_simulates_selected_bag_without_ros_process(tmp_path):
    bag = tmp_path / "rosbag" / "mapping_trial"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    backend = OfflineConsoleBackend(
        {
            "sensor_only": {
                "mode": "SENSOR_ONLY",
                "command": ["ros2", "launch", "test", "sensor.launch.py"],
                "allowed_argument_keys": [],
            }
        },
        runtime_dir=tmp_path,
    )
    result = backend.start_playback("mapping_trial", rate=1.5)
    assert result["playing"] is True
    assert result["simulated"] is True
    assert "不会读取 ROS 消息" in result["message"]
    assert backend.stop_playback()["playing"] is False

    with pytest.raises(ValueError, match="不能越出"):
        backend.start_playback("../outside")


def test_offline_bag_preview_requires_mapping_and_is_marked_simulated(tmp_path):
    bag = tmp_path / "rosbag" / "mapping_trial"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    backend = OfflineConsoleBackend(
        {
            "mapping": {
                "mode": "MAPPING",
                "command": ["ros2", "launch", "test", "mapping.launch.py"],
                "allowed_argument_keys": [],
            }
        },
        runtime_dir=tmp_path,
    )

    assert backend.mapping_status()["available"] is False
    backend.start("mapping")
    assert backend.mapping_status()["available"] is False
    backend.start_playback("mapping_trial")
    map_preview = backend.mapping_status()
    pointcloud_preview = backend.mapping_pointcloud_status()
    assert map_preview["available"] is True
    assert map_preview["simulated"] is True
    assert "不是 bag 中的真实地图" in map_preview["message"]
    assert pointcloud_preview["available"] is True
    assert pointcloud_preview["simulated"] is True
    backend.stop_all()
    assert backend.mapping_status()["available"] is False


def test_ros_mapping_preview_survives_readiness_transition_into_mapping():
    bridge = object.__new__(RosConsoleBridge)
    bridge._lock = threading.RLock()
    bridge._readiness = {}
    bridge._active_mode = "MAPPING"
    bridge._mapping_map = {"available": True}
    bridge._mapping_pointcloud = {"available": True}
    bridge._pointcloud_voxels = {}
    bridge._robot_pose = {"available": True}
    bridge._notify_status = lambda: None

    message = SimpleNamespace(
        ready=False,
        active_mode="MAPPING",
        map_id="",
        map_version_id="",
        localization_state="UNKNOWN",
        health_revision=1,
        blocker_codes=[],
        blocker_messages=[],
        warning_codes=[],
        warning_messages=[],
    )
    RosConsoleBridge._readiness_callback(bridge, message)

    assert bridge._mapping_map["available"] is True
    assert bridge._mapping_pointcloud["available"] is True

    message.active_mode = "IDLE"
    RosConsoleBridge._readiness_callback(bridge, message)
    assert bridge._mapping_map["available"] is False
    assert bridge._mapping_pointcloud["available"] is False


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


def test_service_routes_offline_bag_playback_to_simulator(tmp_path):
    bag = tmp_path / "rosbag" / "selected_trial"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    ros = ModeController()
    offline = OfflineConsoleBackend({}, runtime_dir=tmp_path)
    experiments = ExperimentManager(tmp_path / "experiments", rosbag_root=tmp_path / "rosbag")
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=ros,
        experiment_manager=experiments,
        backends={
            "ros": {"mode_controller": ros},
            "offline": {"mode_controller": offline, "localization_controller": offline},
        },
    )

    service.set_backend("offline")
    result = service.bag_action("play", {"bag_id": "selected_trial", "rate": 1.25})
    assert result["simulated"] is True
    assert service.bags()["playback"]["playing"] is True
    assert service.bag_action("stop")["playing"] is False


def test_console_delegates_only_configured_profile_and_audits_writes(tmp_path):
    controller = ModeController()
    mapping_sessions = MappingSessionController(tmp_path / "mapping_sessions")
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        health_provider=lambda: {"overall_state": "OK"},
        readiness_provider=lambda: {"ready": False, "blocker_codes": ["MODE_NOT_NAVIGATION"]},
        mode_controller=controller,
        mapping_session_controller=mapping_sessions,
        experiment_manager=Experiments(),
    )
    result = service.prepare_mapping_session(
        "greenhouse_01",
        {
            "start_sensor": "true",
            "runtime_dir": "/ignored/frontend/path",
            "record_bag": "false",
        },
    )
    assert result["state"] == "MAPPING"
    assert mapping_sessions.calls == [
        (
            "start",
            {
                "map_id": "greenhouse_01",
                "arguments": {"start_sensor": "true"},
            },
        )
    ]
    assert controller.calls == []
    assert service.overview()["task_readiness"]["ready"] is False
    service.stop_mode()
    audit = (tmp_path / "logs" / "web_console_audit.jsonl").read_text(encoding="utf-8")
    assert "mapping_session_start" in audit and "system_mode_stop" in audit


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
    bridge_source = (Path(__file__).parents[1] / "agt_web_console" / "ros_bridge.py").read_text(encoding="utf-8")

    assert '<html lang="zh-CN">' in html
    assert "从系统检查到任务执行" in html
    assert "runtime-backend" in html
    assert "switch-backend" in html
    assert "access-token" in html
    assert "relocalize-action-mode" in html
    assert 'action: "relocalize"' in javascript
    assert '"/api/v1/localization/relocalize"' in javascript
    assert "timeout_s:" in javascript
    assert "timeout_sec:" not in javascript
    assert "sensor-config" in html
    assert "mapping-state-badge" in html
    assert "mapping-map-canvas" in html
    assert "mapping-pointcloud-canvas" in html
    assert "mapping-input-source" in html
    assert "mapping-map-center" in html
    assert "mapping-pointcloud-center" in html
    assert "mapping-pointcloud-rotation" in html
    assert "data-pointcloud-view=\"xz\"" in html
    assert "data-pointcloud-view=\"yz\"" in html
    assert "projection" in javascript
    assert "drawPointcloudCoordinates" in javascript
    assert "rotationDeg" in javascript
    assert "chassis-state-badge" in html
    assert "start-chassis-monitor" in html
    assert "chassis-backend" in html
    assert "record-bag-profile" in html
    assert "bag-list" in html
    assert "bag-selection" in html
    assert "play-selected-bag" in html
    assert "copy-chassis-command" in html
    assert "task-flow-note" in html
    assert "start-mapping-profile" in html
    assert "start-navigation-profile" in html
    assert "finish-mapping" in html
    assert "mapping-finish-title" in html
    assert "mapping-finish-dialog" in html
    assert "navigation-map-version" in html
    assert '"/api/v1/mapping/map"' in javascript
    assert '"/api/v1/mapping/pointcloud"' in javascript
    assert '"/api/v1/chassis/status"' in javascript
    assert '"/api/v1/bags/play"' in javascript
    assert '"/api/v1/bags/stop"' in javascript
    assert "operation_mode:=monitor" in javascript
    assert 'args.start_sensor = inputSource === "bag" ? "false" : "true"' in javascript
    assert "bindPreviewCanvas" in javascript
    assert "centerOnRobot" in javascript
    assert "_mapping_active_locked" in bridge_source
    assert "Do not clear" in bridge_source
    assert '"active_mode": self._active_mode' in bridge_source
    assert "未启动建图链，点云地图预览为空" in bridge_source
    assert "未发现 /agt/system/change_mode Action server" in bridge_source
    assert "不读取 ROS 消息" in javascript
    assert "传感器将保持运行" in javascript
    assert "user_config_path" in javascript
    assert '"/api/v1/runtime/backend"' in javascript
    assert "X-AGT-Token" in javascript
    assert "websocketUrl.searchParams" in javascript
    assert "profileArguments" in javascript
    assert '"/api/v1/mapping/session/prepare"' in javascript
    assert '"/api/v1/mapping/finish"' in javascript
    assert '"/api/v1/mapping/session"' in javascript
    assert "mapping_output_dir" not in javascript
    assert "ManageMappingSession" in bridge_source
    assert "map_version_id" in javascript
    assert "discardableFailedSession" in javascript
    assert "mapping-finish-confirm" in javascript
    assert "activeMapping" in javascript


def test_offline_mapping_session_can_be_retained_without_writing_files(tmp_path):
    profiles = {
        "mapping": {
            "mode": "MAPPING",
            "command": ["ros2", "launch", "agt_bringup", "system.launch.py"],
            "allowed_argument_keys": ["map_name", "use_sim_time"],
        }
    }
    offline = OfflineConsoleBackend(profiles, runtime_dir=tmp_path)
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=ModeController(),
        backends={
            "ros": {"mode_controller": ModeController()},
            "offline": {
                "health_provider": offline.health,
                "readiness_provider": offline.readiness,
                "mode_controller": offline,
                "localization_controller": offline,
            },
        },
    )
    service.set_backend("offline")
    service.prepare_mapping_session("offline_map")
    service.set_mode("mapping", {"map_name": "offline_map"})
    with pytest.raises(RuntimeError, match="不能创建第二个会话"):
        service.prepare_mapping_session("second_map")
    result = service.finish_mapping("retain")
    assert result["state"] == "SIMULATED_RETAINED"
    assert result["offline_map_slot"]["occupied"] is True
    with pytest.raises(RuntimeError, match="最多保留一个"):
        service.prepare_mapping_session("second_map")
    assert not (tmp_path / "mapping_sessions").exists()


def test_offline_mapping_ignores_unfinished_ros_session(tmp_path):
    real_root = tmp_path / "mapping_sessions" / "real_map" / "mapping_real"
    real_root.mkdir(parents=True)
    (real_root / "session.yaml").write_text(
        "map_name: real_map\nstate: PREPARED\nroot: %s\npcd_output_dir: %s\nmap_url: %s\n"
        % (real_root, real_root / "pcd", real_root / "real_map"),
        encoding="utf-8",
    )
    offline = OfflineConsoleBackend({}, runtime_dir=tmp_path)
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        backends={
            "ros": {"mode_controller": ModeController()},
            "offline": {"mode_controller": offline, "localization_controller": offline},
        },
    )
    service.set_backend("offline")
    session = service.prepare_mapping_session("offline_map")
    assert session["offline"] is True
    assert service.mapping_session_status()["map_name"] == "offline_map"
    assert (real_root / "session.yaml").is_file()


def test_navigation_requires_a_selected_ready_version(tmp_path):
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=ModeController(),
        map_registry=object(),
    )
    with pytest.raises(ValueError, match="必须选择一个地图版本"):
        service.set_mode("navigation", {})


def test_online_mapping_finalize_and_commit_are_separate_action_operations(tmp_path):
    controller = ModeController()
    mapping_sessions = MappingSessionController(tmp_path / "mapping_sessions")
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=controller,
        mapping_session_controller=mapping_sessions,
    )
    session = service.prepare_mapping_session("online_map", {"start_rviz": "true"})
    candidate = service.finish_mapping("retain")
    assert candidate["state"] == "CANDIDATE_READY"
    assert not candidate.get("map_version_id")
    registered = service.finish_mapping("commit")
    assert registered["state"] == "REGISTERED"
    assert registered["map_version_id"].startswith("map_")
    assert [call[0] for call in mapping_sessions.calls] == [
        "start", "status", "finalize", "status", "commit"
    ]


def test_online_mapping_id_is_fixed_when_session_starts(tmp_path):
    controller = ModeController()
    mapping_sessions = MappingSessionController(tmp_path / "mapping_sessions")
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=controller,
        mapping_session_controller=mapping_sessions,
    )
    service.prepare_mapping_session("fixed_name")
    with pytest.raises(ValueError, match="启动时固定"):
        service.finish_mapping("retain", "renamed_later")


def test_failed_online_mapping_is_discarded_through_the_action(tmp_path):
    controller = ModeController()
    mapping_sessions = MappingSessionController(
        tmp_path / "mapping_sessions", fail_finalize=True
    )
    service = WebConsoleService(
        WebConsoleConfig(runtime_dir=str(tmp_path)),
        mode_controller=controller,
        mapping_session_controller=mapping_sessions,
    )
    service.prepare_mapping_session("failed_map")
    with pytest.raises(RuntimeError, match="capture assets"):
        service.finish_mapping("retain")
    result = service.finish_mapping("delete")
    assert result["state"] == "DISCARDED"
    assert [call[0] for call in mapping_sessions.calls][-2:] == ["status", "discard"]
