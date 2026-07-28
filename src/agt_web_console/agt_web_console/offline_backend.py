"""Deterministic Web-only backend for hardware-free console checks."""

from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping

from agt_system_manager.process_manager import ProfileRegistry
import yaml


class OfflineConsoleBackend:
    """Simulate project control responses without starting ROS processes."""

    _PROFILE_MODES = {
        "sensor_only": "SENSOR_ONLY",
        "mapping": "MAPPING",
        "navigation": "NAVIGATION",
        "qt_mapping": "MAPPING",
        "qt_navigation": "NAVIGATION",
        "localization_rviz": "LOCALIZATION_DEBUG",
    }

    def __init__(
        self,
        profiles: Mapping[str, Any],
        allowed_executables: tuple[str, ...] = ("ros2", "rviz2"),
        runtime_dir: str | Path = "runtime",
    ) -> None:
        self._registry = ProfileRegistry(profiles, allowed_executables)
        self._runtime_dir = Path(runtime_dir).expanduser()
        self._lock = threading.RLock()
        self._listeners: list[Callable[[Mapping[str, Any]], None]] = []
        self._active_profile = ""
        self._active_mode = "IDLE"
        self._started_at = 0.0
        self._offline_preview_available = False
        self._localization_mode = "MANUAL_ONLY"
        self._localization = self._initial_localization()
        self._playback = {
            "playing": False,
            "bag_id": "",
            "pid": 0,
            "returncode": None,
            "simulated": True,
            "message": "离线模式未进行 bag 模拟回放",
        }

    @staticmethod
    def _initial_localization() -> dict[str, Any]:
        return {
            "state": 0,
            "pose_valid": False,
            "localization_accepted": False,
            "has_converged": False,
            "ambiguous_result": False,
            "status_stale": False,
            "error_code": 0,
            "message": "等待离线模拟重定位",
            "backend": "offline-simulator",
            "candidate_source": "",
            "candidate_id": "",
            "map_id": "offline_demo",
            "map_hash": "",
            "fitness_score": 0.0,
            "overlap_ratio": 0.0,
            "inlier_ratio": 0.0,
            "ambiguity_score": 0.0,
            "translation_innovation": 0.0,
            "yaw_innovation": 0.0,
            "runtime_ms": 0.0,
            "tested_candidates": 0,
            "total_candidates": 0,
            "consecutive_successes": 0,
            "consecutive_failures": 0,
            "simulated": True,
        }

    def add_status_listener(self, callback: Callable[[Mapping[str, Any]], None]) -> None:
        with self._lock:
            self._listeners.append(callback)

    def _emit(self) -> None:
        with self._lock:
            listeners = list(self._listeners)
        event = {
            "type": "status",
            "robot_state": self.robot_state(),
            "mission": self.mission_status(),
            "health": self.health(),
            "task_readiness": self.readiness(),
            "localization": self.localization(),
            "mode": self.status(),
        }
        for callback in listeners:
            callback(event)

    def robot_state(self) -> dict[str, Any]:
        with self._lock:
            active_mode = self._active_mode
            active_profile = self._active_profile
        return {
            "revision": int(time.monotonic() * 10),
            "system_mode": active_mode,
            "active_profile": active_profile,
            "managed_process_count": int(bool(active_profile)),
            "running_process_count": int(bool(active_profile)),
            "system_health_known": True,
            "task_readiness_known": True,
            "active_map_known": False,
            "localization_status_known": True,
            "mission_status_known": True,
            "mission": self.mission_status(),
            "nav2_state": "INACTIVE",
            "safety_status_known": True,
            "safety_motion_enabled": False,
            "emergency_stop": False,
            "estop_latched": False,
            "navigation_ready": False,
            "chassis_status_known": False,
            "chassis_connected": False,
            "bag_status_known": True,
            "bag_session": self.playback_status(),
            "blocker_codes": ["OFFLINE_MODE"],
            "blocker_messages": ["离线模拟后端不会进入可执行状态"],
            "message": "离线模拟状态；不可作为真实机器人 READY 证据",
            "simulated": True,
        }

    @staticmethod
    def mission_status() -> dict[str, Any]:
        return {
            "state": "IDLE",
            "mission_id": "",
            "message": "离线模式禁止执行 Mission",
            "simulated": True,
        }

    def _sensor_active(self) -> bool:
        return self._active_profile in {"sensor_only", "mapping", "navigation"}

    def health(self) -> dict[str, Any]:
        with self._lock:
            sensor_active = self._sensor_active()
            localization_ready = self._localization["localization_accepted"]
            mapping_preview = self._active_mode == "MAPPING" and self._offline_preview_available
        return {
            "overall_state": "OK",
            "revision": int(time.monotonic() * 10),
            "blocker_codes": ["OFFLINE_MODE"],
            "blocker_messages": ["离线模拟后端不会连接真实传感器、车辆或安全控制链"],
            "warning_codes": ["SIMULATED_INPUTS"],
            "warning_messages": ["当前状态和重定位结果均为网页模拟数据"],
            "components": [
                self._component("mid360_pointcloud", "MID360 点云", sensor_active, "离线模拟点云"),
                self._component("imu", "IMU", sensor_active, "离线模拟 IMU"),
                self._component("agt_localization", "定位模块", localization_ready, "离线模拟定位结果"),
                self._component("agt_safety", "安全链", False, "离线模式不启动安全链"),
                self._component("agt_chassis", "底盘", False, "未连接车辆，离线模式不发送底盘命令"),
                self._component("fast_livo_odometry", "FAST-LIVO2 里程计", mapping_preview, "离线 bag 建图预览输入"),
                self._component("registered_cloud", "注册点云", mapping_preview, "离线 bag 建图预览输入"),
                self._component("mapping_occupancy", "二维建图地图", mapping_preview, "离线 bag 建图预览输入"),
            ],
        }

    @staticmethod
    def _component(component_id: str, display_name: str, active: bool, detail: str) -> dict[str, Any]:
        return {
            "component_id": component_id,
            "display_name": display_name,
            "state": "OK" if active else "UNKNOWN",
            "required": True,
            "present": active,
            "observed_rate_hz": 10.0 if active else None,
            "message_age_sec": 0.05 if active else None,
            "detail": detail,
            "simulated": True,
        }

    def readiness(self) -> dict[str, Any]:
        return {
            "ready": False,
            "active_mode": self._active_mode,
            "map_id": "offline_demo",
            "map_version_id": "",
            "localization_state": int(self._localization["state"]),
            "health_revision": int(time.monotonic() * 10),
            "blocker_codes": ["OFFLINE_MODE"],
            "blocker_messages": ["离线测试模式禁止任务执行、速度输出和车辆控制"],
            "warning_codes": ["SIMULATED_LOCALIZATION"],
            "warning_messages": ["网页上的定位成功只代表交互链路可用，不代表真实定位成功"],
            "simulated": True,
        }

    def localization(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._localization)

    def status(self) -> dict[str, Any]:
        with self._lock:
            processes = []
            if self._active_profile:
                processes.append({
                    "pid": 0,
                    "profile": self._active_profile,
                    "mode": self._active_mode,
                    "started_at": self._started_at,
                    "returncode": None,
                    "simulated": True,
                    "command": ["offline-simulator", self._active_profile],
                    "log_path": str(self._runtime_dir / "logs" / "offline" / f"{self._active_profile}.log"),
                })
            return {"active_mode": self._active_mode, "processes": processes, "backend": "offline", "simulated": True}

    def start(self, profile: str, arguments: Mapping[str, Any] | None = None) -> dict[str, Any]:
        profile = str(profile)
        launch_profile = self._registry.get(profile)
        launch_profile.build_command(arguments or {})
        with self._lock:
            self._active_profile = profile
            self._active_mode = self._PROFILE_MODES[profile]
            self._offline_preview_available = self._active_mode == "MAPPING" and self._playback["playing"]
            self._started_at = time.time()
            log_path = self._runtime_dir / "logs" / "offline" / f"{profile}.log"
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                f"offline profile started: {profile}\narguments: {dict(arguments or {})}\n",
                encoding="utf-8",
            )
        self._emit()
        return {
            "success": True,
            "active_mode": self._active_mode,
            "profile": profile,
            "process_ids": [],
            "log_paths": [str(log_path)],
            "message": f"已启动离线模拟模块：{launch_profile.description or profile}",
            "offline": True,
        }

    def stop_all(self) -> list[dict[str, Any]]:
        with self._lock:
            stopped = []
            if self._active_profile:
                stopped.append({"pid": 0, "profile": self._active_profile, "simulated": True})
            self._active_profile = ""
            self._active_mode = "IDLE"
            self._offline_preview_available = False
        self._emit()
        return stopped

    def stop_mode(self, mode: str) -> list[dict[str, Any]]:
        if str(mode).upper() == self._active_mode:
            return self.stop_all()
        return []

    def set_mode(self, mode: str) -> dict[str, Any]:
        if mode not in {"MANUAL_ONLY", "AUTO_ON_START", "AUTO_RECOVERY"}:
            raise ValueError("unsupported localization mode")
        with self._lock:
            self._localization_mode = mode
        self._emit()
        return {"success": True, "mode": mode, "attempts": 0, "message": f"已保存离线模拟重定位策略：{mode}"}

    def relocalize(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = dict(values or {})
        allowed = {"mode", "max_candidates", "timeout_s"}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unsupported relocalization fields: {sorted(unknown)}")
        mode = str(values.get("mode", "AUTO_SEARCH"))
        if mode not in {"AUTO_SEARCH", "LOCAL_CANDIDATES", "EXTERNAL_COARSE_POSE"}:
            raise ValueError("unsupported relocalization Action mode")
        max_candidates = min(max(int(values.get("max_candidates", 128)), 1), 1024)
        timeout_s = min(max(float(values.get("timeout_s", 30.0)), 0.1), 300.0)
        with self._lock:
            self._localization = {
                **self._localization,
                "state": 3,
                "pose_valid": True,
                "localization_accepted": True,
                "has_converged": True,
                "status_stale": False,
                "message": "离线模拟重定位成功；未读取真实 PCD、点云或车辆状态",
                "candidate_source": "offline_demo",
                "candidate_id": "offline_candidate_0001",
                "fitness_score": 0.01,
                "overlap_ratio": 1.0,
                "inlier_ratio": 1.0,
                "runtime_ms": min(timeout_s * 1000.0, 20.0),
                "tested_candidates": 1,
                "total_candidates": max_candidates,
                "consecutive_successes": 1,
                "consecutive_failures": 0,
            }
            status = dict(self._localization)
        self._emit()
        return {
            "success": True,
            "error_code": 0,
            "failure_reason": "",
            "final_status": status,
            "offline": True,
            "message": status["message"],
        }

    def _bag_path(self, bag_id: str) -> Path:
        if not isinstance(bag_id, str) or not bag_id.strip() or Path(bag_id).is_absolute():
            raise ValueError("bag_id 必须是运行目录 rosbag 下的相对 bundle 名称")
        root = (self._runtime_dir / "rosbag").resolve()
        path = (root / bag_id).resolve()
        try:
            path.relative_to(root)
        except ValueError as error:
            raise ValueError("bag_id 不能越出运行目录 rosbag") from error
        if not path.is_dir() or not (path / "metadata.yaml").is_file():
            raise ValueError("指定 bag 不是完整的 metadata.yaml bundle")
        return path

    def list_bags(self) -> list[dict[str, Any]]:
        root = (self._runtime_dir / "rosbag").resolve()
        result = []
        for metadata_path in sorted(root.glob("*/metadata.yaml"), reverse=True):
            try:
                with open(metadata_path, "r", encoding="utf-8") as stream:
                    metadata = yaml.safe_load(stream) or {}
                information = metadata.get("rosbag2_bagfile_information", {})
                result.append(
                    {
                        "bag_id": metadata_path.parent.name,
                        "message_count": int(information.get("message_count", 0)),
                        "storage_identifier": str(
                            information.get("storage_identifier", "")
                        ),
                        "mapping_input_ready": False,
                        "contains_mapping_outputs": False,
                        "contains_navigation_outputs": False,
                        "simulated": True,
                    }
                )
            except (OSError, TypeError, ValueError, yaml.YAMLError):
                continue
        return result

    def start_playback(self, bag_id: str, *, rate: float = 1.0) -> dict[str, Any]:
        path = self._bag_path(bag_id)
        try:
            rate = float(rate)
        except (TypeError, ValueError) as error:
            raise ValueError("bag 回放速率必须是数字") from error
        if not 0.1 <= rate <= 4.0:
            raise ValueError("bag 回放速率必须在 0.1 到 4.0 之间")
        with self._lock:
            if self._playback["playing"]:
                raise RuntimeError("已有 bag 正在离线模拟回放")
            self._playback = {
                "playing": True,
                "bag_id": str(path.relative_to((self._runtime_dir / "rosbag").resolve())),
                "pid": 0,
                "returncode": None,
                "rate": rate,
                "simulated": True,
                "message": "离线模拟回放：不会读取 ROS 消息、发布 topic 或启动 ros2 bag play",
            }
            self._offline_preview_available = self._active_mode == "MAPPING"
            result = dict(self._playback)
        self._emit()
        return result

    def stop_playback(self) -> dict[str, Any]:
        with self._lock:
            if self._playback["playing"]:
                bag_id = self._playback["bag_id"]
                self._playback = {
                    "playing": False,
                    "bag_id": bag_id,
                    "pid": 0,
                    "returncode": 0,
                    "simulated": True,
                    "message": "离线模拟回放已停止",
                }
            result = dict(self._playback)
        self._emit()
        return result

    def playback_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._playback)

    def mapping_status(self) -> dict[str, Any]:
        """Return a bounded, clearly simulated preview for Web workflow checks."""
        with self._lock:
            if self._active_mode != "MAPPING":
                return {"available": False, "message": "未启动建图链，离线二维地图预览为空"}
            if not self._offline_preview_available:
                return {
                    "available": False,
                    "message": "请先播放指定 bag；离线后端不会读取 bag 消息，只提供模拟预览",
                }
            width, height, resolution = 64, 48, 0.1
            data = []
            for y in range(height):
                for x in range(width):
                    boundary = x in {0, width - 1} or y in {0, height - 1}
                    obstacle = 20 <= x <= 28 and 17 <= y <= 31 or 41 <= x <= 51 and 9 <= y <= 14
                    data.append(100 if boundary or obstacle else 0)
            return {
                "available": True,
                "simulated": True,
                "source": self._playback["bag_id"],
                "message": "离线 bag 建图预览（模拟数据，不是 bag 中的真实地图）",
                "frame_id": "map",
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": {"x": -3.2, "y": -2.4},
                "data": data,
                "downsample_factor": 1,
                "robot_pose": {"available": True, "frame_id": "map", "x": 0.0, "y": 0.0, "yaw": 0.0, "age_sec": 0.0},
            }

    def mapping_pointcloud_status(self) -> dict[str, Any]:
        """Return a bounded, clearly simulated point-cloud preview."""
        with self._lock:
            if self._active_mode != "MAPPING":
                return {"available": False, "message": "未启动建图链，离线点云地图预览为空"}
            if not self._offline_preview_available:
                return {
                    "available": False,
                    "message": "请先播放指定 bag；离线后端不会读取点云消息，只提供模拟预览",
                }
            points = []
            for x in range(-30, 31, 2):
                points.extend(((x * 0.1, -2.0, 0.0), (x * 0.1, 2.0, 0.0)))
            for y in range(-20, 21, 2):
                points.extend(((-3.0, y * 0.1, 0.0), (3.0, y * 0.1, 0.0)))
            for x in range(20, 29, 1):
                for y in range(17, 32, 2):
                    points.append(((x - 32) * 0.1, (y - 24) * 0.1, 0.25))
            for x in range(41, 52, 1):
                for y in range(9, 15, 2):
                    points.append(((x - 32) * 0.1, (y - 24) * 0.1, 0.35))
            return {
                "available": True,
                "simulated": True,
                "source": self._playback["bag_id"],
                "message": "离线 bag 点云预览（模拟数据，不是 bag 中的真实点云）",
                "frame_id": "map",
                "point_count": len(points),
                "voxel_size": 0.1,
                "points": points,
                "robot_pose": {"available": True, "frame_id": "map", "x": 0.0, "y": 0.0, "yaw": 0.0, "age_sec": 0.0},
            }

    def close(self) -> None:
        self.stop_playback()
        self.stop_all()
