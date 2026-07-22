"""HTTP-independent operations surface shared by REST and WebSocket clients."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class WebConsoleConfig:
    host: str = "127.0.0.1"
    port: int = 8080
    token: str = ""
    runtime_dir: str = "runtime"
    backend: str = "ros"

    def validate(self) -> None:
        if self.host not in {"127.0.0.1", "::1", "localhost"} and not self.token:
            raise ValueError("a non-loopback Web console listener requires a token")
        if not 1 <= int(self.port) <= 65535:
            raise ValueError("Web console port is outside the valid range")
        if self.backend not in {"ros", "offline"}:
            raise ValueError("Web console backend must be ros or offline")


class WebConsoleService:
    """Validate console requests and delegate only to injected project services."""

    def __init__(
        self,
        config: WebConsoleConfig | None = None,
        *,
        health_provider: Callable[[], Mapping[str, Any]] | None = None,
        readiness_provider: Callable[[], Mapping[str, Any]] | None = None,
        mode_controller: Any = None,
        map_registry: Any = None,
        experiment_manager: Any = None,
        bag_profiles: Mapping[str, Any] | None = None,
        localization_controller: Any = None,
        backends: Mapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        self.config = config or WebConsoleConfig()
        self.config.validate()
        default_health_provider = health_provider or (lambda: {"overall_state": "UNKNOWN", "components": []})
        default_readiness_provider = readiness_provider or (lambda: {"ready": False, "blocker_codes": ["HEALTH_UNAVAILABLE"]})
        self.health_provider = default_health_provider
        self.readiness_provider = default_readiness_provider
        self.mode_controller = mode_controller
        self.map_registry = map_registry
        self.experiment_manager = experiment_manager
        self.bag_profiles = dict(bag_profiles or {})
        self.localization_controller = localization_controller
        self._backend_options = {"ros": {
            "health_provider": default_health_provider,
            "readiness_provider": default_readiness_provider,
            "mode_controller": mode_controller,
            "localization_controller": localization_controller,
        }}
        for backend_name, backend in (backends or {}).items():
            self._backend_options[str(backend_name)] = dict(backend)
        if self.config.backend not in self._backend_options:
            raise ValueError(f"backend is not configured: {self.config.backend}")
        self._backend_mode = self.config.backend
        self._apply_backend(self._backend_mode)
        self._lock = threading.RLock()
        self._subscribers: list[Callable[[Mapping[str, Any]], None]] = []
        self._audit_path = Path(self.config.runtime_dir).expanduser() / "logs" / "web_console_audit.jsonl"

    def subscribe(self, callback: Callable[[Mapping[str, Any]], None]) -> None:
        with self._lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[Mapping[str, Any]], None]) -> None:
        with self._lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    def publish(self, event: Mapping[str, Any]) -> None:
        with self._lock:
            subscribers = list(self._subscribers)
        for callback in subscribers:
            callback(dict(event))

    def _audit(self, action: str, data: Mapping[str, Any] | None = None) -> None:
        record = {"action": action, "data": dict(data or {})}
        self._audit_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._audit_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        self.publish({"type": "audit", **record})

    def overview(self) -> dict[str, Any]:
        return {
            "health": dict(self.health_provider()),
            "task_readiness": dict(self.readiness_provider()),
            "localization": self.localization_controller.localization() if self.localization_controller and hasattr(self.localization_controller, "localization") else {},
            "maps": self.maps(),
            "experiments": self.experiments(),
            "mode": self.mode_status(),
            "runtime": self.runtime_status(),
        }

    def runtime_status(self) -> dict[str, Any]:
        with self._lock:
            backend = self._backend_mode
            available = sorted(self._backend_options)
        return {
            "backend": backend,
            "offline": backend == "offline",
            "available_backends": available,
            "description": "离线模拟后端：不连接传感器、车辆或真实 ROS 控制链" if backend == "offline" else "ROS 2 后端：调用真实项目接口和白名单 profile",
        }

    def _apply_backend(self, backend: str) -> None:
        option = self._backend_options[backend]
        self.health_provider = option.get("health_provider", self.health_provider)
        self.readiness_provider = option.get("readiness_provider", self.readiness_provider)
        self.mode_controller = option.get("mode_controller")
        self.localization_controller = option.get("localization_controller")

    def publish_backend(self, backend: str, event: Mapping[str, Any]) -> None:
        with self._lock:
            if backend != self._backend_mode:
                return
        self.publish(event)

    def set_backend(self, backend: str) -> dict[str, Any]:
        backend = str(backend).strip().lower()
        with self._lock:
            if backend not in self._backend_options:
                raise ValueError(f"backend is not configured: {backend}")
            if backend == self._backend_mode:
                return self.runtime_status()
            current_status = self.mode_status()
            if current_status.get("processes"):
                raise RuntimeError("请先停止当前后端管理的模块，再切换运行后端")
            self._backend_mode = backend
            self._apply_backend(backend)
        response = self.runtime_status()
        self._audit("runtime_backend_switch", response)
        self.publish({"type": "runtime", "runtime": response})
        return response

    def mode_status(self) -> dict[str, Any]:
        if self.mode_controller is None:
            return {"active_mode": "IDLE", "processes": []}
        status = self.mode_controller.status()
        return dict(status if isinstance(status, Mapping) else {"processes": status})

    def set_mode(self, profile: str, arguments: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if not isinstance(profile, str) or not profile or any(char in profile for char in "\x00\r\n"):
            raise ValueError("profile must be a non-empty configured identifier")
        if self.mode_controller is None:
            raise RuntimeError("system mode controller is unavailable")
        result = self.mode_controller.start(profile, dict(arguments or {}))
        response = dict(result if isinstance(result, Mapping) else {"result": result})
        self._audit("system_mode_start", {"profile": profile, "arguments": dict(arguments or {}), "result": response})
        return response

    def stop_mode(self, mode: str | None = None) -> dict[str, Any]:
        if self.mode_controller is None:
            raise RuntimeError("system mode controller is unavailable")
        result = self.mode_controller.stop_all() if mode is None else self.mode_controller.stop_mode(mode)
        response = {"stopped": result}
        self._audit("system_mode_stop", {"mode": mode, "result": response})
        return response

    def maps(self, **filters: Any) -> list[dict[str, Any]]:
        if self.map_registry is None:
            return []
        return [dict(row) for row in self.map_registry.list_versions(**filters)]

    def validate_map(self, version_id: str) -> dict[str, Any]:
        if self.map_registry is None:
            raise RuntimeError("map registry is unavailable")
        row = self.map_registry._row(version_id)
        result = self.map_registry.validate_manifest(row["manifest_path"])
        return {"valid": result.valid, "map_id": result.map_id, "map_version_id": result.map_version_id, "errors": list(result.errors), "warnings": list(result.warnings), "asset_hashes": dict(result.asset_hashes), "map_hash": result.map_hash}

    def activate_map(self, version_id: str) -> dict[str, Any]:
        if self.map_registry is None:
            raise RuntimeError("map registry is unavailable")
        result = self.map_registry.activate(version_id)
        response = self.validate_map(version_id)
        response["activated"] = result.valid
        self._audit("map_activate", {"version_id": version_id, "result": response})
        return response

    def map_action(self, version_id: str, action: str) -> dict[str, Any]:
        if self.map_registry is None:
            raise RuntimeError("map registry is unavailable")
        if action == "pin":
            self.map_registry.set_pinned(version_id, True)
        elif action == "unpin":
            self.map_registry.set_pinned(version_id, False)
        elif action == "archive":
            self.map_registry.archive(version_id)
        elif action == "delete":
            response = {"deleted_path": str(self.map_registry.soft_delete(version_id))}
            self._audit("map_delete", {"version_id": version_id, "result": response})
            return response
        elif action == "purge":
            self.map_registry.purge(version_id)
        else:
            raise ValueError(f"unsupported map action: {action}")
        response = {"version_id": version_id, "action": action}
        self._audit(f"map_{action}", response)
        return response

    def import_map(self, values: Mapping[str, Any]) -> dict[str, Any]:
        if self.map_registry is None:
            raise RuntimeError("map registry is unavailable")
        allowed = {"map_id", "map_yaml", "pcd", "processing_record", "platform_profile"}
        unknown = set(values) - allowed
        required = {"map_id", "map_yaml", "pcd", "processing_record"}
        if unknown or not required.issubset(values):
            raise ValueError(f"map import requires map_id, map_yaml, pcd and processing_record; unknown={sorted(unknown)}")
        result = self.map_registry.import_legacy(**dict(values))
        response = {"valid": result.valid, "map_id": result.map_id, "map_version_id": result.map_version_id, "errors": list(result.errors), "warnings": list(result.warnings)}
        self._audit("map_import", response)
        return response

    def experiments(self, **filters: Any) -> list[dict[str, Any]]:
        if self.experiment_manager is None:
            return []
        return [dict(item) for item in self.experiment_manager.list(**filters)]

    def create_experiment(self, values: Mapping[str, Any]) -> dict[str, Any]:
        if self.experiment_manager is None:
            raise RuntimeError("experiment manager is unavailable")
        allowed = {"title", "objective", "hypothesis", "tags", "operator_note", "platform_profile", "active_map", "launch_profile", "launch_arguments"}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unknown experiment fields: {sorted(unknown)}")
        experiment_id = self.experiment_manager.create(**dict(values))
        response = {"experiment_id": experiment_id, "state": "CREATED"}
        self._audit("experiment_create", response)
        return response

    def experiment_action(self, experiment_id: str, action: str, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.experiment_manager is None:
            raise RuntimeError("experiment manager is unavailable")
        if self._backend_mode == "offline" and action in {"start_bag", "stop_bag"}:
            raise RuntimeError("离线测试模式不启动或停止 rosbag；请切换到 ROS 2 后端")
        values = dict(values or {})
        if action == "start":
            result = self.experiment_manager.start(experiment_id, values.get("health"))
        elif action == "event":
            self.experiment_manager.add_event(experiment_id, str(values.get("type", "operator_event")), values.get("data", {}))
            result = {"state": "RUNNING"}
        elif action == "finalize":
            result = self.experiment_manager.finalize(experiment_id, values.get("health"), str(values.get("result_status", "COMPLETED")))
        elif action == "invalid":
            self.experiment_manager.mark_invalid(experiment_id, str(values.get("reason", "operator marked invalid")))
            result = {"state": "INVALID"}
        elif action == "start_bag":
            profile_id = str(values.get("profile", "minimal"))
            profile = self.bag_profiles.get(profile_id)
            if not isinstance(profile, Mapping):
                raise ValueError(f"unknown bag profile: {profile_id}")
            bag_path = self.experiment_manager.start_bag(experiment_id, profile_id, profile)
            result = {"state": "RECORDING", "profile": profile_id, "path": str(bag_path)}
        elif action == "stop_bag":
            self.experiment_manager.stop_bag(experiment_id)
            result = {"state": "RUNNING", "bag": self.experiment_manager.bag_status() if hasattr(self.experiment_manager, "bag_status") else {}}
        else:
            raise ValueError(f"unsupported experiment action: {action}")
        response = dict(result if isinstance(result, Mapping) else {"result": result})
        self._audit(f"experiment_{action}", {"experiment_id": experiment_id, "result": response})
        return response

    def localization_mode(self, mode: str) -> dict[str, Any]:
        if mode not in {"MANUAL_ONLY", "AUTO_ON_START", "AUTO_RECOVERY"}:
            raise ValueError("unsupported localization mode")
        if self.localization_controller is None:
            raise RuntimeError("localization mode controller is unavailable")
        result = self.localization_controller.set_mode(mode)
        response = dict(result if isinstance(result, Mapping) else {"mode": mode})
        self._audit("localization_mode", {"mode": mode})
        return response

    def relocalize(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.localization_controller is None or not hasattr(self.localization_controller, "relocalize"):
            raise RuntimeError("relocalization Action is unavailable")
        response = dict(self.localization_controller.relocalize(dict(values or {})))
        self._audit("localization_relocalize", {"request": dict(values or {}), "result": response})
        return response

    def logs(self, component: str = "system_manager") -> list[dict[str, Any]]:
        roots = {"system_manager": Path(self.config.runtime_dir) / "logs" / "system_manager", "web_console": Path(self.config.runtime_dir) / "logs", "experiments": Path(self.config.runtime_dir) / "experiments"}
        root = roots.get(component)
        if root is None:
            raise ValueError("logs are limited to managed component roots")
        return [{"path": str(path), "size": path.stat().st_size} for path in sorted(root.rglob("*.log")) if path.is_file()]
