"""HTTP-independent operations surface shared by REST and WebSocket clients."""

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import threading
import time
from typing import Any, Callable, Mapping
from uuid import uuid4

import yaml


@dataclass(frozen=True)
class WebConsoleConfig:
    host: str = "127.0.0.1"
    port: int = 8080
    token: str = ""
    runtime_dir: str = "runtime"
    backend: str = "ros"
    can_interface: str = "can0"

    def validate(self) -> None:
        if self.host not in {"127.0.0.1", "::1", "localhost"} and not self.token:
            raise ValueError("a non-loopback Web console listener requires a token")
        if not 1 <= int(self.port) <= 65535:
            raise ValueError("Web console port is outside the valid range")
        if self.backend not in {"ros", "offline"}:
            raise ValueError("Web console backend must be ros or offline")
        if not self.can_interface or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for char in self.can_interface):
            raise ValueError("CAN interface name is invalid")


class WebConsoleService:
    """Validate console requests and delegate only to injected project services."""

    def __init__(
        self,
        config: WebConsoleConfig | None = None,
        *,
        health_provider: Callable[[], Mapping[str, Any]] | None = None,
        readiness_provider: Callable[[], Mapping[str, Any]] | None = None,
        mapping_provider: Callable[[], Mapping[str, Any]] | None = None,
        mapping_pointcloud_provider: Callable[[], Mapping[str, Any]] | None = None,
        chassis_provider: Callable[[], Mapping[str, Any]] | None = None,
        mapping_save_provider: Callable[[str], Mapping[str, Any]] | None = None,
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
        self.mapping_provider = mapping_provider or (lambda: {})
        self.mapping_pointcloud_provider = mapping_pointcloud_provider or (lambda: {})
        self.chassis_provider = chassis_provider or (lambda: {})
        self.mapping_save_provider = mapping_save_provider or (lambda _map_url: {"success": False, "message": "建图保存服务不可用"})
        self.mode_controller = mode_controller
        self.map_registry = map_registry
        self.experiment_manager = experiment_manager
        self.bag_profiles = dict(bag_profiles or {})
        self.localization_controller = localization_controller
        self._backend_options = {"ros": {
            "health_provider": default_health_provider,
            "readiness_provider": default_readiness_provider,
            "mapping_provider": self.mapping_provider,
            "mapping_pointcloud_provider": self.mapping_pointcloud_provider,
            "chassis_provider": self.chassis_provider,
            "mapping_save_provider": self.mapping_save_provider,
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
        self._mapping_session: dict[str, Any] | None = None

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
        self.mapping_provider = option.get("mapping_provider", self.mapping_provider)
        self.mapping_pointcloud_provider = option.get("mapping_pointcloud_provider", self.mapping_pointcloud_provider)
        self.chassis_provider = option.get("chassis_provider", self.chassis_provider)
        self.mapping_save_provider = option.get("mapping_save_provider", self.mapping_save_provider)
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
            if current_status.get("processes") or self._playback_status().get("playing"):
                raise RuntimeError("请先停止当前后端管理的模块和 bag 回放，再切换运行后端")
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

    def mapping_status(self) -> dict[str, Any]:
        return dict(self.mapping_provider())

    def mapping_pointcloud_status(self) -> dict[str, Any]:
        return dict(self.mapping_pointcloud_provider())

    def chassis_status(self) -> dict[str, Any]:
        return dict(self.chassis_provider())

    def _playback_status(self) -> dict[str, Any]:
        if self._backend_mode == "offline" and self.mode_controller is not None and hasattr(self.mode_controller, "playback_status"):
            return dict(self.mode_controller.playback_status())
        if self.experiment_manager is not None and hasattr(self.experiment_manager, "playback_status"):
            return dict(self.experiment_manager.playback_status())
        return {"playing": False}

    @staticmethod
    def _valid_map_name(map_name: str) -> bool:
        return bool(re.fullmatch(r"[A-Za-z0-9_-]+", map_name))

    def _mapping_session_root(self) -> Path:
        return Path(self.config.runtime_dir).expanduser().resolve() / "mapping_sessions"

    def _write_mapping_session(self, session: Mapping[str, Any]) -> None:
        path = Path(str(session["session_file"])).resolve()
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(yaml.safe_dump(dict(session), sort_keys=False), encoding="utf-8")
        os.replace(temporary, path)

    def _discover_mapping_session(self, backend: str | None = None) -> dict[str, Any] | None:
        expected_offline = backend == "offline" if backend is not None else None
        if self._mapping_session is not None and (
            expected_offline is None or bool(self._mapping_session.get("offline", False)) == expected_offline
        ):
            return self._mapping_session
        root = self._mapping_session_root()
        candidates = sorted(root.glob("*/**/session.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)
        for path in candidates:
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    session = yaml.safe_load(stream) or {}
                if not isinstance(session, dict) or not session.get("map_name"):
                    continue
                if expected_offline is not None and bool(session.get("offline", False)) != expected_offline:
                    continue
                session["session_file"] = str(path)
                self._mapping_session = session
                return session
            except (OSError, TypeError, yaml.YAMLError):
                continue
        return None

    def _mapping_asset_status(self, session: Mapping[str, Any]) -> dict[str, Any]:
        root = Path(str(session["root"])).resolve()
        map_name = str(session["map_name"])
        paths = {
            "map_pgm": root / f"{map_name}.pgm",
            "map_yaml": root / f"{map_name}.yaml",
            "pcd": root / "pcd" / "localization_map.pcd",
            "processing_record": root / "pcd" / "localization_map.processing.yaml",
        }
        assets: dict[str, Any] = {}
        for name, path in paths.items():
            try:
                stat = path.stat()
                assets[name] = {"path": str(path), "exists": path.is_file(), "bytes": int(stat.st_size)}
            except OSError:
                assets[name] = {"path": str(path), "exists": False, "bytes": 0}
        record_state = ""
        record_hash = ""
        record = paths["processing_record"]
        if record.is_file():
            try:
                with open(record, "r", encoding="utf-8") as stream:
                    data = yaml.safe_load(stream) or {}
                record_state = str(data.get("state", "")).lower()
                record_hash = str(data.get("pcd_sha256") or data.get("map_hash") or "")
            except (OSError, TypeError, yaml.YAMLError):
                record_state = "invalid"
        return {
            "assets": assets,
            "pgm_ready": assets["map_pgm"]["exists"] and assets["map_yaml"]["exists"],
            "pcd_ready": assets["pcd"]["exists"] and assets["processing_record"]["exists"] and record_state == "ready",
            "processing_state": record_state,
            "pcd_sha256": record_hash,
        }

    @staticmethod
    def _sha256_file(path: Path) -> str:
        digest = hashlib.sha256()
        with open(path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return "sha256:" + digest.hexdigest()

    def _ensure_processing_record_hash(self, session: Mapping[str, Any]) -> str:
        """Finalize a managed ready record before immutable version registration."""
        root = Path(str(session["root"])).resolve()
        pcd_path = root / "pcd" / "localization_map.pcd"
        record_path = root / "pcd" / "localization_map.processing.yaml"
        if not pcd_path.is_file() or not record_path.is_file():
            raise RuntimeError("PCD processing record或定位点云不存在")
        try:
            with open(record_path, "r", encoding="utf-8") as stream:
                record = yaml.safe_load(stream) or {}
        except (OSError, TypeError, yaml.YAMLError) as error:
            raise RuntimeError(f"PCD processing record无法读取：{error}") from error
        if not isinstance(record, dict) or str(record.get("state", "")).lower() != "ready":
            raise RuntimeError("PCD processing record不是 ready 状态")
        actual_hash = self._sha256_file(pcd_path)
        recorded_hash = str(record.get("pcd_sha256") or record.get("map_hash") or "")
        if recorded_hash and recorded_hash != actual_hash:
            raise RuntimeError("PCD processing record哈希与定位点云内容不一致")
        if not recorded_hash:
            record["pcd_sha256"] = actual_hash
            temporary = record_path.with_name(record_path.name + ".tmp")
            temporary.write_text(yaml.safe_dump(record, sort_keys=False), encoding="utf-8")
            os.replace(temporary, record_path)
        return actual_hash

    def mapping_session_status(self) -> dict[str, Any]:
        session = self._discover_mapping_session(backend=self._backend_mode)
        if session is None:
            return {"state": "IDLE", "available": False, "message": "尚未创建建图会话"}
        if session.get("offline"):
            return dict(session)
        status = self._mapping_asset_status(session)
        active_mode = self.mode_status().get("active_mode", "IDLE")
        phase = str(session.get("state", "PREPARED"))
        if phase == "PREPARED" and active_mode == "MAPPING":
            phase = "MAPPING"
        elif phase == "WAITING_ASSETS":
            if status["pgm_ready"] and status["pcd_ready"]:
                phase = "READY_TO_REGISTER"
        return {**dict(session), **status, "state": phase, "active_mode": active_mode, "available": True}

    def prepare_mapping_session(self, map_name: str) -> dict[str, Any]:
        map_name = str(map_name).strip()
        if not self._valid_map_name(map_name):
            raise ValueError("建图名称只能包含字母、数字、下划线和短横线")
        current_mode = self.mode_status().get("active_mode", "IDLE")
        existing = self._discover_mapping_session(backend=self._backend_mode)
        if current_mode == "NAVIGATION":
            raise RuntimeError("请先停止导航模式，再创建建图会话")
        if current_mode == "MAPPING":
            if existing and existing.get("map_name") == map_name:
                return self.mapping_session_status()
            raise RuntimeError("当前建图会话仍在运行，不能创建第二个会话")
        if self._backend_mode == "offline":
            if existing and existing.get("state") == "SIMULATED_RETAINED":
                raise RuntimeError("离线模式最多保留一个模拟地图；请先删除当前地图再创建新的地图")
            if existing and existing.get("state") == "SIMULATED_DISCARDED":
                self._mapping_session = None
                existing = None
            elif existing:
                if existing.get("map_name") == map_name:
                    return self.mapping_session_status()
                raise RuntimeError("离线模式只能同时存在一个模拟建图会话")
            self._mapping_session = {
                "offline": True,
                "available": True,
                "state": "SIMULATED",
                "map_name": map_name,
                "offline_map_slot": {
                    "occupied": False,
                    "simulation_only": True,
                    "exportable": False,
                    "message": "离线模拟最多保留一个地图槽位；真实 PGM/YAML/PCD 需要切换 ROS 2 后端",
                },
                "message": "离线模式只模拟建图会话，不写入 PGM、PCD 或地图版本",
            }
            return dict(self._mapping_session)
        if existing and existing.get("map_name") == map_name and existing.get("state") in {"PREPARED", "MAPPING", "SAVING_MAP", "WAITING_ASSETS"}:
            return self.mapping_session_status()
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        session_id = f"mapping_{timestamp}_{uuid4().hex[:8]}"
        root = self._mapping_session_root() / map_name / session_id
        pcd_dir = root / "pcd"
        pcd_dir.mkdir(parents=True, exist_ok=False)
        session = {
            "schema_version": 1,
            "session_id": session_id,
            "map_name": map_name,
            "root": str(root),
            "pcd_output_dir": str(pcd_dir),
            "map_url": str(root / map_name),
            "state": "PREPARED",
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "version_id": "",
            "session_file": str(root / "session.yaml"),
        }
        self._write_mapping_session(session)
        self._mapping_session = session
        self._audit("mapping_session_prepare", {"session_id": session_id, "map_name": map_name})
        return self.mapping_session_status()

    def _validate_navigation_selection(self, arguments: Mapping[str, Any]) -> None:
        if self.map_registry is None:
            raise RuntimeError("地图版本管理器不可用")
        version_id = str(arguments.get("map_version_id", "")).strip()
        if not version_id:
            raise ValueError("启动导航前必须选择一个地图版本")
        row = self.map_registry._row(version_id)
        if int(row.get("deleted", 0)) or str(row.get("state", "")).upper() != "READY":
            raise ValueError("选择的地图版本不是 READY 状态")
        if not int(row.get("active", 0)):
            raise ValueError("请先在地图版本管理中激活选择的地图版本")
        details = self._map_asset_paths(row)
        for key in ("map", "global_map_pcd", "global_map_processing_record"):
            if str(arguments.get(key, "")).strip() != details[key]:
                raise ValueError("导航参数与所选地图版本资产不一致")

    def _map_asset_paths(self, row: Mapping[str, Any]) -> dict[str, str]:
        manifest_path = Path(str(row["manifest_path"])).resolve()
        with open(manifest_path, "r", encoding="utf-8") as stream:
            manifest = yaml.safe_load(stream) or {}
        assets = manifest.get("assets", {})
        root = manifest_path.parent
        result = {}
        for asset_id, output_key in (("navigation_yaml", "map"), ("localization_pcd", "global_map_pcd"), ("processing_record", "global_map_processing_record")):
            raw = assets.get(asset_id, {})
            path = (root / str(raw.get("path", ""))).resolve()
            try:
                path.relative_to(root)
            except ValueError as error:
                raise ValueError("地图资产路径越出版本目录") from error
            if not path.is_file():
                raise ValueError(f"地图资产不存在: {asset_id}")
            result[output_key] = str(path)
        return result

    def set_mode(self, profile: str, arguments: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if not isinstance(profile, str) or not profile or any(char in profile for char in "\x00\r\n"):
            raise ValueError("profile must be a non-empty configured identifier")
        if self.mode_controller is None:
            raise RuntimeError("system mode controller is unavailable")
        values = dict(arguments or {})
        current_mode = self.mode_status().get("active_mode", "IDLE")
        if profile == "sensor_only" and current_mode == "SENSOR_ONLY":
            raise RuntimeError("传感器模式已经运行，不能重复启动")
        if profile in {"mapping", "navigation"} and current_mode == {"mapping": "MAPPING", "navigation": "NAVIGATION"}[profile]:
            raise RuntimeError(f"{profile} 模式已经运行，不能重复启动")
        if profile == "mapping":
            session = self._discover_mapping_session(backend=self._backend_mode)
            if session is None or bool(session.get("offline", False)) != (self._backend_mode == "offline"):
                raise RuntimeError("建图会话未准备，请先创建建图会话")
            if session.get("pcd_output_dir") and values.get("mapping_output_dir") != session["pcd_output_dir"]:
                raise ValueError("建图输出目录不是当前受管建图会话目录")
        if profile == "navigation":
            if current_mode == "MAPPING":
                raise RuntimeError("请先完成或放弃当前建图会话，再启动导航")
            self._validate_navigation_selection(values)
        result = self.mode_controller.start(profile, values)
        response = dict(result if isinstance(result, Mapping) else {"result": result})
        if profile == "mapping" and self._mapping_session and not self._mapping_session.get("offline"):
            self._mapping_session["state"] = "MAPPING"
            self._write_mapping_session(self._mapping_session)
        self._audit("system_mode_start", {"profile": profile, "arguments": values, "result": response})
        return response

    def stop_mode(self, mode: str | None = None) -> dict[str, Any]:
        if self.mode_controller is None:
            raise RuntimeError("system mode controller is unavailable")
        result = self.mode_controller.stop_all() if mode is None else self.mode_controller.stop_mode(mode)
        response = {"stopped": result}
        self._audit("system_mode_stop", {"mode": mode, "result": response})
        return response

    def finish_mapping(self, action: str, map_name: str | None = None) -> dict[str, Any]:
        if action not in {"retain", "delete"}:
            raise ValueError("建图结束操作只能是 retain 或 delete")
        session = self._discover_mapping_session(backend=self._backend_mode)
        if session is None:
            raise RuntimeError("当前没有建图会话")
        if session.get("offline"):
            if self.mode_status().get("active_mode") == "MAPPING":
                self.stop_mode()
            if session.get("state") == "SIMULATED_RETAINED" and action == "retain":
                raise RuntimeError("离线模拟地图槽位已经占用；请先删除当前地图")
            session["state"] = "SIMULATED_RETAINED" if action == "retain" else "SIMULATED_DISCARDED"
            slot = dict(session.get("offline_map_slot") or {})
            slot.update({"occupied": action == "retain", "simulation_only": True, "exportable": False})
            session["offline_map_slot"] = slot
            session["message"] = (
                "离线模拟地图已保留（仅一个模拟槽位），未写入真实 PGM、YAML、PCD 或地图版本；"
                "请切换 ROS 2 后端并用历史 bag 重建后导出实车资产"
                if action == "retain"
                else "离线模拟地图已删除，未写入任何真实地图文件"
            )
            return dict(session)
        if action == "delete":
            if self.mode_status().get("active_mode") == "MAPPING":
                self.stop_mode()
            failed_version_id = str(session.get("failed_version_id", "")).strip()
            if failed_version_id and self.map_registry is not None:
                self.map_registry.soft_delete(failed_version_id)
                self.map_registry.purge(failed_version_id)
            root = Path(str(session["root"])).resolve()
            staging = self._mapping_session_root().resolve()
            try:
                root.relative_to(staging)
            except ValueError as error:
                raise RuntimeError("建图会话目录不在受管临时目录内") from error
            shutil.rmtree(root, ignore_errors=False)
            session["state"] = "DISCARDED"
            self._mapping_session = None
            self._audit("mapping_session_discard", {"session_id": session["session_id"]})
            return {"state": "DISCARDED", "session_id": session["session_id"], "message": "建图临时文件已删除，未登记地图版本"}

        if self.mode_status().get("active_mode") != "MAPPING":
            raise RuntimeError("只有正在运行的建图模式可以完成保存；失败会话请使用删除临时文件")

        requested_name = str(map_name or "").strip()
        if requested_name:
            if not self._valid_map_name(requested_name):
                raise ValueError("建图名称只能包含字母、数字、下划线和短横线")
            if requested_name != str(session["map_name"]):
                root = Path(str(session["root"])).resolve()
                old_assets = (root / f"{session['map_name']}.pgm", root / f"{session['map_name']}.yaml")
                if any(path.exists() for path in old_assets):
                    raise RuntimeError("地图文件已经写入，不能在保存阶段修改建图名称")
                session["map_name"] = requested_name
                session["map_url"] = str(root / requested_name)
                self._write_mapping_session(session)

        session["state"] = "SAVING_MAP"
        self._write_mapping_session(session)
        try:
            save_result = dict(self.mapping_save_provider(str(session["map_url"])))
            if not save_result.get("success", False):
                raise RuntimeError(save_result.get("message", "PGM/YAML 保存服务失败"))
            self.stop_mode()
            session["state"] = "WAITING_ASSETS"
            self._write_mapping_session(session)
            deadline = time.monotonic() + 90.0
            while time.monotonic() < deadline:
                status = self._mapping_asset_status(session)
                if status["pgm_ready"] and status["pcd_ready"]:
                    break
                time.sleep(0.25)
            status = self._mapping_asset_status(session)
            if not status["pgm_ready"] or not status["pcd_ready"]:
                raise RuntimeError("地图保存未完成：需要同时存在 PGM/YAML 和 ready PCD processing record")
            self._ensure_processing_record_hash(session)
            status = self._mapping_asset_status(session)
            if self.map_registry is None:
                raise RuntimeError("地图版本管理器不可用，不能登记保留的建图结果")
            result = self.map_registry.import_legacy(
                map_id=str(session["map_name"]),
                map_yaml=str(Path(session["root"]) / f"{session['map_name']}.yaml"),
                localization_pcd=str(Path(session["root"]) / "pcd" / "localization_map.pcd"),
                processing_record=str(Path(session["root"]) / "pcd" / "localization_map.processing.yaml"),
            )
            if not result.valid:
                if result.map_version_id:
                    session["failed_version_id"] = result.map_version_id
                raise RuntimeError("地图版本登记失败：" + "；".join(result.errors))
            session["state"] = "REGISTERED"
            session["version_id"] = result.map_version_id
            self._write_mapping_session(session)
            response = {
                "state": "REGISTERED",
                "session_id": session["session_id"],
                "map_name": session["map_name"],
                "map_version_id": result.map_version_id,
                "assets": status["assets"],
                "message": "PGM、YAML、PCD 已确认并登记为地图版本；请在地图管理中激活后启动导航",
            }
            self._audit("mapping_session_registered", response)
            return response
        except Exception as error:
            session["state"] = "ERROR"
            session["message"] = str(error)
            self._write_mapping_session(session)
            raise

    def maps(self, **filters: Any) -> list[dict[str, Any]]:
        if self.map_registry is None:
            return []
        result = []
        for row in self.map_registry.list_versions(**filters):
            item = dict(row)
            try:
                item["assets"] = self._map_asset_paths(row)
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
                item["assets"] = {}
            result.append(item)
        return result

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

    def bags(self) -> dict[str, Any]:
        if self.experiment_manager is None or not hasattr(self.experiment_manager, "list_bags"):
            return {"bags": [], "playback": {"playing": False}}
        return {
            "bags": [dict(item) for item in self.experiment_manager.list_bags()],
            "playback": self._playback_status(),
        }

    def bag_action(self, action: str, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.experiment_manager is None:
            raise RuntimeError("experiment manager is unavailable")
        values = dict(values or {})
        if self._backend_mode == "offline":
            if self.mode_controller is None or not hasattr(self.mode_controller, "start_playback"):
                raise RuntimeError("离线模拟回放不可用")
            if action == "play":
                result = self.mode_controller.start_playback(str(values.get("bag_id", "")), rate=values.get("rate", 1.0))
            elif action == "stop":
                result = self.mode_controller.stop_playback()
            else:
                raise ValueError("unsupported bag action")
            response = dict(result)
            self._audit(f"offline_bag_{action}", {"request": values, "result": response})
            return response
        if action == "play":
            if not hasattr(self.experiment_manager, "start_playback"):
                raise RuntimeError("bag playback is unavailable")
            active_mode = str(self.mode_status().get("active_mode", "IDLE")).upper()
            requested_profile = str(values.get("playback_profile", "")).strip()
            if active_mode == "MAPPING":
                if requested_profile and requested_profile != "mapping_inputs":
                    raise ValueError("建图模式只能回放 mapping_inputs，避免重复发布 FAST-LIVO2 输出")
                playback_profile = "mapping_inputs"
            elif active_mode == "LOCALIZATION_DEBUG":
                if requested_profile and requested_profile != "localization_inputs":
                    raise ValueError("定位调试模式只能回放 localization_inputs")
                playback_profile = "localization_inputs"
            elif active_mode == "NAVIGATION":
                raise RuntimeError("导航模式运行期间禁止回放 bag，避免覆盖导航、TF 或定位输出")
            else:
                playback_profile = requested_profile or "all"
            result = self.experiment_manager.start_playback(
                str(values.get("bag_id", "")),
                rate=values.get("rate", 1.0),
                playback_profile=playback_profile,
            )
        elif action == "stop":
            if hasattr(self.experiment_manager, "stop_playback"):
                self.experiment_manager.stop_playback()
            result = {"state": "STOPPED"}
        else:
            raise ValueError("unsupported bag action")
        response = dict(result)
        self._audit(f"bag_{action}", {"request": values, "result": response})
        return response

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
