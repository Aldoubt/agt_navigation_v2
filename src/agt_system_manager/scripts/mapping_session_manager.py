#!/usr/bin/env python3

from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace
from typing import Any
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_prefix
from agt_interfaces.action import ChangeSystemMode, ManageMappingSession
from agt_interfaces.srv import ManageBagSession, ManageMapVersion
from nav2_msgs.srv import SaveMap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from agt_system_manager.mapping_session import (
    MappingSessionError,
    MappingSessionRepository,
    mapping_session_timeout,
)


_ERROR_CODES = {
    "invalid_identifier": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "owned_argument": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "unknown_argument": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "invalid_argument": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "invalid_timeout": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "invalid_operation": ManageMappingSession.Result.ERROR_INVALID_REQUEST,
    "server_unavailable": ManageMappingSession.Result.ERROR_SERVER_UNAVAILABLE,
    "mode_change_failed": ManageMappingSession.Result.ERROR_STOP_FAILED,
    "mapping_start_failed": ManageMappingSession.Result.ERROR_START_FAILED,
    "bag_operation_failed": ManageMappingSession.Result.ERROR_START_FAILED,
    "session_active": ManageMappingSession.Result.ERROR_INVALID_STATE,
    "invalid_state": ManageMappingSession.Result.ERROR_INVALID_STATE,
    "session_not_found": ManageMappingSession.Result.ERROR_NOT_FOUND,
    "grid_save_failed": ManageMappingSession.Result.ERROR_GRID_SAVE_FAILED,
    "mapping_stop_failed": ManageMappingSession.Result.ERROR_STOP_FAILED,
    "asset_timeout": ManageMappingSession.Result.ERROR_ASSET_TIMEOUT,
    "offline_candidate_failed": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "candidate_quality_failed": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "candidate_incomplete": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "candidate_grid_invalid": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "pcd_not_ready": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "processing_record_invalid": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "processing_record_mismatch": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "pcd_hash_mismatch": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "map_registration_failed": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "map_activation_failed": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
    "map_commit_failed": ManageMappingSession.Result.ERROR_COMMIT_FAILED,
}


class RemoteMapRegistry:
    """MapRegistry-shaped adapter backed only by the map manager service."""

    def __init__(self, owner: "MappingSessionManager") -> None:
        self._owner = owner

    @staticmethod
    def _result(response):
        version = response.version
        return SimpleNamespace(
            valid=bool(version.valid),
            errors=tuple(version.validation_errors),
            map_version_id=str(version.map_version_id),
            navigation_yaml=str(version.navigation_yaml),
            tasks_directory=str(version.tasks_directory),
        )

    def import_legacy(
        self,
        *,
        map_id,
        map_yaml,
        localization_pcd,
        processing_record,
        platform_profile="",
        parent_version_id=None,
    ):
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_IMPORT_CANDIDATE
        request.map_id = str(map_id)
        request.candidate_map_yaml = str(map_yaml)
        request.localization_pcd = str(localization_pcd)
        request.processing_record = str(processing_record)
        request.platform_profile = str(platform_profile)
        request.parent_map_version_id = str(parent_version_id or "")
        response = self._owner._call_service(
            self._owner._map_client, request, 30.0, "map candidate import"
        )
        if not response.success:
            raise MappingSessionError("map_registration_failed", response.message)
        return self._result(response)

    def activate(self, version_id):
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_ACTIVATE
        request.map_version_id = str(version_id)
        response = self._owner._call_service(
            self._owner._map_client, request, 30.0, "map activation"
        )
        if not response.success:
            raise MappingSessionError("map_activation_failed", response.message)
        return self._result(response)

    def discard_failed(self, version_id: str) -> None:
        request = ManageMapVersion.Request()
        request.operation = ManageMapVersion.Request.OP_SOFT_DELETE
        request.map_version_id = str(version_id)
        request.confirm_destructive = True
        response = self._owner._call_service(
            self._owner._map_client, request, 30.0, "failed map cleanup"
        )
        if not response.success and response.error_code != response.ERROR_NOT_FOUND:
            raise MappingSessionError("map_commit_failed", response.message)


class MappingSessionManager(Node):
    def __init__(self) -> None:
        super().__init__("agt_mapping_session_manager")
        runtime_dir = Path(
            str(self.declare_parameter("runtime_dir", "runtime").value)
        ).expanduser().resolve()
        self._runtime_dir = runtime_dir
        self._platform_profile = Path(
            str(self.declare_parameter("platform_profile", "").value)
        ).expanduser().resolve()
        self._static_grid_padding = float(
            self.declare_parameter("static_grid_padding", 2.0).value
        )
        self._static_evidence_range = float(
            self.declare_parameter("static_evidence_range", 40.0).value
        )
        self._raytrace_interval = float(
            self.declare_parameter("raytrace_interval", 1.0).value
        )
        if not self._platform_profile.is_file():
            raise RuntimeError(
                f"mapping-session platform profile does not exist: {self._platform_profile}"
            )
        if (
            self._static_grid_padding <= 0.0
            or self._static_evidence_range <= 0.0
            or self._raytrace_interval <= 0.0
        ):
            raise RuntimeError("managed static-map range, padding, and interval must be positive")
        self._repository = MappingSessionRepository(runtime_dir)
        self._operation_lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self._map_client = self.create_client(
            ManageMapVersion, "/agt/maps/manage", callback_group=group
        )
        self._bag_client = self.create_client(
            ManageBagSession, "/agt/data/bags/manage", callback_group=group
        )
        self._registry = RemoteMapRegistry(self)
        self._mode_action = ActionClient(
            self, ChangeSystemMode, "/agt/system/change_mode", callback_group=group
        )
        self._mapping_save = self.create_client(
            SaveMap, "/agt_mapping_map_saver/save_map", callback_group=group
        )
        self._server = ActionServer(
            self,
            ManageMappingSession,
            "/agt/mapping/manage_session",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _goal: CancelResponse.REJECT,
            callback_group=group,
        )

    @staticmethod
    def _goal(goal: ManageMappingSession.Goal):
        if goal.operation not in {
            ManageMappingSession.Goal.OP_STATUS,
            ManageMappingSession.Goal.OP_START,
            ManageMappingSession.Goal.OP_FINALIZE_CAPTURE,
            ManageMappingSession.Goal.OP_COMMIT,
            ManageMappingSession.Goal.OP_DISCARD,
        }:
            return GoalResponse.REJECT
        if len(goal.argument_keys) != len(goal.argument_values):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _timeout(request: ManageMappingSession.Goal) -> float:
        return mapping_session_timeout(request.timeout_s)

    @staticmethod
    def _wait(future: Any, timeout_s: float, description: str) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(max(0.1, timeout_s)):
            raise MappingSessionError("server_unavailable", f"{description} timed out")
        error = future.exception()
        if error is not None:
            raise error
        return future.result()

    def _call_service(self, client, request, timeout_s: float, description: str):
        timeout_s = min(max(float(timeout_s), 0.1), 300.0)
        if not client.wait_for_service(timeout_sec=min(timeout_s, 5.0)):
            raise MappingSessionError(
                "server_unavailable", f"{description} service is unavailable"
            )
        return self._wait(client.call_async(request), timeout_s, description)

    def _bag_operation(
        self,
        operation: int,
        *,
        experiment_id: str = "",
        profile_id: str = "",
        title: str = "",
        platform_profile: str = "",
        timeout_s: float,
    ):
        request = ManageBagSession.Request()
        request.operation = operation
        request.experiment_id = experiment_id
        request.profile_id = profile_id
        request.experiment_title = title
        request.platform_profile = platform_profile
        response = self._call_service(
            self._bag_client, request, timeout_s, "experiment/bag operation"
        )
        if not response.success:
            raise MappingSessionError("bag_operation_failed", response.message)
        return response

    def _start_mapping_capture(
        self, session: dict[str, Any], timeout_s: float
    ) -> dict[str, Any]:
        platform_profile = str(
            (session.get("start_arguments") or {}).get("platform_profile", "")
        )
        created = self._bag_operation(
            ManageBagSession.Request.OP_CREATE_EXPERIMENT,
            title=f"mapping {session['session_id']}",
            platform_profile=platform_profile,
            timeout_s=timeout_s,
        )
        experiment_id = str(created.session.experiment_id)
        if not experiment_id:
            raise MappingSessionError(
                "bag_operation_failed", "experiment manager returned no experiment identity"
            )
        try:
            recording = self._bag_operation(
                ManageBagSession.Request.OP_START_RECORDING,
                experiment_id=experiment_id,
                profile_id="mapping",
                timeout_s=timeout_s,
            )
            relative_uri = Path(str(recording.session.relative_uri))
            if relative_uri.is_absolute() or ".." in relative_uri.parts:
                raise MappingSessionError(
                    "bag_operation_failed", "experiment manager returned an unsafe bag URI"
                )
            bag_directory = (self._runtime_dir / relative_uri).resolve()
            bag_directory.relative_to(self._runtime_dir)
        except Exception:
            try:
                self._bag_operation(
                    ManageBagSession.Request.OP_INTERRUPT_EXPERIMENT,
                    experiment_id=experiment_id,
                    timeout_s=min(timeout_s, 10.0),
                )
            except Exception:
                pass
            raise
        return self._repository.update(
            session,
            str(session["state"]),
            experiment_id=experiment_id,
            bag_directory=str(bag_directory),
            bag_profile="mapping",
        )

    def _stop_mapping_capture(
        self, session: dict[str, Any], timeout_s: float, *, interrupt: bool = False
    ) -> None:
        errors = []
        try:
            self._stop_mapping(timeout_s)
        except Exception as exc:
            errors.append(str(exc))
        experiment_id = str(session.get("experiment_id", ""))
        if experiment_id:
            try:
                self._bag_operation(
                    ManageBagSession.Request.OP_STOP_RECORDING,
                    experiment_id=experiment_id,
                    timeout_s=timeout_s,
                )
            except Exception as exc:
                errors.append(str(exc))
            try:
                self._bag_operation(
                    (
                        ManageBagSession.Request.OP_INTERRUPT_EXPERIMENT
                        if interrupt
                        else ManageBagSession.Request.OP_COMPLETE_EXPERIMENT
                    ),
                    experiment_id=experiment_id,
                    timeout_s=timeout_s,
                )
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise MappingSessionError("bag_operation_failed", "; ".join(errors))

    def _change_mode(
        self,
        mode: int,
        *,
        profile: str = "",
        arguments: dict[str, str] | None = None,
        timeout_s: float,
    ) -> ChangeSystemMode.Result:
        if not self._mode_action.wait_for_server(timeout_sec=min(timeout_s, 5.0)):
            raise MappingSessionError(
                "server_unavailable", "/agt/system/change_mode Action is unavailable"
            )
        goal = ChangeSystemMode.Goal()
        goal.mode = mode
        goal.profile = profile
        values = arguments or {}
        goal.argument_keys = list(values)
        goal.argument_values = [values[key] for key in values]
        goal.wait_for_health = False
        goal.startup_timeout_s = min(timeout_s, 300.0)
        handle = self._wait(
            self._mode_action.send_goal_async(goal), min(timeout_s, 10.0), "mode request"
        )
        if not handle.accepted:
            raise MappingSessionError("server_unavailable", "system mode request was rejected")
        wrapped = self._wait(handle.get_result_async(), timeout_s, "mode change")
        if not wrapped.result.success:
            raise MappingSessionError("mode_change_failed", wrapped.result.message)
        return wrapped.result

    def _save_grid(self, map_url: Path, timeout_s: float) -> None:
        if not self._mapping_save.wait_for_service(timeout_sec=min(timeout_s, 5.0)):
            raise RuntimeError("/agt_mapping_map_saver/save_map is unavailable")
        request = SaveMap.Request()
        request.map_topic = "/agt/map/mapping_occupancy"
        request.map_url = str(map_url)
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.196
        request.occupied_thresh = 0.65
        response = self._wait(
            self._mapping_save.call_async(request), timeout_s, "mapping grid save"
        )
        if not response.result:
            raise RuntimeError("map_saver returned failure")

    def _stop_mapping(self, timeout_s: float) -> None:
        self._change_mode(ChangeSystemMode.Goal.MODE_IDLE, timeout_s=timeout_s)

    def _build_static_candidate(
        self,
        session: dict[str, Any],
        paths: Any,
        baseline_yaml: Path,
        timeout_s: float,
    ) -> dict[str, Any]:
        platform_profile = Path(
            str((session.get("start_arguments") or {}).get("platform_profile", ""))
        ).expanduser().resolve()
        if not platform_profile.is_file():
            raise MappingSessionError(
                "offline_candidate_failed",
                f"platform profile is unavailable: {platform_profile}",
            )
        output_directory = paths.root / (
            "offline_static_candidate_"
            + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            + "_"
            + uuid4().hex[:8]
        )
        output_directory.mkdir(parents=False, exist_ok=False)
        executable = (
            Path(get_package_prefix("agt_map_processing"))
            / "lib"
            / "agt_map_processing"
            / "generate_traversability_variants.py"
        )
        if not executable.is_file():
            raise MappingSessionError(
                "offline_candidate_failed", f"static-map generator is unavailable: {executable}"
            )
        command = [
            str(executable),
            "--bag",
            str(paths.bag_directory),
            "--baseline-yaml",
            str(baseline_yaml),
            "--output-dir",
            str(output_directory),
            "--platform-profile",
            str(platform_profile),
            "--rebuild-raytraced-baseline",
            "--raytrace-interval",
            str(self._raytrace_interval),
            "--raytrace-max-range",
            str(self._static_evidence_range),
            "--maximum-evidence-range",
            str(self._static_evidence_range),
            "--grid-padding",
            str(self._static_grid_padding),
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            output, _ = process.communicate(timeout=max(0.1, timeout_s))
        except subprocess.TimeoutExpired as error:
            process.terminate()
            output, _ = process.communicate(timeout=15.0)
            raise MappingSessionError(
                "offline_candidate_failed",
                f"static-map generation exceeded the bounded timeout: {output[-2000:]}",
            ) from error
        if process.returncode != 0:
            raise MappingSessionError(
                "offline_candidate_failed",
                f"static-map generator exited with {process.returncode}: {output[-4000:]}",
            )
        report_path = output_directory / "comparison_report.json"
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MappingSessionError(
                "offline_candidate_failed", f"static-map report is unreadable: {error}"
            ) from error
        parameters = report.get("parameters") or {}
        expected_paths = {
            "bag": paths.bag_directory,
            "baseline_yaml": baseline_yaml,
            "output_dir": output_directory,
            "platform_profile": platform_profile,
        }
        for key, expected in expected_paths.items():
            try:
                actual = Path(str(parameters[key])).expanduser().resolve()
            except (KeyError, OSError, ValueError) as error:
                raise MappingSessionError(
                    "candidate_quality_failed", f"static-map report has no valid {key}"
                ) from error
            if actual != expected.resolve():
                raise MappingSessionError(
                    "candidate_quality_failed", f"static-map report {key} does not match the request"
                )
        return {
            "map_yaml": output_directory / "ground_temporal.yaml",
            "map_image": output_directory / "ground_temporal.pgm",
            "report_path": report_path,
            "report": report,
        }

    @staticmethod
    def _feedback(goal_handle: Any, state: str, progress: float, message: str) -> None:
        goal_handle.publish_feedback(
            ManageMappingSession.Feedback(
                state=state,
                progress=float(progress),
                message=message,
            )
        )

    def _fill_result(
        self,
        result: ManageMappingSession.Result,
        session: dict[str, Any] | None,
    ) -> None:
        if not session:
            return
        result.state = str(session.get("state", ""))
        result.session_id = str(session.get("session_id", ""))
        result.map_id = str(session.get("map_id", ""))
        result.map_version_id = str(session.get("map_version_id", ""))
        result.session_file = str(session.get("session_file", ""))
        try:
            paths = self._repository.paths(session)
        except (KeyError, OSError, MappingSessionError):
            return
        result.candidate_map_yaml = str(paths.map_yaml)
        result.candidate_map_image = str(paths.map_image)
        result.localization_pcd = str(paths.pcd)
        result.processing_record = str(paths.processing_record)
        result.bag_directory = str(paths.bag_directory)
        result.registered_map_yaml = str(session.get("registered_map_yaml", ""))
        result.tasks_directory = str(session.get("tasks_directory", ""))

    def _execute_operation(self, goal_handle: Any) -> dict[str, Any]:
        request = goal_handle.request
        timeout_s = self._timeout(request)
        if request.operation == ManageMappingSession.Goal.OP_STATUS:
            return self._repository.load(request.session_id)
        if request.operation == ManageMappingSession.Goal.OP_START:
            arguments = dict(zip(request.argument_keys, request.argument_values))
            arguments.setdefault("platform_profile", str(self._platform_profile))
            session, launch_arguments = self._repository.prepare(request.map_id, arguments)
            session = self._repository.update(session, "STARTING")
            self._feedback(goal_handle, "STARTING", 0.1, "starting managed mapping capture")
            try:
                session = self._start_mapping_capture(session, timeout_s)
                self._change_mode(
                    ChangeSystemMode.Goal.MODE_MAPPING,
                    profile="mapping",
                    arguments=launch_arguments,
                    timeout_s=timeout_s,
                )
            except Exception as error:
                try:
                    self._stop_mapping_capture(
                        session, min(timeout_s, 30.0), interrupt=True
                    )
                except Exception:
                    pass
                self._repository.update(
                    session,
                    "START_FAILED",
                    last_error_code="mapping_start_failed",
                    last_error=str(error),
                )
                raise MappingSessionError("mapping_start_failed", str(error)) from error
            session = self._repository.update(session, "MAPPING")
            self._feedback(goal_handle, "MAPPING", 1.0, "mapping capture is running")
            return session
        if request.operation == ManageMappingSession.Goal.OP_FINALIZE_CAPTURE:
            session = self._repository.load(request.session_id)
            return self._repository.finalize_capture(
                request.session_id,
                save_grid=self._save_grid,
                stop_mapping=lambda remaining: self._stop_mapping_capture(
                    session, remaining
                ),
                build_candidate=self._build_static_candidate,
                timeout_s=timeout_s,
                feedback=lambda state, progress, message: self._feedback(
                    goal_handle, state, progress, message
                ),
            )
        if request.operation == ManageMappingSession.Goal.OP_COMMIT:
            self._feedback(goal_handle, "COMMITTING", 0.1, "validating candidate assets")
            session = self._repository.commit(
                request.session_id,
                map_registry=self._registry,
                activate=bool(request.activate_after_commit),
            )
            self._feedback(goal_handle, "REGISTERED", 1.0, "map version registered")
            return session
        if request.operation == ManageMappingSession.Goal.OP_DISCARD:
            session = self._repository.load(request.session_id)
            if str(session.get("state")) in {
                "STARTING",
                "MAPPING",
                "SAVING_GRID",
                "STOPPING_MAPPING",
                "WAITING_ASSETS",
            }:
                self._feedback(goal_handle, "STOPPING_MAPPING", 0.2, "stopping test capture")
                self._stop_mapping_capture(session, timeout_s, interrupt=True)
                session = self._repository.update(session, "ABORTED")
            failed_version_id = str(session.get("failed_map_version_id", ""))
            if failed_version_id:
                try:
                    self._registry.discard_failed(failed_version_id)
                except MappingSessionError:
                    pass
            return self._repository.discard(str(session["session_id"]))
        raise MappingSessionError("invalid_operation", "unsupported mapping-session operation")

    def _execute(self, goal_handle: Any) -> ManageMappingSession.Result:
        result = ManageMappingSession.Result()
        session = None
        if not self._operation_lock.acquire(blocking=False):
            result.error_code = ManageMappingSession.Result.ERROR_INVALID_STATE
            result.message = "another mapping-session operation is already running"
            goal_handle.abort()
            return result
        try:
            session = self._execute_operation(goal_handle)
            result.success = True
            result.error_code = ManageMappingSession.Result.ERROR_NONE
            result.message = str(session.get("last_error", "")) or str(session.get("state", ""))
            self._fill_result(result, session)
            goal_handle.succeed()
        except MappingSessionError as error:
            result.success = False
            result.error_code = _ERROR_CODES.get(
                error.code, ManageMappingSession.Result.ERROR_INTERNAL
            )
            result.message = str(error)
            try:
                session = self._repository.load(goal_handle.request.session_id)
            except MappingSessionError:
                session = None
            self._fill_result(result, session)
            goal_handle.abort()
        except Exception as error:
            result.success = False
            result.error_code = ManageMappingSession.Result.ERROR_INTERNAL
            result.message = str(error)
            self.get_logger().error(f"mapping-session operation failed: {error}")
            goal_handle.abort()
        finally:
            self._operation_lock.release()
        return result

    def destroy_node(self):
        self._server.destroy()
        self._mode_action.destroy()
        self._mapping_save.destroy()
        self._map_client.destroy()
        self._bag_client.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingSessionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
