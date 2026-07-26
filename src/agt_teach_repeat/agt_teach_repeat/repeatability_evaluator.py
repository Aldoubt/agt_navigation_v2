"""System-internal teach path repeatability metrics and ROS recorder."""

import json
import math
import time

from agt_interfaces.msg import LocalizationStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, String

from .path_io import (
    RAW_FIELDS,
    atomic_write_csv,
    atomic_write_json,
    load_manifest,
    load_reference_path,
    manifest_reference_path,
    pose_to_dict,
)
from .repeatability_metrics import (
    INTERNAL_REPEATABILITY_NOTICE,
    INTERNAL_REPEATABILITY_NOTICE_ZH,
    repeatability_metrics,
)
from .path_processing import path_length
from .path_types import PathPose, TeachRepeatError
from .ros_utils import latched_qos, quaternion_yaw


class RepeatabilityEvaluator(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("agt_repeatability_evaluator", parameter_overrides=parameter_overrides)
        manifest_value = str(self.declare_parameter("manifest", "").value)
        self.experiment_root = str(self.declare_parameter("experiment_root", "").value)
        self.experiment_id = str(self.declare_parameter("experiment_id", "").value)
        self.language = str(self.declare_parameter("report_language", "zh_CN").value)
        self.safety_status_name = str(
            self.declare_parameter(
                "safety_status_name", "agt_safety/tracked_controller"
            ).value
        )
        if not manifest_value:
            raise RuntimeError("manifest parameter is required")
        self.manifest_path, self.manifest = load_manifest(manifest_value)
        self.reference = load_reference_path(
            manifest_reference_path(self.manifest_path, self.manifest),
            expected_demo_id=self.manifest["demo_id"],
        )
        self.metrics_publisher = self.create_publisher(
            String, "/agt/teach/metrics", latched_qos()
        )
        self.create_subscription(
            String,
            "/agt/teach/execution_status",
            self._execution_callback,
            latched_qos(),
        )
        self.create_subscription(
            LocalizationStatus,
            "/agt/localization/status",
            self._localization_callback,
            50,
        )
        self.create_subscription(
            Odometry,
            "/agt/mapping/odometry",
            self._mapping_odom_callback,
            50,
        )
        self.create_subscription(
            Odometry,
            "/agt/chassis/odometry",
            self._chassis_odom_callback,
            50,
        )
        self.create_subscription(Twist, "/agt/navigation/cmd_vel_raw", self._raw_cmd_callback, 20)
        self.create_subscription(
            Twist, "/agt/navigation/cmd_vel", self._collision_cmd_callback, 20
        )
        self.create_subscription(Bool, "/agt/safety/emergency_stop", self._estop_callback, 20)
        self.create_subscription(
            DiagnosticArray, "/agt/safety/status", self._safety_callback, 20
        )
        self._reset("")

    def _reset(self, run_id):
        self.run_id = str(run_id)
        self.started_at = None
        self.executed = []
        self.localization_rows = []
        self.mapping_rows = []
        self.chassis_rows = []
        self.safety_rows = []
        self.tracking_lost_count = 0
        self.degraded_count = 0
        self.estop_count = 0
        self.manual_count = 0
        self.collision_stop_count = 0
        self.safety_readiness_loss_count = 0
        self._previous_localization_state = None
        self._previous_estop = False
        self._raw_cmd_moving = False
        self._collision_stopped = False
        self._previous_safety_ready = None

    def _execution_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        state = str(status.get("state", ""))
        run_id = str(status.get("run_id", ""))
        if state in {"STARTING", "RUNNING"} and run_id and self.run_id != run_id:
            self._reset(run_id)
            self.started_at = time.monotonic()
        if state in {"SUCCEEDED", "FAILED", "CANCELED"} and run_id == self.run_id:
            self._finalize(status)

    def _localization_callback(self, message):
        state = int(message.state)
        if state != self._previous_localization_state:
            if state == LocalizationStatus.STATE_LOST:
                self.tracking_lost_count += 1
            elif state == LocalizationStatus.STATE_DEGRADED:
                self.degraded_count += 1
            self._previous_localization_state = state
        row = {
            "timestamp_ns": (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
            "state": state,
            "pose_valid": bool(message.pose_valid),
            "localization_accepted": bool(message.localization_accepted),
            "has_converged": bool(message.has_converged),
            "ambiguous_result": bool(message.ambiguous_result),
            "status_stale": bool(message.status_stale),
            "error_code": int(message.error_code),
            "map_id": str(message.map_id),
            "map_hash": str(message.map_hash),
            "fitness_score": _finite_or_none(message.fitness_score),
            "overlap_ratio": _finite_or_none(message.overlap_ratio),
            "inlier_ratio": _finite_or_none(message.inlier_ratio),
            "ambiguity_score": _finite_or_none(message.ambiguity_score),
            "translation_innovation": _finite_or_none(
                message.translation_innovation
            ),
            "yaw_innovation": _finite_or_none(message.yaw_innovation),
            "runtime_ms": _finite_or_none(message.runtime_ms),
            "tested_candidates": int(message.tested_candidates),
            "total_candidates": int(message.total_candidates),
        }
        self.localization_rows.append(row)
        pose = message.global_pose.pose.pose
        if self.run_id and message.pose_valid and message.global_pose.header.frame_id == "map":
            try:
                self.executed.append(
                    PathPose(
                        timestamp_ns=row["timestamp_ns"],
                        x=pose.position.x,
                        y=pose.position.y,
                        z=pose.position.z,
                        qx=pose.orientation.x,
                        qy=pose.orientation.y,
                        qz=pose.orientation.z,
                        qw=pose.orientation.w,
                        frame_id="map",
                        child_frame_id="base_footprint",
                    ).normalized()
                )
            except TeachRepeatError:
                pass

    def _mapping_odom_callback(self, message):
        if self.run_id:
            self.mapping_rows.append(_odom_row(message))

    def _chassis_odom_callback(self, message):
        if self.run_id:
            self.chassis_rows.append(_odom_row(message))

    def _raw_cmd_callback(self, message):
        self._raw_cmd_moving = abs(message.linear.x) > 1.0e-3 or abs(message.angular.z) > 1.0e-3

    def _collision_cmd_callback(self, message):
        stopped = (
            self._raw_cmd_moving
            and abs(message.linear.x) <= 1.0e-3
            and abs(message.angular.z) <= 1.0e-3
        )
        if stopped and not self._collision_stopped:
            self.collision_stop_count += 1
        self._collision_stopped = stopped

    def _estop_callback(self, message):
        current = bool(message.data)
        if current and not self._previous_estop:
            self.estop_count += 1
        self._previous_estop = current

    def _safety_callback(self, message):
        ready = False
        motion_enabled = ""
        estop_latched = ""
        for status in message.status:
            if status.name != self.safety_status_name:
                continue
            values = {item.key: item.value.lower() for item in status.values}
            motion_enabled = values.get("motion_enabled", "")
            estop_latched = values.get("estop_latched", "")
            ready = motion_enabled == "true" and estop_latched == "false"
            break
        if self.run_id:
            if self._previous_safety_ready is True and not ready:
                self.safety_readiness_loss_count += 1
            stamp = message.header.stamp
            self.safety_rows.append(
                {
                    "timestamp_ns": (
                        int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    ),
                    "ready": ready,
                    "status_name": self.safety_status_name,
                    "motion_enabled": motion_enabled,
                    "estop_latched": estop_latched,
                }
            )
            self._previous_safety_ready = ready

    def _finalize(self, execution_status):
        if (
            execution_status.get("state") == "CANCELED"
            and execution_status.get("failure_reason") == "USER_CANCELED"
        ):
            self.manual_count += 1
        duration = time.monotonic() - self.started_at if self.started_at is not None else 0.0
        if self.executed:
            metrics = repeatability_metrics(
                self.reference,
                self.executed,
                duration_s=duration,
                tracking_lost_count=self.tracking_lost_count,
                localization_degraded_count=self.degraded_count,
                emergency_stop_count=self.estop_count,
                manual_intervention_count=self.manual_count,
                collision_monitor_stop_count=self.collision_stop_count,
            )
        else:
            metrics = _empty_repeatability_metrics(
                duration,
                self.tracking_lost_count,
                self.degraded_count,
                self.estop_count,
                self.manual_count,
                self.collision_stop_count,
                path_length(self.reference),
            )
        metrics["safety_readiness_loss_count"] = self.safety_readiness_loss_count
        metrics["demo_id"] = self.manifest["demo_id"]
        metrics["run_id"] = self.run_id
        metrics["execution_result"] = execution_status
        run_dir = self.manifest_path.parent / "runs" / self.run_id
        atomic_write_csv(
            run_dir / "executed_path.csv",
            (pose_to_dict(pose) for pose in self.executed),
            RAW_FIELDS,
        )
        atomic_write_csv(
            run_dir / "localization_samples.csv",
            self.localization_rows,
            (
                "timestamp_ns",
                "state",
                "pose_valid",
                "localization_accepted",
                "has_converged",
                "ambiguous_result",
                "status_stale",
                "error_code",
                "map_id",
                "map_hash",
                "fitness_score",
                "overlap_ratio",
                "inlier_ratio",
                "ambiguity_score",
                "translation_innovation",
                "yaw_innovation",
                "runtime_ms",
                "tested_candidates",
                "total_candidates",
            ),
        )
        atomic_write_csv(
            run_dir / "mapping_odometry.csv",
            self.mapping_rows,
            _ODOMETRY_FIELDS,
        )
        atomic_write_csv(
            run_dir / "chassis_odometry.csv",
            self.chassis_rows,
            _ODOMETRY_FIELDS,
        )
        atomic_write_csv(
            run_dir / "safety_samples.csv",
            self.safety_rows,
            (
                "timestamp_ns",
                "ready",
                "status_name",
                "motion_enabled",
                "estop_latched",
            ),
        )
        atomic_write_json(run_dir / "metrics.json", metrics)
        (run_dir / "report.md").parent.mkdir(parents=True, exist_ok=True)
        from .path_io import _atomic_bytes
        _atomic_bytes(run_dir / "report.md", self._report(metrics).encode("utf-8"))
        if self.experiment_root and self.experiment_id:
            from agt_experiment_manager.manager import ExperimentManager

            manager = ExperimentManager(self.experiment_root)
            failure_context = self._failure_context(execution_status, metrics)
            manager.record_teach_repeat_result(
                self.experiment_id,
                demo_id=self.manifest["demo_id"],
                run_id=self.run_id,
                teach_manifest=str(self.manifest_path),
                reference_path_hash=self.manifest["assets"]["reference_path_sha256"],
                map_identity=self.manifest["map"],
                repeatability_metrics=metrics,
                localization_summary={
                    "tracking_lost_count": self.tracking_lost_count,
                    "degraded_count": self.degraded_count,
                },
                execution_result=execution_status,
                failure_case=failure_context,
            )
            if failure_context:
                manager.record_failure_case(
                    self.experiment_id,
                    demo_id=self.manifest["demo_id"],
                    run_id=self.run_id,
                    **failure_context,
                )
        message = String()
        message.data = json.dumps(metrics, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self.metrics_publisher.publish(message)
        self.run_id = ""

    def _failure_context(self, execution_status, metrics):
        if execution_status.get("state") == "SUCCEEDED":
            return None
        reason = str(execution_status.get("failure_reason") or "EXECUTION_FAILED")
        category = "".join(
            character if character.isalnum() else "_"
            for character in reason.upper().split(":", 1)[0]
        ).strip("_")
        category = (category or "EXECUTION_FAILED")[:64]
        last_pose = pose_to_dict(self.executed[-1]) if self.executed else {}
        latest_localization = self.localization_rows[-1] if self.localization_rows else {}
        latest_safety = self.safety_rows[-1] if self.safety_rows else {}
        return {
            "category": category,
            "robot_pose": last_pose,
            "reference_progress": float(
                execution_status.get("distance_completed_m", 0.0)
            ),
            "lateral_error_m": float(
                execution_status.get("lateral_error_m", 0.0)
            ),
            "localization_status": latest_localization,
            "navigation_status": dict(execution_status),
            "safety_status": latest_safety,
            "operator_note": "",
        }

    def _report(self, metrics):
        notice = (
            INTERNAL_REPEATABILITY_NOTICE_ZH
            if self.language == "zh_CN"
            else INTERNAL_REPEATABILITY_NOTICE
        )
        return "\n".join(
            [
                f"# Teach Repeat {metrics['run_id']}",
                "",
                f"- Demo: `{metrics['demo_id']}`",
                f"- Completion: `{metrics['completion_ratio']:.6f}`",
                f"- Lateral RMSE: `{_metric_text(metrics['lateral_rmse_m'], 'm')}`",
                f"- Lateral P95: `{_metric_text(metrics['lateral_p95_m'], 'm')}`",
                f"- Yaw RMSE: `{_metric_text(metrics['yaw_rmse_deg'], 'deg')}`",
                f"- Emergency stops: `{metrics['emergency_stop_count']}`",
                "",
                notice,
                "",
            ]
        )


def _finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _metric_text(value, unit):
    return "not available" if value is None else f"{value:.6f} {unit}"


_ODOMETRY_FIELDS = (
    "timestamp_ns",
    "x",
    "y",
    "yaw",
    "frame_id",
    "child_frame_id",
)


def _empty_repeatability_metrics(
    duration,
    tracking_lost_count,
    degraded_count,
    estop_count,
    manual_count,
    collision_stop_count,
    reference_length,
):
    return {
        "completion_ratio": 0.0,
        "reference_length_m": float(reference_length),
        "executed_length_m": 0.0,
        "lateral_mean_m": None,
        "lateral_rmse_m": None,
        "lateral_p95_m": None,
        "lateral_max_m": None,
        "yaw_mean_abs_deg": None,
        "yaw_rmse_deg": None,
        "yaw_p95_deg": None,
        "yaw_max_deg": None,
        "duration_s": float(duration),
        "tracking_lost_count": int(tracking_lost_count),
        "localization_degraded_count": int(degraded_count),
        "emergency_stop_count": int(estop_count),
        "manual_intervention_count": int(manual_count),
        "collision_monitor_stop_count": int(collision_stop_count),
        "measurement_basis": "onboard_localization_system_internal_repeatability",
        "ground_truth_independent": False,
        "notice": INTERNAL_REPEATABILITY_NOTICE,
        "trajectory_metrics_available": False,
    }


def _odom_row(message):
    pose = message.pose.pose
    return {
        "timestamp_ns": (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        ),
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "yaw": quaternion_yaw(pose.orientation),
        "frame_id": str(message.header.frame_id),
        "child_frame_id": str(message.child_frame_id),
    }


def main(args=None):
    rclpy.init(args=args)
    node = RepeatabilityEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
