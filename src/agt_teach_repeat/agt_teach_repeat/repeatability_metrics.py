"""ROS-independent system-internal repeatability metrics."""

import math

from .path_processing import deterministic_percentile, nearest_segment_metrics, path_length
from .path_types import TeachRepeatError


INTERNAL_REPEATABILITY_NOTICE = (
    "The trajectory metrics use the robot's onboard localization estimate and measure "
    "system-internal repeatability. They are not an independent absolute-position ground truth."
)
INTERNAL_REPEATABILITY_NOTICE_ZH = (
    "轨迹指标使用机器人机载定位估计，衡量的是系统内部重复性；它们不是独立的绝对位置真值。"
)


def repeatability_metrics(
    reference,
    executed,
    *,
    duration_s=0.0,
    tracking_lost_count=0,
    localization_degraded_count=0,
    emergency_stop_count=0,
    manual_intervention_count=0,
    collision_monitor_stop_count=0,
):
    reference = tuple(reference)
    executed = tuple(executed)
    if len(reference) < 2:
        raise TeachRepeatError("path_too_short", "reference path requires at least two poses")
    if not executed:
        raise TeachRepeatError("empty_executed_path", "executed path requires samples")
    projections = [
        nearest_segment_metrics(reference, pose.x, pose.y, pose.yaw)
        for pose in executed
    ]
    lateral = [abs(item["cross_track_error"]) for item in projections]
    yaw_degrees = [abs(math.degrees(item["heading_error"])) for item in projections]

    def square_mean(values):
        return math.sqrt(sum(value * value for value in values) / len(values))
    total = path_length(reference)
    return {
        "completion_ratio": (
            max(item["along_track_progress"] for item in projections) / total
            if total
            else 0.0
        ),
        "reference_length_m": total,
        "executed_length_m": path_length(executed),
        "lateral_mean_m": sum(lateral) / len(lateral),
        "lateral_rmse_m": square_mean(lateral),
        "lateral_p95_m": deterministic_percentile(lateral, 95.0),
        "lateral_max_m": max(lateral),
        "yaw_mean_abs_deg": sum(yaw_degrees) / len(yaw_degrees),
        "yaw_rmse_deg": square_mean(yaw_degrees),
        "yaw_p95_deg": deterministic_percentile(yaw_degrees, 95.0),
        "yaw_max_deg": max(yaw_degrees),
        "duration_s": float(duration_s),
        "tracking_lost_count": int(tracking_lost_count),
        "localization_degraded_count": int(localization_degraded_count),
        "emergency_stop_count": int(emergency_stop_count),
        "manual_intervention_count": int(manual_intervention_count),
        "collision_monitor_stop_count": int(collision_monitor_stop_count),
        "measurement_basis": "onboard_localization_system_internal_repeatability",
        "ground_truth_independent": False,
        "trajectory_metrics_available": True,
        "notice": INTERNAL_REPEATABILITY_NOTICE,
    }
