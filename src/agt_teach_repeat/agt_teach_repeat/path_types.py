"""Dependency-free data types for teach path assets."""

from dataclasses import dataclass, field
import math


class TeachRepeatError(ValueError):
    """Stable validation failure for a teach-repeat asset or request."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


def wrap_angle(value):
    value = float(value)
    if not math.isfinite(value):
        raise TeachRepeatError("non_finite_angle", "angle must be finite")
    return math.atan2(math.sin(value), math.cos(value))


def normalize_quaternion(qx, qy, qz, qw):
    values = tuple(float(value) for value in (qx, qy, qz, qw))
    if not all(math.isfinite(value) for value in values):
        raise TeachRepeatError(
            "non_finite_quaternion", "quaternion values must be finite"
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise TeachRepeatError("zero_quaternion", "quaternion norm must be non-zero")
    return tuple(value / norm for value in values)


def quaternion_from_yaw(yaw):
    yaw = wrap_angle(yaw)
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_yaw(qx, qy, qz, qw):
    x, y, z, w = normalize_quaternion(qx, qy, qz, qw)
    return wrap_angle(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


@dataclass(frozen=True)
class PathPose:
    timestamp_ns: int
    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    frame_id: str = "odom"
    child_frame_id: str = "base_footprint"

    def normalized(self):
        numeric = (
            self.x,
            self.y,
            self.z,
            self.linear_x,
            self.linear_y,
            self.angular_z,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise TeachRepeatError("non_finite_pose", "pose values must be finite")
        if int(self.timestamp_ns) < 0:
            raise TeachRepeatError("invalid_timestamp", "timestamp must be non-negative")
        qx, qy, qz, qw = normalize_quaternion(self.qx, self.qy, self.qz, self.qw)
        return PathPose(
            timestamp_ns=int(self.timestamp_ns),
            x=float(self.x),
            y=float(self.y),
            z=float(self.z),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            linear_x=float(self.linear_x),
            linear_y=float(self.linear_y),
            angular_z=float(self.angular_z),
            frame_id=str(self.frame_id),
            child_frame_id=str(self.child_frame_id),
        )

    @property
    def yaw(self):
        return quaternion_yaw(self.qx, self.qy, self.qz, self.qw)

    def with_yaw(self, yaw, *, frame_id=None):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        return PathPose(
            timestamp_ns=self.timestamp_ns,
            x=self.x,
            y=self.y,
            z=self.z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            linear_x=self.linear_x,
            linear_y=self.linear_y,
            angular_z=self.angular_z,
            frame_id=self.frame_id if frame_id is None else str(frame_id),
            child_frame_id=self.child_frame_id,
        )


@dataclass(frozen=True)
class TransformSE2:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(float(value)) for value in (self.x, self.y, self.z, self.yaw)):
            raise TeachRepeatError("non_finite_transform", "map transform must be finite")

    def apply(self, pose):
        pose = pose.normalized()
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        qx, qy, qz, qw = quaternion_from_yaw(wrap_angle(pose.yaw + self.yaw))
        return PathPose(
            timestamp_ns=pose.timestamp_ns,
            x=self.x + cosine * pose.x - sine * pose.y,
            y=self.y + sine * pose.x + cosine * pose.y,
            z=self.z + pose.z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            linear_x=pose.linear_x,
            linear_y=pose.linear_y,
            angular_z=pose.angular_z,
            frame_id="map",
            child_frame_id=pose.child_frame_id,
        )


@dataclass(frozen=True)
class ProcessingConfig:
    minimum_translation_m: float = 0.02
    minimum_yaw_change_rad: float = 0.02
    resample_distance_m: float = 0.10
    smoothing_enabled: bool = True
    smoothing_method: str = "moving_average"
    smoothing_window: int = 5
    max_smoothing_deviation_m: float = 0.05
    maximum_point_count: int = 20000
    control_point_spacing_m: float = 2.0
    control_point_curvature_threshold: float = 0.35
    maximum_control_points: int = 200

    def __post_init__(self):
        positive = (
            self.minimum_translation_m,
            self.minimum_yaw_change_rad,
            self.resample_distance_m,
            self.control_point_spacing_m,
        )
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0 for value in positive
        ):
            raise TeachRepeatError(
                "invalid_processing_config", "processing distances must be positive"
            )
        if (
            not math.isfinite(float(self.max_smoothing_deviation_m))
            or self.max_smoothing_deviation_m < 0.0
        ):
            raise TeachRepeatError(
                "invalid_processing_config",
                "smoothing deviation must be non-negative",
            )
        if self.smoothing_method != "moving_average":
            raise TeachRepeatError(
                "invalid_smoothing_method", "only moving_average is supported"
            )
        if self.smoothing_window < 1 or self.smoothing_window % 2 == 0:
            raise TeachRepeatError(
                "invalid_smoothing_window",
                "smoothing_window must be a positive odd integer",
            )
        if self.maximum_point_count < 2 or self.maximum_control_points < 2:
            raise TeachRepeatError(
                "invalid_processing_config", "point limits must be at least two"
            )


@dataclass
class ProcessingReport:
    raw_count: int = 0
    valid_count: int = 0
    processed_count: int = 0
    raw_length_m: float = 0.0
    processed_length_m: float = 0.0
    maximum_smoothing_deviation_m: float = 0.0
    maximum_curvature: float = 0.0
    warnings: list = field(default_factory=list)

    def to_dict(self):
        return {
            "raw_count": int(self.raw_count),
            "valid_count": int(self.valid_count),
            "processed_count": int(self.processed_count),
            "raw_length_m": float(f"{self.raw_length_m:.12g}"),
            "processed_length_m": float(f"{self.processed_length_m:.12g}"),
            "maximum_smoothing_deviation_m": float(
                f"{self.maximum_smoothing_deviation_m:.12g}"
            ),
            "maximum_curvature": float(f"{self.maximum_curvature:.12g}"),
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class ProcessedPath:
    poses: tuple
    control_points: tuple
    report: ProcessingReport
