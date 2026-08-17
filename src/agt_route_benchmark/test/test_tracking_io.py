from pathlib import Path

import pytest

from agt_route_benchmark.tracking_io import read_executed_trajectory_csv, write_executed_trajectory_csv
from agt_route_benchmark.tracking_metrics import ExecutedPose


def test_executed_trajectory_csv_roundtrip(tmp_path: Path):
    path = tmp_path / "executed.csv"
    poses = (
        ExecutedPose(0.0, 1.0, 2.0, 0.1),
        ExecutedPose(0.05, 1.1, 2.0, 0.1),
    )
    write_executed_trajectory_csv(poses, path)
    assert read_executed_trajectory_csv(path) == poses
    assert path.read_text(encoding="utf-8").splitlines()[0] == "stamp_s,x_m,y_m,yaw_rad"


def test_executed_trajectory_requires_monotonic_timestamps(tmp_path: Path):
    path = tmp_path / "executed.csv"
    path.write_text(
        "stamp_s,x_m,y_m,yaw_rad\n1.0,0.0,0.0,0.0\n0.5,1.0,0.0,0.0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="monotonic"):
        read_executed_trajectory_csv(path)
