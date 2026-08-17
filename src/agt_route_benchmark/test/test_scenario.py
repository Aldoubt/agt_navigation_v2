from pathlib import Path
import pytest
from agt_route_benchmark.scenario import load_scenario


def test_p2p_requires_start_and_goal(tmp_path: Path):
    p = tmp_path / "bad.yaml"
    p.write_text("id: S01_straight_row\nlevel: p2p\ndevelopment_fixture: true\n", encoding="utf-8")
    with pytest.raises(ValueError, match="start"):
        load_scenario(p)


def test_formal_mode_rejects_development_fixture(tmp_path: Path):
    p = tmp_path / "s.yaml"
    p.write_text(
        "id: S01_straight_row\nlevel: p2p\ndevelopment_fixture: true\n"
        "start: {x: 0, y: 0, yaw: 0}\ngoal: {x: 1, y: 0, yaw: 0}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="development_fixture"):
        load_scenario(p, formal=True)


def test_mission_requires_semantic_ids(tmp_path: Path):
    p = tmp_path / "m.yaml"
    p.write_text("id: S06_full_mission\nlevel: mission\ndevelopment_fixture: false\nmission: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="required_semantic_ids"):
        load_scenario(p)


def test_mission_loads_reference_reachable_ids_without_making_them_required(tmp_path: Path):
    p = tmp_path / "m.yaml"
    p.write_text(
        "id: S06_full_mission\nlevel: mission\ndevelopment_fixture: false\n"
        "mission:\n  required_semantic_ids: [row_1, row_2, row_3]\n"
        "reference_reachable_semantic_ids: [row_1, row_3]\n",
        encoding="utf-8",
    )
    scenario = load_scenario(p)
    assert scenario.required_semantic_ids == ("row_1", "row_2", "row_3")
    assert scenario.reference_reachable_semantic_ids == ("row_1", "row_3")
