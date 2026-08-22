from pathlib import Path

from agt_route_benchmark.paper1_baseline_snapshot import (
    P1_ACCEPTANCE_PURPOSE,
    write_p1_acceptance,
)


def test_write_p1_acceptance_records_formal_purpose(tmp_path: Path):
    output = tmp_path / "acceptance.yaml"
    write_p1_acceptance(output, accepted_at="2026-08-22T13:00:00+08:00")
    text = output.read_text(encoding="utf-8")
    assert P1_ACCEPTANCE_PURPOSE in text
    assert "not_e1_h_a4_human_effort_experiment: true" in text
