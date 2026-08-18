import pytest

from agt_map_workbench.navigation_override_metadata import (
    build_navigation_override_record,
    next_override_id,
    validate_navigation_override_records,
)


POLYGON = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]


def test_build_formal_force_free_override():
    record = build_navigation_override_record(
        override_id="ovr_0001",
        mode="force_free",
        polygon_xy=POLYGON,
        reason="Sparse-return hole contradicted by PCD review.",
        evidence_category="pcd_inspection",
    )
    assert record == {
        "id": "ovr_0001",
        "mode": "force_free",
        "polygon_xy": POLYGON,
        "reason": "Sparse-return hole contradicted by PCD review.",
        "evidence_category": "pcd_inspection",
    }


def test_formal_override_rejects_blank_reason_and_unsupported_evidence():
    with pytest.raises(ValueError, match="reason"):
        build_navigation_override_record(
            override_id="ovr_0001",
            mode="force_free",
            polygon_xy=POLYGON,
            reason="  ",
            evidence_category="pcd_inspection",
        )
    with pytest.raises(ValueError, match="evidence"):
        build_navigation_override_record(
            override_id="ovr_0001",
            mode="force_free",
            polygon_xy=POLYGON,
            reason="documented",
            evidence_category="planner_result",
        )


def test_formal_override_rejects_unknown_and_no_go_modes():
    for mode in ("unknown", "no_go"):
        with pytest.raises(ValueError, match="force_free or force_occupied"):
            build_navigation_override_record(
                override_id="ovr_0001",
                mode=mode,
                polygon_xy=POLYGON,
                reason="documented",
                evidence_category="field_note",
            )


def test_record_sequence_requires_unique_nonempty_ids():
    first = build_navigation_override_record(
        override_id="ovr_0001",
        mode="force_free",
        polygon_xy=POLYGON,
        reason="documented",
        evidence_category="field_note",
    )
    duplicate = dict(first)
    with pytest.raises(ValueError, match="duplicate"):
        validate_navigation_override_records([first, duplicate])
    with pytest.raises(ValueError, match="id"):
        build_navigation_override_record(
            override_id=" ",
            mode="force_free",
            polygon_xy=POLYGON,
            reason="documented",
            evidence_category="field_note",
        )


def test_next_override_id_is_stable_and_monotonic():
    records = [
        {"id": "ovr_0001"},
        {"id": "ovr_0003"},
        {"id": "legacy_name"},
    ]
    assert next_override_id(records) == "ovr_0004"
    assert next_override_id([]) == "ovr_0001"
