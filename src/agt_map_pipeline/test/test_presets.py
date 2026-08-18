from dataclasses import asdict
import pytest, yaml
from agt_map_pipeline.presets import resolve_preset, write_resolved_preset

def test_greenhouse_preset_is_declarative_and_requires_human_semantics():
    p = resolve_preset("greenhouse")
    assert p.name == "greenhouse" and p.version == "1"
    assert p.structure_config and p.corridor_config and p.aisle_graph_config
    assert "site_boundary" in p.required_human_layers
    assert p.corridor_config.enable_boundary_aisles is False

def test_resolved_preset_is_fully_materialized(tmp_path):
    p = resolve_preset("greenhouse")
    out = tmp_path / "resolved.yaml"
    payload = write_resolved_preset(p, out)
    stored = yaml.safe_load(out.read_text())
    assert stored == payload
    assert stored["navigation"] == asdict(p.navigation_config)
    assert stored["structure"] == asdict(p.structure_config)

def test_unknown_preset_fails_closed():
    with pytest.raises(ValueError, match="unsupported preset"):
        resolve_preset("magic")
