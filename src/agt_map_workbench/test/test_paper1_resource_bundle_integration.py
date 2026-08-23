from pathlib import Path
from types import SimpleNamespace

from agt_map_workbench import paper1_workbench as module


def test_12f_bundle_writer_freezes_boundary_and_does_not_assume_formal_map_selected(
    tmp_path: Path, monkeypatch
):
    window = SimpleNamespace(
        _traversability_evidence=object(),
        _navigation_result=object(),
        _site_boundary=object(),
    )

    calls = {}

    def write_boundary(boundary, path, *, overwrite=False):
        assert boundary is window._site_boundary
        assert overwrite is False
        path = Path(path)
        path.write_text("frame_id: map\n", encoding="utf-8")
        calls["boundary"] = path
        return path

    def write_candidate(
        evidence,
        navigation,
        boundary,
        output_dir,
        *,
        source_navigation_asset,
        overwrite,
    ):
        assert (Path(output_dir) / "site_boundary.yaml").is_file()
        calls["source_navigation_asset"] = source_navigation_asset
        (Path(output_dir) / "navigation_map_12f.yaml").write_text(
            "image: navigation_map_12f.pgm\n", encoding="utf-8"
        )
        return Path(output_dir)

    monkeypatch.setattr(module, "write_site_boundary", write_boundary)
    monkeypatch.setattr(module, "write_traversability_candidate", write_candidate)

    writer = module.Paper1MapWorkbenchWindow._traversability_writer(window)
    writer(tmp_path)

    assert calls["boundary"] == tmp_path / "layers" / "traversability" / "site_boundary.yaml"
    assert calls["source_navigation_asset"] == "WORKBENCH_CURRENT_NAVIGATION"
