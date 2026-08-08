from agt_offline_assets import append_cleaning_operation


def test_cleaning_operation_records_hashes_without_overwriting_input(tmp_path):
    source, output = tmp_path / "raw.pcd", tmp_path / "clean.pcd"
    source.write_text("raw\n", encoding="utf-8")
    output.write_text("clean\n", encoding="utf-8")
    report = append_cleaning_operation(tmp_path / "cleaning.yaml", operation="crop_box", parameters={"min_z": 0}, input_path=source, output_path=output, operator_note="operator patch")
    assert report["status"] == "PASS"
    assert source.read_text(encoding="utf-8") == "raw\n"
    assert "sha256:" in (tmp_path / "cleaning.yaml").read_text(encoding="utf-8")
