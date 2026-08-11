from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/interfaces/pointcloud_processing_recipe.md"
CLI = ROOT / "src/agt_offline_assets/scripts/agt_offline_assets_cli.py"
PROCESSING = ROOT / "src/agt_offline_assets/agt_offline_assets/pointcloud_processing.py"
PROFILE = ROOT / "src/agt_offline_assets/agt_offline_assets/pointcloud_profile.py"
PCD_IO = ROOT / "src/agt_offline_assets/agt_offline_assets/pcd_io.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_v25_12b_recipe_and_run_schemas_are_explicit():
    doc = _read(DOC)
    processing = _read(PROCESSING)
    for token in (
        "agt_pointcloud_processing_recipe/v1",
        "agt_pointcloud_processing/v1",
        "agt_pointcloud_processing_report/v1",
        "processing_content_sha256",
    ):
        assert token in doc or token in processing


def test_v25_12b_supported_operations_are_locked():
    text = _read(PROCESSING)
    for operation in (
        "remove_nonfinite",
        "crop_box",
        "crop_polygon",
        "delete_polygon",
        "height_range",
        "voxel_downsample",
        "sor",
        "radius_outlier",
        "ground_separation",
    ):
        assert f'"{operation}"' in text


def test_v25_12b_cli_exposes_inspect_process_and_validate():
    cli = _read(CLI)
    for command in (
        "inspect-pcd",
        "process-pointcloud",
        "validate-pointcloud-processing",
    ):
        assert f'"{command}"' in cli
    assert '"--profile"' in cli
    assert '"--sample-limit"' in cli
    assert "summarize_pointcloud" in cli


def test_v25_12b_profile_is_read_only_deterministic_and_sample_bounded():
    profile = _read(PROFILE)
    for token in (
        "deterministic_even_spacing",
        "sample_limit",
        "percentiles_sampled",
        "finite_xyz_count",
        "nonfinite_xyz_count",
    ):
        assert token in profile
    assert "np.linspace" in profile


def test_v25_12b_is_non_destructive_and_not_a_quality_acceptance():
    doc = _read(DOC)
    assert "source PCD remains read-only" in doc
    assert "does not mean the resulting point cloud has passed localization quality" in doc
    assert "WorkBench" not in doc  # canonical spelling is Workbench
    assert "AGT Map Workbench" in doc


def test_v25_12b_accepts_pcl_compressed_input_but_normalizes_formal_output():
    pcd = _read(PCD_IO)
    doc = _read(DOC)
    assert '"binary_compressed"' in pcd
    assert "_lzf_decompress" in pcd
    assert "structure-of-arrays" in pcd
    assert 'mode not in {"ascii", "binary", "binary_compressed"}' in pcd
    assert 'mode not in {"ascii", "binary"}' in pcd  # writer remains normalized
    assert "binary_compressed input" in doc
    assert "output_data" in doc


def test_v25_12b_does_not_add_runtime_ros_interfaces():
    action_dir = ROOT / "src/agt_interfaces/action"
    service_dir = ROOT / "src/agt_interfaces/srv"
    forbidden = (
        "ProcessPointCloud.action",
        "EditPointCloud.action",
        "ProcessPointCloud.srv",
        "EditPointCloud.srv",
    )
    for name in forbidden:
        assert not (action_dir / name).exists()
        assert not (service_dir / name).exists()
