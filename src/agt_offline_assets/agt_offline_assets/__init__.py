"""Pure offline asset preparation primitives for AGT Navigation V2.5."""

from .contracts import (
    AssetContractError,
    DatasetBinding,
    DerivationRecipe,
    RoutePolicy,
    sha256_file,
    sha256_path_bundle,
)
from .workspace import (
    MapWorkspace,
    compute_map_content_sha256,
    create_map_workspace,
    refresh_map_manifest,
)
from .map_validation import MapComplianceResult, validate_map_workspace
from .session_ingest import MappingSessionIngestResult, ingest_mapping_session
from .route_asset import (
    RouteSample,
    create_route_candidate_asset,
    derive_route_candidate,
    load_route_csv,
    write_route_csv,
)
from .feasibility import FeasibilityResult, validate_route_asset
from .alignment import AlignmentResult, identity_alignment, solve_site_control_points, write_alignment_report
from .preview import write_route_preview
from .tuning import apply_route_tuning
from .cleaning import append_cleaning_operation
from .site_package import (
    SitePackageComplianceResult,
    compute_site_package_content_sha256,
    create_site_package,
    generate_site_package_id,
    refresh_site_package,
    validate_site_package,
)
from .pcd_io import PcdCloud, PcdSchema, read_pcd, write_pcd
from .pointcloud_profile import DEFAULT_PERCENTILES, summarize_pointcloud
from .pointcloud_processing import (
    PointCloudProcessingCompliance,
    PointCloudProcessingResult,
    load_pointcloud_recipe,
    process_pointcloud,
    validate_pointcloud_processing,
)
from .navigation_map_derivation import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    NAVIGATION_DERIVATION_SCHEMA,
    GroundRelativeNavigationConfig,
    NavigationMapResult,
    apply_navigation_overrides,
    derive_ground_relative_navigation_map,
    write_navigation_map_derivation,
)
from .navigation_structure import (
    NavigationStructureConfig,
    NavigationStructureResult,
    RowModel,
    derive_navigation_structure,
)
from .navigation_corridor import (
    CorridorRefinementConfig,
    CorridorRefinementResult,
    derive_corridor_refinement,
)

__all__ = [
    "AssetContractError",
    "DatasetBinding",
    "DerivationRecipe",
    "RoutePolicy",
    "sha256_file",
    "sha256_path_bundle",
    "MapWorkspace",
    "compute_map_content_sha256",
    "create_map_workspace",
    "refresh_map_manifest",
    "MapComplianceResult",
    "validate_map_workspace",
    "MappingSessionIngestResult",
    "ingest_mapping_session",
    "RouteSample",
    "create_route_candidate_asset",
    "derive_route_candidate",
    "load_route_csv",
    "write_route_csv",
    "FeasibilityResult",
    "validate_route_asset",
    "AlignmentResult",
    "identity_alignment",
    "solve_site_control_points",
    "write_alignment_report",
    "write_route_preview",
    "apply_route_tuning",
    "append_cleaning_operation",
    "SitePackageComplianceResult",
    "compute_site_package_content_sha256",
    "create_site_package",
    "generate_site_package_id",
    "refresh_site_package",
    "validate_site_package",
    "PcdCloud",
    "PcdSchema",
    "read_pcd",
    "write_pcd",
    "DEFAULT_PERCENTILES",
    "summarize_pointcloud",
    "PointCloudProcessingCompliance",
    "PointCloudProcessingResult",
    "load_pointcloud_recipe",
    "process_pointcloud",
    "validate_pointcloud_processing",
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "NAVIGATION_DERIVATION_SCHEMA",
    "GroundRelativeNavigationConfig",
    "NavigationMapResult",
    "apply_navigation_overrides",
    "derive_ground_relative_navigation_map",
    "write_navigation_map_derivation",
    "NavigationStructureConfig",
    "NavigationStructureResult",
    "RowModel",
    "derive_navigation_structure",
    "CorridorRefinementConfig",
    "CorridorRefinementResult",
    "derive_corridor_refinement",
]
