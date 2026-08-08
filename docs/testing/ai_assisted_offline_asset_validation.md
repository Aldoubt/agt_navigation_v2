# AI-assisted offline asset validation

AI/Codex may read rosbag metadata, calculate hashes, run contract/preflight
tests, inspect Dataset/Recipe/Map/Route lineage, aggregate metrics, compare
experiments and generate machine-readable reports. It may repair a test or
tool only when the failure and intended contract are explicit.

AI must not modify an original bag, silently change map coordinates, alter
calibration truth, promote provisional calibration, claim vehicle accuracy
from a handheld bag, mix RTK truth into the estimator under evaluation, or
weaken a fail-closed contract to make a test pass.

Each run records repository commit, branch, dirty state, commands, input
hashes, generated identities, failures and remaining manual decisions under
`runtime/reports/offline_asset_validation/<run_id>/`. `report.json` is the
machine-readable source; `report.md` is an operator summary and
`commands.log` is the exact command log.

The report generator is an audit helper, not an approval authority. Alignment,
calibration verification, route readiness and vehicle acceptance remain
explicit gates requiring recorded human evidence.
