"""Fail-closed placeholder for future offline map optimization backends."""


NOT_IMPLEMENTED = 1


def reject_optimization(backend: str) -> tuple[bool, int, str]:
    if backend not in {"pose_graph", "factor_graph", "visual_ba", "noop"}:
        return False, NOT_IMPLEMENTED, "unsupported optimization backend"
    return False, NOT_IMPLEMENTED, "map optimization backend is reserved but not implemented"
