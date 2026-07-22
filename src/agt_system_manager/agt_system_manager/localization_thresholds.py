"""Display localization margins using the current relocalization parameters."""

from math import isfinite
from typing import Any, Mapping


def _parameters(config: Mapping[str, Any]) -> Mapping[str, Any]:
    root = config.get("/**", config)
    return root.get("ros__parameters", root) if isinstance(root, Mapping) else {}


def evaluate_localization_display(status: Mapping[str, Any], config: Mapping[str, Any]) -> dict[str, Any]:
    parameters = _parameters(config)
    fitness_limit = float(parameters.get("fitness_score_threshold", 2.0))
    translation_limit = float(parameters.get("max_translation_innovation", 5.0))
    yaw_limit = float(parameters.get("max_yaw_innovation", 1.5707963268))
    ambiguity_limit = float(parameters.get("ambiguity_ratio", 0.10))
    fitness = float(status.get("fitness_score", float("nan")))
    translation = float(status.get("translation_innovation", float("nan")))
    yaw = float(status.get("yaw_innovation", float("nan")))
    ambiguity = float(status.get("ambiguity_score", float("nan")))
    margins = {
        "fitness_score": fitness_limit - fitness if isfinite(fitness) else float("nan"),
        "translation_innovation": translation_limit - translation if isfinite(translation) else float("nan"),
        "yaw_innovation": yaw_limit - yaw if isfinite(yaw) else float("nan"),
        "ambiguity_score": ambiguity_limit - ambiguity if isfinite(ambiguity) else float("nan"),
    }
    accepted = bool(status.get("pose_valid")) and bool(status.get("localization_accepted"))
    stale = bool(status.get("status_stale"))
    ambiguous = bool(status.get("ambiguous_result"))
    finite_margins = all(isfinite(value) for value in margins.values())
    if stale or ambiguous or not accepted or status.get("error_code", 0) != 0 or not finite_margins or any(value < 0.0 for value in margins.values()):
        level = "FAIL"
    elif min(margins.values()) >= 0.5 * max(fitness_limit, translation_limit, yaw_limit, ambiguity_limit):
        level = "EXCELLENT"
    elif min(margins.values()) >= 0.0:
        level = "ACCEPTABLE"
    else:
        level = "SUSPECT"
    return {
        "level": level,
        "margins": margins,
        "thresholds": {
            "fitness_score": fitness_limit,
            "translation_innovation": translation_limit,
            "yaw_innovation": yaw_limit,
            "ambiguity_score": ambiguity_limit,
        },
    }
