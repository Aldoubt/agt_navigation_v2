from __future__ import annotations
from abc import ABC, abstractmethod
from ..contracts import ExperimentSpec, PlannerResult


class PlannerAdapter(ABC):
    @abstractmethod
    def plan(self, spec: ExperimentSpec) -> PlannerResult:
        raise NotImplementedError
