"""Bounded relocalization-mode policy, independent of ROS and Action transport."""

from dataclasses import dataclass


MANUAL_ONLY = "MANUAL_ONLY"
AUTO_ON_START = "AUTO_ON_START"
AUTO_RECOVERY = "AUTO_RECOVERY"


@dataclass
class RelocalizationPolicy:
    mode: str = MANUAL_ONLY
    max_attempts: int = 3
    retry_cooldown_s: float = 10.0
    total_timeout_s: float = 60.0
    max_candidates: int = 128
    attempts: int = 0
    exhausted: bool = False
    startup_pending: bool = False
    last_attempt_at: float | None = None
    deadline: float | None = None

    def __post_init__(self) -> None:
        if self.max_attempts <= 0 or self.retry_cooldown_s < 0.0 or self.total_timeout_s <= 0.0 or self.max_candidates <= 0:
            raise ValueError("relocalization retry limits must be positive and bounded")
        self.set_mode(self.mode)

    def set_mode(self, mode: str) -> None:
        if mode not in {MANUAL_ONLY, AUTO_ON_START, AUTO_RECOVERY}:
            raise ValueError(f"unsupported relocalization mode: {mode}")
        self.mode = mode
        self.attempts = 0
        self.exhausted = False
        self.last_attempt_at = None
        self.deadline = None
        self.startup_pending = mode == AUTO_ON_START

    def should_trigger(self, *, now: float, localization_state: str, map_ready: bool, cloud_healthy: bool) -> bool:
        if self.mode == MANUAL_ONLY or self.exhausted or not map_ready or not cloud_healthy:
            return False
        if self.mode == AUTO_ON_START:
            return self.startup_pending
        if localization_state not in {"DEGRADED", "RECOVERING"}:
            return False
        if self.attempts >= self.max_attempts:
            self.exhausted = True
            return False
        if self.last_attempt_at is not None and now - self.last_attempt_at < self.retry_cooldown_s:
            return False
        if self.deadline is not None and now >= self.deadline:
            self.exhausted = True
            return False
        return True

    def start_attempt(self, now: float) -> None:
        if self.attempts >= self.max_attempts:
            self.exhausted = True
            return
        self.attempts += 1
        self.last_attempt_at = now
        if self.deadline is None:
            self.deadline = now + self.total_timeout_s
        self.startup_pending = False

    def finish_attempt(self, success: bool) -> None:
        if success:
            self.attempts = 0
            self.exhausted = False
            self.deadline = None
            return
        if self.mode == AUTO_ON_START or self.attempts >= self.max_attempts:
            self.exhausted = True

    def snapshot(self) -> dict[str, object]:
        return {
            "mode": self.mode,
            "attempts": self.attempts,
            "max_attempts": self.max_attempts,
            "retry_cooldown_s": self.retry_cooldown_s,
            "total_timeout_s": self.total_timeout_s,
            "max_candidates": self.max_candidates,
            "exhausted": self.exhausted,
            "startup_pending": self.startup_pending,
        }
