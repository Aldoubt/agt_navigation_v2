"""Whitelisted launch profile and managed process utilities."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any, Callable, Mapping


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class LaunchProfile:
    profile_id: str
    mode: str
    command: tuple[str, ...]
    allowed_argument_keys: tuple[str, ...] = ()
    description: str = ""

    def build_command(self, arguments: Mapping[str, Any] | None = None) -> list[str]:
        arguments = arguments or {}
        unknown = set(arguments) - set(self.allowed_argument_keys)
        if unknown:
            raise ProfileError(
                f"profile {self.profile_id} rejects argument keys: {sorted(unknown)}"
            )
        command = list(self.command)
        for key in self.allowed_argument_keys:
            if key not in arguments:
                continue
            value = str(arguments[key])
            if not value or any(char in value for char in ("\x00", "\n", "\r")):
                raise ProfileError(f"invalid value for launch argument {key}")
            command.append(f"{key}:={value}")
        return command


class ProfileRegistry:
    def __init__(self, profiles: Mapping[str, Any], allowed_executables: tuple[str, ...] = ("ros2", "rviz2")):
        self._profiles: dict[str, LaunchProfile] = {}
        for profile_id, raw in profiles.items():
            command = raw.get("command")
            if not isinstance(command, list) or not command or not all(isinstance(item, str) for item in command):
                raise ProfileError(f"profile {profile_id} command must be a non-empty argv list")
            if command[0] not in allowed_executables:
                raise ProfileError(f"profile {profile_id} executable is not allowed")
            if any(any(char in item for char in ("\x00", "\n", "\r")) for item in command):
                raise ProfileError(f"profile {profile_id} contains invalid command text")
            self._profiles[profile_id] = LaunchProfile(
                profile_id=profile_id,
                mode=str(raw.get("mode", "IDLE")).upper(),
                command=tuple(command),
                allowed_argument_keys=tuple(str(item) for item in raw.get("allowed_argument_keys", [])),
                description=str(raw.get("description", "")),
            )

    def get(self, profile_id: str) -> LaunchProfile:
        try:
            return self._profiles[profile_id]
        except KeyError as exc:
            raise ProfileError(f"unknown launch profile: {profile_id}") from exc

    def ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._profiles))


@dataclass
class ManagedProcess:
    profile: LaunchProfile
    process: Any
    command: list[str]
    log_path: Path
    started_at: float
    pid: int

    def snapshot(self) -> dict[str, Any]:
        return {
            "profile": self.profile.profile_id,
            "mode": self.profile.mode,
            "pid": self.pid,
            "command": self.command,
            "started_at": self.started_at,
            "log_path": str(self.log_path),
            "returncode": self.process.poll(),
        }


class ProcessManager:
    """Own only process groups launched through the profile registry."""

    def __init__(
        self,
        registry: ProfileRegistry,
        runtime_dir: str | Path,
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.registry = registry
        self.runtime_dir = Path(runtime_dir)
        self._popen = popen_factory
        self._clock = clock
        self._managed: dict[str, ManagedProcess] = {}

    def start(
        self,
        profile_id: str,
        arguments: Mapping[str, Any] | None = None,
        *,
        replace_mode: bool = True,
    ) -> ManagedProcess:
        profile = self.registry.get(profile_id)
        existing = self._managed.get(profile.mode)
        if existing is not None and existing.process.poll() is None:
            if existing.profile.profile_id == profile_id:
                return existing
            if replace_mode:
                self.stop_mode(profile.mode)
            else:
                raise ProfileError(f"mode {profile.mode} already has a managed process")
        command = profile.build_command(arguments)
        log_dir = self.runtime_dir / "logs" / "system_manager"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{profile_id}.log"
        log_file = open(log_path, "ab", buffering=0)
        try:
            process = self._popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_file.close()
            raise
        managed = ManagedProcess(profile, process, command, log_path, self._clock(), int(process.pid))
        self._managed[profile.mode] = managed
        return managed

    def stop_mode(self, mode: str, *, grace_sec: float = 5.0) -> list[dict[str, Any]]:
        managed = self._managed.pop(str(mode).upper(), None)
        if managed is None:
            return []
        process = managed.process
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=grace_sec)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=grace_sec)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=grace_sec)
        return [managed.snapshot()]

    def stop_all(self) -> list[dict[str, Any]]:
        return [item for mode in list(self._managed) for item in self.stop_mode(mode)]

    def status(self) -> list[dict[str, Any]]:
        return [item.snapshot() for item in self._managed.values()]

    def write_status(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(json.dumps(self.status(), indent=2), encoding="utf-8")
        os.replace(temporary, target)
