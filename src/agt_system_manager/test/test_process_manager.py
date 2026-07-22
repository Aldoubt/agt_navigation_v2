import pytest

from agt_system_manager.process_manager import ProfileError, ProfileRegistry


def test_registry_rejects_shell_string_and_unknown_profile():
    with pytest.raises(ProfileError):
        ProfileRegistry({"bad": {"command": "ros2 launch something"}})
    registry = ProfileRegistry({"ok": {"mode": "MAPPING", "command": ["ros2", "launch", "x"]}})
    with pytest.raises(ProfileError):
        registry.get("unknown")


def test_profile_only_accepts_declared_argument_keys_and_no_newlines():
    registry = ProfileRegistry(
        {"ok": {"mode": "MAPPING", "command": ["ros2", "launch", "x"], "allowed_argument_keys": ["map"]}}
    )
    profile = registry.get("ok")
    assert profile.build_command({"map": "/tmp/map.yaml"})[-1] == "map:=/tmp/map.yaml"
    with pytest.raises(ProfileError):
        profile.build_command({"shell": "rm -rf /"})
    with pytest.raises(ProfileError):
        profile.build_command({"map": "bad\ncommand"})


def test_profile_registry_allows_only_configured_executables():
    with pytest.raises(ProfileError):
        ProfileRegistry({"bad": {"command": ["bash", "-c", "echo unsafe"]}})
