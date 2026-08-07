#!/usr/bin/env python3

import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from agt_web_console.app import create_app
from agt_web_console.instance_lock import WebConsoleInstanceLock
from agt_web_console.offline_backend import OfflineConsoleBackend
from agt_web_console.ros_bridge import RosConsoleBridge
from agt_web_console.service import WebConsoleConfig, WebConsoleService


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Run the local AGT Web console")
    parser.add_argument("--config", required=True)
    parser.add_argument("--profiles-file", required=False)
    parser.add_argument("--backend", choices=("ros", "offline"), required=False)
    parsed = parser.parse_args(args)
    with open(parsed.config, "r", encoding="utf-8") as stream:
        config_data = yaml.safe_load(stream) or {}
    config = WebConsoleConfig(
        host=str(config_data.get("host", "127.0.0.1")),
        port=int(config_data.get("port", 8080)),
        token=str(config_data.get("token", "")),
        runtime_dir=str(config_data.get("runtime_dir", "runtime")),
        backend=str(parsed.backend or config_data.get("backend", "ros")),
        can_interface=str(config_data.get("can_interface", "can0")),
    )
    config.validate()
    instance_lock = WebConsoleInstanceLock(config.runtime_dir)
    try:
        instance_lock.acquire()
    except RuntimeError as error:
        parser.error(str(error))
    profiles_file = Path(parsed.profiles_file).expanduser() if parsed.profiles_file else Path(get_package_share_directory("agt_system_manager")) / "config" / "mode_profiles.yaml"
    import rclpy
    rclpy.init()
    ros_controller = RosConsoleBridge(runtime_dir=config.runtime_dir, can_interface=config.can_interface)
    with open(profiles_file, "r", encoding="utf-8") as stream:
        profiles = (yaml.safe_load(stream) or {}).get("profiles", {})
    offline_controller = OfflineConsoleBackend(profiles, runtime_dir=config.runtime_dir)
    service = WebConsoleService(
        config,
        health_provider=ros_controller.health,
        readiness_provider=ros_controller.readiness,
        mapping_provider=ros_controller.mapping_status,
        mapping_pointcloud_provider=ros_controller.mapping_pointcloud_status,
        chassis_provider=ros_controller.chassis_status,
        mapping_session_controller=ros_controller,
        mode_controller=ros_controller,
        business_controller=ros_controller,
        robot_state_provider=ros_controller.robot_state,
        mission_provider=ros_controller.mission_status,
        localization_controller=ros_controller,
        backends={
            "ros": {
                "health_provider": ros_controller.health,
                "readiness_provider": ros_controller.readiness,
                "mapping_provider": ros_controller.mapping_status,
                "mapping_pointcloud_provider": ros_controller.mapping_pointcloud_status,
                "chassis_provider": ros_controller.chassis_status,
                "mapping_session_controller": ros_controller,
                "mode_controller": ros_controller,
                "business_controller": ros_controller,
                "robot_state_provider": ros_controller.robot_state,
                "mission_provider": ros_controller.mission_status,
                "localization_controller": ros_controller,
            },
            "offline": {
                "health_provider": offline_controller.health,
                "readiness_provider": offline_controller.readiness,
                "mode_controller": offline_controller,
                "business_controller": None,
                "robot_state_provider": offline_controller.robot_state,
                "mission_provider": offline_controller.mission_status,
                "localization_controller": offline_controller,
                "mapping_provider": offline_controller.mapping_status,
                "mapping_pointcloud_provider": offline_controller.mapping_pointcloud_status,
                "chassis_provider": lambda: {"available": False, "message": "离线模式不连接 CAN 或底盘"},
                "mapping_session_controller": None,
            },
        },
    )
    ros_controller.add_status_listener(lambda event: service.publish_backend("ros", event))
    offline_controller.add_status_listener(lambda event: service.publish_backend("offline", event))
    app = create_app(service)
    try:
        from fastapi import HTTPException
        from fastapi.responses import FileResponse
        import uvicorn
    except ImportError as error:
        raise RuntimeError("install FastAPI, Starlette, and Uvicorn to run the Web console") from error

    static_root = (Path(get_package_share_directory("agt_web_console")) / "static").resolve()

    @app.get("/", include_in_schema=False)
    def static_index():
        index_path = static_root / "index.html"
        if not index_path.is_file():
            raise HTTPException(status_code=404, detail="Web console index is not installed")
        return FileResponse(index_path)

    @app.get("/{asset_path:path}", include_in_schema=False)
    def static_asset(asset_path: str):
        relative_parts = Path(asset_path).parts
        if Path(asset_path).is_absolute() or ".." in relative_parts:
            raise HTTPException(status_code=404, detail="static asset not found")
        requested_path = static_root / Path(*relative_parts)
        if not requested_path.is_file():
            raise HTTPException(status_code=404, detail="static asset not found")
        return FileResponse(requested_path)

    try:
        uvicorn.run(app, host=config.host, port=config.port)
    finally:
        offline_controller.close()
        ros_controller.close()
        if rclpy.ok():
            rclpy.shutdown()
        instance_lock.release()


if __name__ == "__main__":
    main()
