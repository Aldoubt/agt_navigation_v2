"""Optional FastAPI adapter for the console service."""

from typing import Any

from .service import WebConsoleService


def create_app(service: WebConsoleService):
    try:
        from fastapi import FastAPI, Header, HTTPException, WebSocket, WebSocketDisconnect
    except ImportError as error:
        raise RuntimeError("FastAPI is required for the Web console runtime") from error

    app = FastAPI(title="AGT 导航实验与运维控制台", version="1")

    def authorize(token: str | None) -> None:
        if service.config.token and token != service.config.token:
            raise HTTPException(status_code=401, detail="invalid console token")

    @app.get("/api/v1/overview")
    def overview(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.overview()

    @app.get("/api/v1/health")
    def health(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return dict(service.health_provider())

    @app.get("/api/v1/task-readiness")
    def readiness(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return dict(service.readiness_provider())

    @app.get("/api/v1/system/status")
    def system_status(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.mode_status()

    @app.get("/api/v1/mapping/map")
    def mapping_map(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.mapping_status()

    @app.get("/api/v1/mapping/pointcloud")
    def mapping_pointcloud(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.mapping_pointcloud_status()

    @app.get("/api/v1/mapping/session")
    def mapping_session(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.mapping_session_status()

    @app.post("/api/v1/mapping/session/prepare")
    def prepare_mapping_session(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.prepare_mapping_session(str(body.get("map_name", "")))
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/v1/mapping/finish")
    def finish_mapping(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.finish_mapping(str(body.get("action", "")), str(body.get("map_name", "")))
        except (ValueError, RuntimeError, OSError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/v1/chassis/status")
    def chassis_status(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.chassis_status()

    @app.post("/api/v1/system/mode")
    def system_mode(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.set_mode(str(body.get("profile", "")), body.get("arguments", {}))
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/v1/system/stop")
    def system_stop(body: dict[str, Any] | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.stop_mode((body or {}).get("mode"))
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/v1/maps")
    def maps(map_id: str | None = None, state: str | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.maps(map_id=map_id, state=state)

    @app.post("/api/v1/maps/{version_id}/validate")
    def validate_map(version_id: str, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.validate_map(version_id)
        except (KeyError, RuntimeError) as error:
            raise HTTPException(status_code=404, detail=str(error)) from error

    @app.post("/api/v1/maps/{version_id}/activate")
    def activate_map(version_id: str, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.activate_map(version_id)
        except (KeyError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/v1/maps/import")
    def import_map(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.import_map(body)
        except (ValueError, RuntimeError, OSError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/v1/maps/{version_id}/{action}")
    def map_action(version_id: str, action: str, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.map_action(version_id, action)
        except (KeyError, ValueError, RuntimeError, OSError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/v1/experiments")
    def experiments(state: str | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.experiments(state=state)

    @app.get("/api/v1/bags")
    def bags(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.bags()

    @app.post("/api/v1/bags/{action}")
    def bag_action(action: str, body: dict[str, Any] | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.bag_action(action, body or {})
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/v1/experiments")
    def create_experiment(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.create_experiment(body)
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/v1/experiments/{experiment_id}/{action}")
    def experiment_action(experiment_id: str, action: str, body: dict[str, Any] | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.experiment_action(experiment_id, action, body)
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/v1/localization/mode")
    def localization_mode(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.localization_mode(str(body.get("mode", "")))
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/v1/localization/relocalize")
    def relocalize(body: dict[str, Any] | None = None, x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.relocalize(body or {})
        except (ValueError, RuntimeError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.get("/api/v1/runtime")
    def runtime_status(x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        return service.runtime_status()

    @app.post("/api/v1/runtime/backend")
    def runtime_backend(body: dict[str, Any], x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.set_backend(str(body.get("backend", "")))
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/v1/logs")
    def logs(component: str = "system_manager", x_agt_token: str | None = Header(default=None)):
        authorize(x_agt_token)
        try:
            return service.logs(component)
        except ValueError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.websocket("/ws")
    async def websocket(websocket: WebSocket):
        import asyncio

        websocket_token = websocket.headers.get("x-agt-token") or websocket.query_params.get("token")
        if service.config.token and websocket_token != service.config.token:
            await websocket.close(code=4401)
            return
        await websocket.accept()
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()

        def enqueue(event):
            loop.call_soon_threadsafe(queue.put_nowait, dict(event))

        service.subscribe(enqueue)
        try:
            await websocket.send_json(service.overview())
            while True:
                await websocket.send_json(await queue.get())
        except WebSocketDisconnect:
            return
        finally:
            service.unsubscribe(enqueue)

    return app
