#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

from agt_interfaces.msg import MapVersionSummary
from agt_interfaces.srv import ActivateMapVersion, ListMapVersions, ManageMapVersion
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from agt_map_manager.facade import (
    MapBusinessFacade, MapRequestError, STATE_VALUES, resolve_assets,
)
from agt_map_manager.registry import MapRegistry


class MapManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("agt_map_manager")
        runtime_dir = Path(
            str(self.declare_parameter("runtime_dir", "runtime").value)
        ).expanduser()
        maps_value = str(self.declare_parameter("maps_root", "").value).strip()
        experiments_value = str(
            self.declare_parameter("experiments_root", "").value
        ).strip()
        maps_root = Path(maps_value).expanduser() if maps_value else runtime_dir / "maps"
        experiments_root = (
            Path(experiments_value).expanduser()
            if experiments_value
            else runtime_dir / "experiments"
        )
        database_existed = (maps_root / "map_registry.sqlite3").is_file()
        self._registry = MapRegistry(maps_root)
        if not database_existed:
            self._registry.rebuild_index()
        self._facade = MapBusinessFacade(self._registry, experiments_root)
        period = float(self.declare_parameter("publish_period_s", 1.0).value)
        if period <= 0.0:
            raise ValueError("publish_period_s must be positive")
        group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._active_publisher = self.create_publisher(
            MapVersionSummary, "/agt/maps/active", latched
        )
        self.create_service(
            ListMapVersions, "/agt/maps/list", self._list, callback_group=group
        )
        self.create_service(
            ManageMapVersion, "/agt/maps/manage", self._manage, callback_group=group
        )
        self.create_service(
            ActivateMapVersion,
            "/agt/maps/activate",
            self._activate,
            callback_group=group,
        )
        self._timer = self.create_timer(
            period, self._publish_active, callback_group=group
        )
        self._publish_active()

    def _summary(self, row) -> MapVersionSummary:
        message = MapVersionSummary()
        message.header.stamp = self.get_clock().now().to_msg()
        message.map_id = str(row.get("map_id", ""))
        message.map_version_id = str(row.get("version_id", ""))
        message.parent_map_version_id = str(row.get("parent_version_id") or "")
        message.active = bool(row.get("active"))
        message.pinned = bool(row.get("pinned"))
        message.deleted = bool(row.get("deleted"))
        state = (
            "DELETED"
            if message.deleted
            else str(row.get("state", "UNKNOWN")).upper()
        )
        message.state = int(STATE_VALUES.get(state, MapVersionSummary.STATE_UNKNOWN))
        message.storage_bytes = int(row.get("storage_bytes", 0))
        message.created_at = str(row.get("created_at", ""))
        try:
            validation = self._facade.validation(row)
            assets = resolve_assets(row)
            message.valid = validation.valid
            message.map_hash = validation.map_hash
            message.manifest_sha256 = assets["manifest_sha256"]
            message.navigation_yaml_sha256 = validation.asset_hashes.get(
                "navigation_yaml", ""
            )
            message.navigation_image_sha256 = validation.asset_hashes.get(
                "navigation_pgm", ""
            )
            message.localization_pcd_sha256 = validation.asset_hashes.get(
                "localization_pcd", ""
            )
            message.navigation_yaml = assets["navigation_yaml"]
            message.localization_pcd = assets["localization_pcd"]
            message.processing_record = assets["processing_record"]
            message.tasks_directory = assets["tasks_directory"]
            message.validation_errors = list(validation.errors)
            message.validation_warnings = list(validation.warnings)
        except Exception as exc:
            message.valid = False
            message.validation_errors = [str(exc)]
        return message

    def _unknown(self) -> MapVersionSummary:
        message = MapVersionSummary()
        message.header.stamp = self.get_clock().now().to_msg()
        return message

    def _publish_active(self) -> None:
        row = self._facade.active_row()
        self._active_publisher.publish(
            self._summary(row) if row is not None else self._unknown()
        )

    def _list(self, request, response):
        try:
            rows = self._facade.list_rows(
                map_id=str(request.map_id),
                state=int(request.state),
                include_deleted=bool(request.include_deleted),
            )
            response.versions = [self._summary(row) for row in rows]
            response.success = True
            response.error_code = ListMapVersions.Response.ERROR_NONE
            response.message = f"listed {len(rows)} map versions"
        except ValueError as exc:
            response.success = False
            response.error_code = ListMapVersions.Response.ERROR_INVALID_REQUEST
            response.message = str(exc)
        except Exception as exc:
            response.success = False
            response.error_code = ListMapVersions.Response.ERROR_INTERNAL
            response.message = str(exc)
        return response

    def _manage(self, request, response):
        try:
            row = self._facade.manage(
                int(request.operation),
                str(request.map_version_id),
                bool(request.confirm_destructive),
                import_values={
                    "map_id": request.map_id,
                    "candidate_map_yaml": request.candidate_map_yaml,
                    "localization_pcd": request.localization_pcd,
                    "processing_record": request.processing_record,
                    "platform_profile": request.platform_profile,
                    "parent_map_version_id": request.parent_map_version_id,
                },
            )
            response.version = self._summary(row) if row is not None else self._unknown()
            response.success = True
            response.error_code = ManageMapVersion.Response.ERROR_NONE
            response.message = "map operation completed"
            self._publish_active()
        except KeyError:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_NOT_FOUND
            response.message = "map version was not found"
        except PermissionError as exc:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_CONFIRMATION_REQUIRED
            response.message = str(exc)
        except MapRequestError as exc:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_INVALID_REQUEST
            response.message = str(exc)
        except ValueError as exc:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_CONFLICT
            response.message = str(exc)
        except RuntimeError as exc:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_VALIDATION_FAILED
            response.message = str(exc)
        except Exception as exc:
            response.success = False
            response.error_code = ManageMapVersion.Response.ERROR_INTERNAL
            response.message = str(exc)
        return response

    @staticmethod
    def _state_name(value: int) -> str:
        return {
            MapVersionSummary.STATE_DRAFT: "DRAFT",
            MapVersionSummary.STATE_PROCESSING: "PROCESSING",
            MapVersionSummary.STATE_READY: "READY",
            MapVersionSummary.STATE_INVALID: "INVALID",
            MapVersionSummary.STATE_ARCHIVED: "ARCHIVED",
            MapVersionSummary.STATE_DELETED: "DELETED",
        }.get(int(value), "UNKNOWN")

    def _activate(self, request, response):
        requested_map_id = str(request.map_id).strip()
        requested_version_id = str(request.map_version_id).strip()
        if not requested_map_id or not requested_version_id:
            response.success = False
            response.error_code = ActivateMapVersion.Response.ERROR_INVALID_REQUEST
            response.map_id = requested_map_id
            response.map_version_id = requested_version_id
            response.state = "UNKNOWN"
            response.blocker_codes = ["NO_ACTIVE_MAP"]
            response.blocker_messages = ["map_id and map_version_id are required"]
            response.operator_messages = ["请选择要激活的 READY 地图版本。"]
            response.technical_messages = ["ActivateMapVersion requires map_id and map_version_id"]
            return response
        try:
            rows = self._facade.list_rows(map_id=requested_map_id, include_deleted=False)
            row = next(
                (item for item in rows if str(item.get("version_id", "")) == requested_version_id),
                None,
            )
            if row is None:
                response.success = False
                response.error_code = ActivateMapVersion.Response.ERROR_NOT_FOUND
                response.map_id = requested_map_id
                response.map_version_id = requested_version_id
                response.state = "UNKNOWN"
                response.blocker_codes = ["NO_ACTIVE_MAP"]
                response.blocker_messages = ["map version was not found"]
                response.operator_messages = ["未找到该地图版本。"]
                response.technical_messages = ["requested map_id/map_version_id is not registered"]
                return response
            if str(row.get("state", "")).upper() != "READY":
                summary = self._summary(row)
                response.success = False
                response.error_code = ActivateMapVersion.Response.ERROR_VALIDATION_FAILED
                response.map_id = requested_map_id
                response.map_version_id = requested_version_id
                response.state = self._state_name(summary.state)
                response.version = summary
                response.blocker_codes = ["MAP_NOT_READY"]
                response.blocker_messages = ["only READY map versions may be activated"]
                response.operator_messages = ["地图版本尚未 READY，不能用于导航。"]
                response.technical_messages = ["requested map version state is not READY"]
                return response
            result = self._registry.activate(requested_version_id)
            row = self._registry._row(requested_version_id)
            summary = self._summary(row)
            response.version = summary
            response.map_id = summary.map_id
            response.map_version_id = summary.map_version_id
            response.state = self._state_name(summary.state)
            response.success = bool(result.valid)
            response.error_code = (
                ActivateMapVersion.Response.ERROR_NONE
                if result.valid
                else ActivateMapVersion.Response.ERROR_VALIDATION_FAILED
            )
            if not result.valid:
                response.blocker_codes = ["MAP_NOT_READY"]
                response.blocker_messages = list(result.errors)
                response.operator_messages = ["地图版本校验未通过，不能激活。"]
                response.technical_messages = list(result.errors)
            self._publish_active()
        except Exception as exc:
            response.success = False
            response.error_code = ActivateMapVersion.Response.ERROR_INTERNAL
            response.map_id = requested_map_id
            response.map_version_id = requested_version_id
            response.state = "UNKNOWN"
            response.blocker_codes = ["MAP_NOT_READY"]
            response.blocker_messages = [str(exc)]
            response.operator_messages = ["地图激活失败，请查看诊断。"]
            response.technical_messages = [str(exc)]
        return response

    def destroy_node(self):
        self._timer.cancel()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapManagerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
