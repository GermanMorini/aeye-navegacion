import copy
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose, Quaternion
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import FromLL

from navegacion_gps_interfaces.msg import NoGoPoint, NoGoZone
from navegacion_gps_interfaces.srv import GetKeepoutState, SetKeepoutZones

from .keepout_mask_utils import exponential_gradient_from_core, rasterize_polygons_core


class KeepoutManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("keepout_manager")

        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("fromll_service_fallback", "/navsat_transform/fromLL")
        self.declare_parameter("fromll_wait_timeout_s", 2.0)
        self.declare_parameter("fromll_call_retries", 4)
        self.declare_parameter("fromll_retry_delay_s", 0.15)
        self.declare_parameter("global_costmap_service", "/global_costmap/get_costmap")
        self.declare_parameter("set_zones_service", "/keepout_manager/set_zones")
        self.declare_parameter("get_state_service", "/keepout_manager/get_state")
        self.declare_parameter("mask_topic", "/keepout_filter_mask")
        self.declare_parameter("filter_info_topic", "/costmap_filter_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("zones_file", "")
        self.declare_parameter("degrade_enabled", True)
        self.declare_parameter("degrade_radius_m", 2.0)
        self.declare_parameter("degrade_edge_cost", 12)
        self.declare_parameter("degrade_min_cost", 1)
        self.declare_parameter("degrade_use_l2", True)
        self.declare_parameter("use_fixed_mask_grid", True)
        self.declare_parameter("mask_origin_x", -150.0)
        self.declare_parameter("mask_origin_y", -150.0)
        self.declare_parameter("mask_width", 3000)
        self.declare_parameter("mask_height", 3000)
        self.declare_parameter("mask_resolution", 0.1)

        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.fromll_service_fallback = str(
            self.get_parameter("fromll_service_fallback").value
        )
        self.fromll_wait_timeout_s = max(
            0.1, float(self.get_parameter("fromll_wait_timeout_s").value)
        )
        self.fromll_call_retries = max(
            1, int(self.get_parameter("fromll_call_retries").value)
        )
        self.fromll_retry_delay_s = max(
            0.0, float(self.get_parameter("fromll_retry_delay_s").value)
        )
        self.global_costmap_service = str(self.get_parameter("global_costmap_service").value)
        self.set_zones_service = str(self.get_parameter("set_zones_service").value)
        self.get_state_service = str(self.get_parameter("get_state_service").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.filter_info_topic = str(self.get_parameter("filter_info_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.zones_file = self._resolve_zones_file(str(self.get_parameter("zones_file").value))
        self.degrade_enabled = bool(self.get_parameter("degrade_enabled").value)
        self.degrade_radius_m = float(self.get_parameter("degrade_radius_m").value)
        self.degrade_edge_cost = int(self.get_parameter("degrade_edge_cost").value)
        self.degrade_min_cost = int(self.get_parameter("degrade_min_cost").value)
        self.degrade_use_l2 = bool(self.get_parameter("degrade_use_l2").value)
        self.use_fixed_mask_grid = bool(self.get_parameter("use_fixed_mask_grid").value)
        self.mask_origin_x = float(self.get_parameter("mask_origin_x").value)
        self.mask_origin_y = float(self.get_parameter("mask_origin_y").value)
        self.mask_width = int(self.get_parameter("mask_width").value)
        self.mask_height = int(self.get_parameter("mask_height").value)
        self.mask_resolution = float(self.get_parameter("mask_resolution").value)

        self._sanitize_degrade_params()
        self._sanitize_mask_grid_params()

        self._lock = threading.Lock()
        self._zones_ll: List[Dict[str, Any]] = []
        self._zones_xy: List[Dict[str, Any]] = []
        self._mask_ready = False
        self._mask_source = "none"
        self._fixed_mask_info: Optional[MapMetaData] = None
        if self.use_fixed_mask_grid:
            self._fixed_mask_info = self._build_fixed_mask_metadata()

        # Service callbacks are mutually exclusive; clients are reentrant so waiting
        # for futures inside service callbacks does not starve client responses.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._fromll_client = self.create_client(
            FromLL, self.fromll_service, callback_group=self._client_group
        )
        self._fromll_fallback_client = None
        if self.fromll_service_fallback and (
            self.fromll_service_fallback != self.fromll_service
        ):
            self._fromll_fallback_client = self.create_client(
                FromLL,
                self.fromll_service_fallback,
                callback_group=self._client_group,
            )
        self._active_fromll_name: Optional[str] = None
        self._active_fromll_client: Optional[Any] = None
        self._last_fromll_error: Optional[str] = None
        self._global_costmap_client = self.create_client(
            GetCostmap,
            self.global_costmap_service,
            callback_group=self._client_group,
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        self._mask_pub = self.create_publisher(OccupancyGrid, self.mask_topic, latched_qos)
        self._filter_info_pub = self.create_publisher(
            CostmapFilterInfo, self.filter_info_topic, latched_qos
        )

        self._set_zones_srv = self.create_service(
            SetKeepoutZones,
            self.set_zones_service,
            self._on_set_zones,
            callback_group=self._service_group,
        )
        self._get_state_srv = self.create_service(
            GetKeepoutState,
            self.get_state_service,
            self._on_get_state,
            callback_group=self._service_group,
        )

        self._publish_filter_info()
        self.load_initial_zones()
        self.get_logger().info(
            "Keepout manager ready "
            f"(set_service={self.set_zones_service}, get_service={self.get_state_service}, "
            f"mask_topic={self.mask_topic}, source={'fixed_grid' if self.use_fixed_mask_grid else 'global_costmap'})"
        )
        self.get_logger().info(
            "Callback groups configured (services=MutuallyExclusive, clients=Reentrant)"
        )

    def _resolve_zones_file(self, configured: str) -> Path:
        if configured:
            return Path(configured)
        return Path.cwd() / "config" / "no_go_zones.yaml"

    def _sanitize_degrade_params(self) -> None:
        if self.degrade_radius_m < 0.0:
            self.get_logger().warning(
                f"degrade_radius_m={self.degrade_radius_m} is invalid; forcing 0.0"
            )
            self.degrade_radius_m = 0.0

        if self.degrade_edge_cost < 1 or self.degrade_edge_cost > 99:
            self.get_logger().warning(
                f"degrade_edge_cost={self.degrade_edge_cost} is out of range [1,99]; clamping"
            )
            self.degrade_edge_cost = max(1, min(99, self.degrade_edge_cost))

        if self.degrade_min_cost < 1 or self.degrade_min_cost > 99:
            self.get_logger().warning(
                f"degrade_min_cost={self.degrade_min_cost} is out of range [1,99]; clamping"
            )
            self.degrade_min_cost = max(1, min(99, self.degrade_min_cost))

        if self.degrade_min_cost > self.degrade_edge_cost:
            self.get_logger().warning(
                "degrade_min_cost is greater than degrade_edge_cost; forcing equality"
            )
            self.degrade_min_cost = self.degrade_edge_cost

    def _sanitize_mask_grid_params(self) -> None:
        if self.mask_width <= 0:
            self.get_logger().warning(f"mask_width={self.mask_width} invalid; forcing 3000")
            self.mask_width = 3000
        if self.mask_height <= 0:
            self.get_logger().warning(f"mask_height={self.mask_height} invalid; forcing 3000")
            self.mask_height = 3000
        if self.mask_resolution <= 0.0 or not np.isfinite(self.mask_resolution):
            self.get_logger().warning(
                f"mask_resolution={self.mask_resolution} invalid; forcing 0.1"
            )
            self.mask_resolution = 0.1

    def _build_fixed_mask_metadata(self) -> MapMetaData:
        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = float(self.mask_resolution)
        info.width = int(self.mask_width)
        info.height = int(self.mask_height)

        origin = Pose()
        origin.position.x = float(self.mask_origin_x)
        origin.position.y = float(self.mask_origin_y)
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = origin
        return info

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_sec:
                return None
            time.sleep(0.01)
        return None

    def _call_from_ll(self, lat: float, lon: float) -> Optional[Tuple[float, float, float]]:
        for attempt in range(self.fromll_call_retries):
            fromll_client = self._resolve_fromll_client()
            if fromll_client is None:
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            req = FromLL.Request()
            req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            future = fromll_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=2.5)
            except Exception as exc:
                self._last_fromll_error = str(exc)
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue
            if res is None:
                self._last_fromll_error = "timeout waiting fromLL response"
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            self._last_fromll_error = None
            return (float(res.map_point.x), float(res.map_point.y), float(res.map_point.z))

        self.get_logger().warning(
            "fromLL conversion failed "
            f"(lat={lat:.8f}, lon={lon:.8f}, reason={self._last_fromll_error or 'unknown'})"
        )
        return None

    def _resolve_fromll_client(self) -> Optional[Any]:
        candidates: List[Tuple[Any, str, float]] = []
        if self._active_fromll_client is not None and self._active_fromll_name is not None:
            candidates.append((self._active_fromll_client, self._active_fromll_name, 0.05))

        candidates.append((self._fromll_client, self.fromll_service, self.fromll_wait_timeout_s))

        fallback = self._fromll_fallback_client
        if fallback is not None:
            candidates.append((fallback, self.fromll_service_fallback, self.fromll_wait_timeout_s))

        seen = set()
        for client, service_name, wait_s in candidates:
            key = (id(client), service_name)
            if key in seen:
                continue
            seen.add(key)
            if client.wait_for_service(timeout_sec=wait_s):
                self._active_fromll_client = client
                self._maybe_log_active_fromll(service_name)
                return client

        self.get_logger().warning(
            "fromLL service unavailable "
            f"(tried '{self.fromll_service}'"
            + (
                f" and '{self.fromll_service_fallback}'"
                if self._fromll_fallback_client is not None
                else ""
            )
            + ")"
        )
        self._last_fromll_error = "fromLL service unavailable"
        return None

    def _maybe_log_active_fromll(self, service_name: str) -> None:
        if self._active_fromll_name == service_name:
            return
        self._active_fromll_name = service_name
        self.get_logger().info(f"Using fromLL service: {service_name}")

    def _get_global_costmap(self) -> Optional[Any]:
        if not self._global_costmap_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f"Service {self.global_costmap_service} not available"
            )
            return None
        future = self._global_costmap_client.call_async(GetCostmap.Request())
        res = self._wait_for_future(future, timeout_sec=2.5)
        if res is None:
            self.get_logger().warning(f"Timeout calling {self.global_costmap_service}")
            return None
        return res.map

    def _resolve_mask_grid_spec(self) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if self.use_fixed_mask_grid:
            if self._fixed_mask_info is None:
                self._fixed_mask_info = self._build_fixed_mask_metadata()
            info = copy.deepcopy(self._fixed_mask_info)
            return (
                {
                    "source": "fixed_mask_grid",
                    "frame_id": self.map_frame,
                    "width": int(info.width),
                    "height": int(info.height),
                    "resolution": float(info.resolution),
                    "origin": info.origin,
                },
                None,
            )

        map_msg = self._get_global_costmap()
        if map_msg is None:
            return None, f"Service {self.global_costmap_service} failed"
        md = map_msg.metadata
        return (
            {
                "source": "global_costmap",
                "frame_id": map_msg.header.frame_id or self.map_frame,
                "width": int(md.size_x),
                "height": int(md.size_y),
                "resolution": float(md.resolution),
                "origin": md.origin,
            },
            None,
        )

    def _convert_zones_to_xy(
        self, zones: List[Dict[str, Any]]
    ) -> Tuple[List[Dict[str, Any]], List[str]]:
        out: List[Dict[str, Any]] = []
        failed_zone_ids: List[str] = []
        for zone in zones:
            zone_id = str(zone.get("id", ""))
            ztype = str(zone.get("type", "no_go"))
            enabled = bool(zone.get("enabled", True))
            polygon_ll = zone.get("polygon", [])
            if not isinstance(polygon_ll, list) or len(polygon_ll) < 3:
                self.get_logger().warning(f"Skipping zone '{zone_id}': invalid polygon")
                failed_zone_ids.append(zone_id)
                continue

            polygon_xy: List[Dict[str, float]] = []
            failed = False
            for vertex in polygon_ll:
                try:
                    lat = float(vertex["lat"])
                    lon = float(vertex["lon"])
                except Exception:
                    failed = True
                    break

                converted = self._call_from_ll(lat, lon)
                if converted is None:
                    failed = True
                    break
                x, y, _ = converted
                polygon_xy.append({"x": x, "y": y})

            if failed or len(polygon_xy) < 3:
                self.get_logger().warning(
                    f"Skipping zone '{zone_id}': fromLL conversion failed"
                )
                failed_zone_ids.append(zone_id)
                continue

            out.append(
                {
                    "id": zone_id,
                    "type": ztype,
                    "enabled": enabled,
                    "polygon_xy": polygon_xy,
                }
            )
        return out, failed_zone_ids

    def _rasterize_mask(
        self, zones_xy: List[Dict[str, Any]], grid_spec: Dict[str, Any]
    ) -> OccupancyGrid:
        width = int(grid_spec["width"])
        height = int(grid_spec["height"])
        resolution = float(grid_spec["resolution"])
        origin = grid_spec["origin"]
        origin_x = float(origin.position.x)
        origin_y = float(origin.position.y)

        core_mask, clipped_vertices, outside_zone_ids = rasterize_polygons_core(
            zones_xy=zones_xy,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        for zone_id, clipped_count in clipped_vertices.items():
            self.get_logger().warning(
                f"Zone '{zone_id}' clipped {clipped_count} vertices to mask bounds"
            )
        for zone_id in outside_zone_ids:
            self.get_logger().warning(
                f"Zone '{zone_id}' is completely outside mask bounds and was skipped"
            )

        if self.degrade_enabled and self.degrade_radius_m > 0.0:
            gradient_mask = exponential_gradient_from_core(
                core_mask=core_mask,
                resolution=resolution,
                radius_m=self.degrade_radius_m,
                edge_cost=self.degrade_edge_cost,
                min_cost=self.degrade_min_cost,
                use_l2=self.degrade_use_l2,
            )
            mask = np.maximum(gradient_mask, core_mask)
            mask[core_mask > 0] = 100
        else:
            mask = core_mask

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(grid_spec["frame_id"])
        out.info.map_load_time = out.header.stamp
        out.info.resolution = resolution
        out.info.width = width
        out.info.height = height
        out.info.origin = origin
        out.data = np.flipud(mask).reshape(-1).astype(np.int8).tolist()
        return out

    def _publish_filter_info(self) -> None:
        msg = CostmapFilterInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.type = 0
        msg.filter_mask_topic = self.mask_topic
        msg.base = 0.0
        msg.multiplier = 1.0
        self._filter_info_pub.publish(msg)

    def _save_zones_to_disk(self) -> None:
        with self._lock:
            zones = list(self._zones_ll)
        data = {"frame_id": self.map_frame, "zones": zones}
        try:
            self.zones_file.parent.mkdir(parents=True, exist_ok=True)
            with self.zones_file.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(data, handle, sort_keys=False)
        except Exception as exc:
            self.get_logger().warning(f"Failed to save zones file: {exc}")

    def _apply_zones(self, zones: List[Dict[str, Any]]) -> Tuple[bool, str, bool]:
        total_started = time.perf_counter()
        self.get_logger().info(f"SetKeepoutZones called (zones={len(zones)})")
        if self._resolve_fromll_client() is None:
            detail = self._last_fromll_error or "unknown"
            self.get_logger().error(f"SetKeepoutZones failed: fromLL unavailable ({detail})")
            return False, f"fromLL service not available ({detail})", False

        grid_spec, err = self._resolve_mask_grid_spec()
        if grid_spec is None:
            self.get_logger().error(f"SetKeepoutZones failed: grid spec error ({err})")
            return False, str(err), False

        zones_xy, failed_zone_ids = self._convert_zones_to_xy(zones)
        if zones and not zones_xy:
            detail = self._last_fromll_error or "unknown"
            self.get_logger().error(
                "SetKeepoutZones failed: no valid zones after LL->XY conversion "
                f"(zones={len(zones)}, reason={detail})"
            )
            return (
                False,
                "No valid zones were converted from LL coordinates "
                f"(check fromLL/navsat availability): {detail}",
                False,
            )
        if failed_zone_ids:
            self.get_logger().warning(
                "Some zones were skipped after LL->XY conversion: "
                + ", ".join(failed_zone_ids)
            )
        started = time.perf_counter()
        occ = self._rasterize_mask(zones_xy, grid_spec)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f"Mask rasterized in {elapsed_ms:.1f} ms (zones={len(zones_xy)}, source={grid_spec['source']})"
        )

        self._mask_pub.publish(occ)
        with self._lock:
            self._zones_ll = zones
            self._zones_xy = zones_xy
            self._mask_ready = True
            self._mask_source = str(grid_spec["source"])

        self._save_zones_to_disk()
        total_ms = (time.perf_counter() - total_started) * 1000.0
        self.get_logger().info(
            "SetKeepoutZones completed "
            f"(zones_in={len(zones)}, zones_ok={len(zones_xy)}, published=true, total_ms={total_ms:.1f})"
        )
        return True, "ok", True

    def _load_zones_from_disk(self) -> Tuple[bool, str]:
        if not self.zones_file.exists():
            self.get_logger().info(f"No zones file found at {str(self.zones_file)}")
            return True, "no zones file"
        try:
            with self.zones_file.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            zones = data.get("zones", [])
            if not isinstance(zones, list):
                return False, "zones file invalid: zones must be list"
            ok, err, _ = self._apply_zones(zones)
            return ok, err
        except Exception as exc:
            return False, f"failed reading zones file: {exc}"

    def load_initial_zones(self) -> None:
        ok, err = self._load_zones_from_disk()
        if not ok:
            self.get_logger().warning(err)

    def _zone_msg_to_dict(self, zone_msg: NoGoZone) -> Dict[str, Any]:
        polygon = []
        for p in zone_msg.polygon:
            polygon.append({"lat": float(p.lat), "lon": float(p.lon)})
        return {
            "id": str(zone_msg.id),
            "type": str(zone_msg.type),
            "enabled": bool(zone_msg.enabled),
            "polygon": polygon,
        }

    def _zone_dict_to_msg(self, zone: Dict[str, Any]) -> NoGoZone:
        msg = NoGoZone()
        msg.id = str(zone.get("id", ""))
        msg.type = str(zone.get("type", "no_go"))
        msg.enabled = bool(zone.get("enabled", True))
        points: List[NoGoPoint] = []
        for v in zone.get("polygon", []):
            try:
                lat = float(v.get("lat"))
                lon = float(v.get("lon"))
            except Exception:
                continue
            p = NoGoPoint()
            p.lat = lat
            p.lon = lon
            points.append(p)
        msg.polygon = points
        return msg

    def _on_set_zones(
        self,
        request: SetKeepoutZones.Request,
        response: SetKeepoutZones.Response,
    ) -> SetKeepoutZones.Response:
        zones = [self._zone_msg_to_dict(z) for z in request.zones]
        ok, err, published = self._apply_zones(zones)
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        response.published = bool(published)
        if not response.ok:
            self.get_logger().warning(
                f"SetKeepoutZones response failed (published={response.published}, error='{response.error}')"
            )
        return response

    def _on_get_state(
        self,
        _request: GetKeepoutState.Request,
        response: GetKeepoutState.Response,
    ) -> GetKeepoutState.Response:
        with self._lock:
            zones = list(self._zones_ll)
            mask_ready = bool(self._mask_ready)
            mask_source = str(self._mask_source)

        response.ok = True
        response.error = ""
        response.frame_id = self.map_frame
        response.mask_ready = mask_ready
        response.mask_source = mask_source
        response.zones = [self._zone_dict_to_msg(z) for z in zones]
        self.get_logger().debug(
            f"GetKeepoutState served (zones={len(zones)}, mask_ready={mask_ready}, source={mask_source})"
        )
        return response


def main() -> None:
    rclpy.init()
    node = KeepoutManagerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
