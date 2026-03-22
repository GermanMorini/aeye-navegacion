from __future__ import annotations

from copy import deepcopy
import math
import random
from pathlib import Path
from typing import Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2
import yaml


DEFAULT_IMU_ORIENTATION_VARIANCE = 0.01
DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE = 0.01
DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE = 0.1
DEFAULT_GPS_HORIZONTAL_VARIANCE = 2.5
DEFAULT_GPS_VERTICAL_VARIANCE = 4.0
DEFAULT_GPS_PROFILE = "ideal"
GLOBAL_LOCALIZATION_CONFIG_FILENAME = "global_localization_v2.yaml"
GPS_PROFILE_SETTINGS: Dict[str, Dict[str, object]] = {
    "ideal": {
        "realistic": False,
        "publish_rate_hz": 0.0,
        "horizontal_noise_stddev_m": 0.0,
        "vertical_noise_stddev_m": 0.0,
        "publish_jitter_stddev_s": 0.0,
        "bias_walk_stddev_m_per_sqrt_s": 0.0,
        "covariance_horizontal_stddev_m": 0.02,
        "covariance_vertical_stddev_m": 0.05,
    },
    "m8n": {
        "realistic": True,
        "publish_rate_hz": 5.0,
        "horizontal_noise_stddev_m": 1.8,
        "vertical_noise_stddev_m": 3.5,
        "publish_jitter_stddev_s": 0.08,
        "bias_walk_stddev_m_per_sqrt_s": 0.04,
        "covariance_horizontal_stddev_m": 1.8,
        "covariance_vertical_stddev_m": 3.5,
    },
    "f9p_rtk": {
        "realistic": True,
        "publish_rate_hz": 8.0,
        "horizontal_noise_stddev_m": 0.15,
        "vertical_noise_stddev_m": 0.25,
        "publish_jitter_stddev_s": 0.02,
        "bias_walk_stddev_m_per_sqrt_s": 0.005,
        "covariance_horizontal_stddev_m": 0.15,
        "covariance_vertical_stddev_m": 0.25,
    },
}


def _covariance_is_zero(values) -> bool:
    return all(abs(float(value)) <= 1.0e-12 for value in values)


def _stamp_to_nanoseconds(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _offset_geodetic_fix(
    latitude_deg: float,
    longitude_deg: float,
    north_m: float,
    east_m: float,
) -> Tuple[float, float]:
    meters_per_deg_lat = 111_320.0
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(latitude_deg)))))
    meters_per_deg_lon = meters_per_deg_lat * cos_lat
    return (
        float(latitude_deg) + float(north_m) / meters_per_deg_lat,
        float(longitude_deg) + float(east_m) / meters_per_deg_lon,
    )


def _resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.parent.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def load_navsat_datum_from_file(path: str) -> Tuple[float, float, float]:
    with open(path, "r", encoding="utf-8") as file_handle:
        data = yaml.safe_load(file_handle) or {}

    try:
        datum = data["navsat_transform"]["ros__parameters"]["datum"]
    except KeyError as exc:
        raise RuntimeError(f"Missing navsat_transform datum in {path}") from exc

    if not isinstance(datum, list) or len(datum) != 3:
        raise RuntimeError(f"Invalid datum in {path}: expected [lat, lon, alt]")

    lat, lon, alt = (float(value) for value in datum)
    if not (-90.0 <= lat <= 90.0):
        raise RuntimeError(f"Invalid latitude in datum from {path}: {lat}")
    if not (-180.0 <= lon <= 180.0):
        raise RuntimeError(f"Invalid longitude in datum from {path}: {lon}")
    return lat, lon, alt


def load_navsat_datum() -> Tuple[float, float, float]:
    package_share_dir = get_package_share_directory("navegacion_gps")
    config_path = _resolve_config_file_path(
        package_share_dir, GLOBAL_LOCALIZATION_CONFIG_FILENAME
    )
    return load_navsat_datum_from_file(config_path)


def build_synthetic_navsat_fix_from_odom(
    odom_msg: Odometry,
    datum: Tuple[float, float, float],
    *,
    frame_id: str,
) -> NavSatFix:
    datum_lat, datum_lon, datum_alt = datum
    east_m = float(odom_msg.pose.pose.position.x)
    north_m = float(odom_msg.pose.pose.position.y)
    altitude = float(datum_alt) + float(odom_msg.pose.pose.position.z)
    latitude, longitude = _offset_geodetic_fix(
        datum_lat,
        datum_lon,
        north_m,
        east_m,
    )

    msg = NavSatFix()
    msg.header = deepcopy(odom_msg.header)
    msg.header.frame_id = frame_id
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = latitude
    msg.longitude = longitude
    msg.altitude = altitude
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg


def resolve_gps_profile(name: str) -> Dict[str, object]:
    profile_name = str(name).strip().lower() or DEFAULT_GPS_PROFILE
    try:
        settings = GPS_PROFILE_SETTINGS[profile_name]
    except KeyError as exc:
        valid_values = ", ".join(sorted(GPS_PROFILE_SETTINGS))
        raise ValueError(
            f"Unsupported gps_profile '{name}'. Valid values: {valid_values}"
        ) from exc
    return {"name": profile_name, **settings}


class GpsNoiseModel:
    def __init__(self, profile_name: str, random_seed: int = 0) -> None:
        settings = resolve_gps_profile(profile_name)
        self.profile_name = settings["name"]
        self.realistic = bool(settings["realistic"])
        self.publish_rate_hz = float(settings["publish_rate_hz"])
        self.horizontal_noise_stddev_m = float(settings["horizontal_noise_stddev_m"])
        self.vertical_noise_stddev_m = float(settings["vertical_noise_stddev_m"])
        self.publish_jitter_stddev_s = float(settings["publish_jitter_stddev_s"])
        self.bias_walk_stddev_m_per_sqrt_s = float(
            settings["bias_walk_stddev_m_per_sqrt_s"]
        )
        self.covariance_horizontal_stddev_m = float(
            settings["covariance_horizontal_stddev_m"]
        )
        self.covariance_vertical_stddev_m = float(
            settings["covariance_vertical_stddev_m"]
        )
        self._random = random.Random(None if int(random_seed) == 0 else int(random_seed))
        self._next_publish_time_ns = 0
        self._last_publish_time_ns = None
        self._bias_north_m = 0.0
        self._bias_east_m = 0.0
        self._bias_alt_m = 0.0

    def apply_profile_covariance(self, msg: NavSatFix) -> bool:
        if (
            self.covariance_horizontal_stddev_m <= 0.0
            and self.covariance_vertical_stddev_m <= 0.0
        ):
            return False
        horizontal_variance = self.covariance_horizontal_stddev_m**2
        vertical_variance = self.covariance_vertical_stddev_m**2
        msg.position_covariance = [
            horizontal_variance,
            0.0,
            0.0,
            0.0,
            horizontal_variance,
            0.0,
            0.0,
            0.0,
            vertical_variance,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return True

    def _advance_bias(self, dt_s: float) -> None:
        if self.bias_walk_stddev_m_per_sqrt_s <= 0.0:
            return
        sigma = self.bias_walk_stddev_m_per_sqrt_s * math.sqrt(max(0.0, dt_s))
        self._bias_north_m += self._random.gauss(0.0, sigma)
        self._bias_east_m += self._random.gauss(0.0, sigma)
        self._bias_alt_m += self._random.gauss(0.0, sigma)

    def _schedule_next_publish(self, reference_time_ns: int) -> None:
        if self.publish_rate_hz <= 0.0:
            self._next_publish_time_ns = reference_time_ns
            return
        base_period_s = 1.0 / self.publish_rate_hz
        jitter_s = self._random.gauss(0.0, self.publish_jitter_stddev_s)
        period_s = max(0.02, base_period_s + jitter_s)
        self._next_publish_time_ns = reference_time_ns + int(period_s * 1_000_000_000)

    def transform(self, msg: NavSatFix, reference_time_ns: int) -> Optional[NavSatFix]:
        if not self.realistic:
            return deepcopy(msg)

        if reference_time_ns < self._next_publish_time_ns:
            return None

        if self._last_publish_time_ns is None:
            dt_s = 0.0
        else:
            dt_s = max(
                0.0,
                (reference_time_ns - self._last_publish_time_ns) / 1_000_000_000.0,
            )
        self._last_publish_time_ns = reference_time_ns
        self._advance_bias(dt_s)

        out = deepcopy(msg)
        north_m = self._bias_north_m + self._random.gauss(
            0.0, self.horizontal_noise_stddev_m
        )
        east_m = self._bias_east_m + self._random.gauss(
            0.0, self.horizontal_noise_stddev_m
        )
        out.latitude, out.longitude = _offset_geodetic_fix(
            out.latitude,
            out.longitude,
            north_m,
            east_m,
        )
        out.altitude = (
            float(out.altitude)
            + self._bias_alt_m
            + self._random.gauss(0.0, self.vertical_noise_stddev_m)
        )
        self.apply_profile_covariance(out)
        self._schedule_next_publish(reference_time_ns)
        return out


class SimSensorNormalizerV2Node(Node):
    def __init__(self) -> None:
        super().__init__("sim_sensor_normalizer_v2")

        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("imu_out_topic", "/imu/data")
        self.declare_parameter("gps_in_topic", "/gps/fix_raw")
        self.declare_parameter("gps_out_topic", "/gps/fix")
        self.declare_parameter("lidar_in_topic", "/scan_3d_raw")
        self.declare_parameter("lidar_out_topic", "/scan_3d")
        self.declare_parameter("odom_in_topic", "/odom_raw")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("lidar_frame_id", "lidar_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_link_frame_id", "base_footprint")
        self.declare_parameter("gps_profile", DEFAULT_GPS_PROFILE)
        self.declare_parameter("gps_random_seed", 0)

        imu_in_topic = str(self.get_parameter("imu_in_topic").value)
        imu_out_topic = str(self.get_parameter("imu_out_topic").value)
        gps_in_topic = str(self.get_parameter("gps_in_topic").value)
        gps_out_topic = str(self.get_parameter("gps_out_topic").value)
        lidar_in_topic = str(self.get_parameter("lidar_in_topic").value)
        lidar_out_topic = str(self.get_parameter("lidar_out_topic").value)
        odom_in_topic = str(self.get_parameter("odom_in_topic").value)
        odom_out_topic = str(self.get_parameter("odom_out_topic").value)

        self._imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self._gps_frame_id = str(self.get_parameter("gps_frame_id").value)
        self._lidar_frame_id = str(self.get_parameter("lidar_frame_id").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_link_frame_id = str(self.get_parameter("base_link_frame_id").value)
        gps_profile = str(self.get_parameter("gps_profile").value)
        gps_random_seed = int(self.get_parameter("gps_random_seed").value)
        self._gps_model = GpsNoiseModel(gps_profile, gps_random_seed)
        self._gps_datum = load_navsat_datum()
        self._last_raw_gps_fix: Optional[NavSatFix] = None
        self._last_local_odom: Optional[Odometry] = None

        self._imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self._gps_pub = self.create_publisher(NavSatFix, gps_out_topic, 10)
        self._lidar_pub = self.create_publisher(PointCloud2, lidar_out_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)

        self.create_subscription(Imu, imu_in_topic, self._on_imu, 10)
        # Keep the bridged raw GPS topic available for observability. In sim_global_v2
        # the ideal GPS profile is anchored to /odometry/local to avoid amplifying
        # small raw-vs-local discrepancies inside the global EKF.
        self.create_subscription(NavSatFix, gps_in_topic, self._on_gps_raw, 10)
        self.create_subscription(PointCloud2, lidar_in_topic, self._on_lidar, 10)
        self.create_subscription(Odometry, odom_in_topic, self._on_odom, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_local_odom, 10)

        self.get_logger().info(
            "sim_sensor_normalizer_v2 ready "
            f"({imu_in_topic},{gps_in_topic},{lidar_in_topic},{odom_in_topic},"
            f" gps_profile={self._gps_model.profile_name},"
            f" datum=({self._gps_datum[0]:.7f},{self._gps_datum[1]:.7f},{self._gps_datum[2]:.3f}))"
        )

    def _get_reference_time_ns(self, msg) -> int:
        stamp_ns = _stamp_to_nanoseconds(msg)
        if stamp_ns > 0:
            return stamp_ns
        return int(self.get_clock().now().nanoseconds)

    def _on_imu(self, msg: Imu) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._imu_frame_id
        if _covariance_is_zero(out.orientation_covariance):
            out.orientation_covariance = [
                DEFAULT_IMU_ORIENTATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ORIENTATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ORIENTATION_VARIANCE,
            ]
        if _covariance_is_zero(out.angular_velocity_covariance):
            out.angular_velocity_covariance = [
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE,
            ]
        if _covariance_is_zero(out.linear_acceleration_covariance):
            out.linear_acceleration_covariance = [
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE,
            ]
        self._imu_pub.publish(out)

    def _on_gps_raw(self, msg: NavSatFix) -> None:
        self._last_raw_gps_fix = deepcopy(msg)

    def _on_lidar(self, msg: PointCloud2) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._lidar_frame_id
        self._lidar_pub.publish(out)

    def _on_local_odom(self, msg: Odometry) -> None:
        self._last_local_odom = deepcopy(msg)

    def _select_gps_reference_odom(self, msg: Odometry) -> Odometry:
        if self._gps_model.profile_name != "ideal":
            return msg
        if self._last_local_odom is not None:
            # The ideal profile is an integration baseline, not a simulator-truth
            # benchmark. Using /odometry/local removes a small but systematic
            # mismatch against /odom_raw that the global EKF was amplifying.
            return self._last_local_odom
        return msg

    def _publish_synthetic_gps_from_odom(self, msg: Odometry) -> None:
        reference_odom = self._select_gps_reference_odom(msg)
        base_fix = build_synthetic_navsat_fix_from_odom(
            reference_odom, self._gps_datum, frame_id=self._gps_frame_id
        )
        out = self._gps_model.transform(
            base_fix, self._get_reference_time_ns(reference_odom)
        )
        if out is None:
            return

        profile_covariance_applied = self._gps_model.apply_profile_covariance(out)
        if (not profile_covariance_applied) and _covariance_is_zero(out.position_covariance):
            out.position_covariance = [
                DEFAULT_GPS_HORIZONTAL_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_GPS_HORIZONTAL_VARIANCE,
                0.0,
                0.0,
                0.0,
                DEFAULT_GPS_VERTICAL_VARIANCE,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._odom_frame_id
        out.child_frame_id = self._base_link_frame_id
        self._odom_pub.publish(out)
        self._publish_synthetic_gps_from_odom(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSensorNormalizerV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
