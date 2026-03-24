from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

EARTH_RADIUS_M = 6_378_137.0
GRAVITY_MPS2 = 9.80665


def clamp_non_negative(value: float, default: float = 0.0) -> float:
    if not math.isfinite(value):
        return float(default)
    return max(float(default), float(value))


def clamp_unit(value: float, default: float = 0.5) -> float:
    if not math.isfinite(value):
        return float(default)
    return min(1.0, max(0.0, float(value)))


def wrap_angle(angle_rad: float) -> float:
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    return angle_rad


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def yaw_from_quaternion(q: Quaternion) -> float:
    w = float(q.w)
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        return 0.0
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z))
    return wrap_angle(math.atan2(siny_cosp, cosy_cosp))


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    half = 0.5 * float(yaw_rad)
    quat = Quaternion()
    quat.w = math.cos(half)
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half)
    return quat


def normalize_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(
        float(x) * float(x)
        + float(y) * float(y)
        + float(z) * float(z)
        + float(w) * float(w)
    )
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return (
        float(x) / norm,
        float(y) / norm,
        float(z) / norm,
        float(w) / norm,
    )


def rotate_vector_body_to_world(
    vx: float,
    vy: float,
    vz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Tuple[float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(qx, qy, qz, qw)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    out_x = (r00 * float(vx)) + (r01 * float(vy)) + (r02 * float(vz))
    out_y = (r10 * float(vx)) + (r11 * float(vy)) + (r12 * float(vz))
    out_z = (r20 * float(vx)) + (r21 * float(vy)) + (r22 * float(vz))
    return out_x, out_y, out_z


def local_xy_from_gps(
    lat_deg: float,
    lon_deg: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
) -> Tuple[float, float]:
    lat_rad = math.radians(float(lat_deg))
    lon_rad = math.radians(float(lon_deg))
    lat0_rad = math.radians(float(origin_lat_deg))
    lon0_rad = math.radians(float(origin_lon_deg))

    dlat = lat_rad - lat0_rad
    dlon = lon_rad - lon0_rad
    x_m = float(dlon * math.cos(lat0_rad) * EARTH_RADIUS_M)
    y_m = float(dlat * EARTH_RADIUS_M)
    return x_m, y_m


def gps_velocity_from_positions(
    prev_x_m: float,
    prev_y_m: float,
    prev_t_s: float,
    cur_x_m: float,
    cur_y_m: float,
    cur_t_s: float,
    min_dt_s: float,
    max_dt_s: float,
) -> Optional[Tuple[float, float]]:
    dt_s = float(cur_t_s) - float(prev_t_s)
    if (not math.isfinite(dt_s)) or (dt_s < float(min_dt_s)) or (dt_s > float(max_dt_s)):
        return None
    vx = (float(cur_x_m) - float(prev_x_m)) / dt_s
    vy = (float(cur_y_m) - float(prev_y_m)) / dt_s
    if (not math.isfinite(vx)) or (not math.isfinite(vy)):
        return None
    return vx, vy


def low_pass_filter(
    prev_value: Optional[float],
    measurement: float,
    alpha: float,
) -> float:
    if not math.isfinite(float(measurement)):
        return float(prev_value) if prev_value is not None else 0.0
    if prev_value is None or not math.isfinite(float(prev_value)):
        return float(measurement)
    a = clamp_unit(alpha, default=0.5)
    return (a * float(measurement)) + ((1.0 - a) * float(prev_value))


def world_velocity_to_body(
    vx_world_mps: float,
    vy_world_mps: float,
    yaw_rad: float,
) -> Tuple[float, float]:
    cy = math.cos(float(yaw_rad))
    sy = math.sin(float(yaw_rad))
    vx_body = (cy * float(vx_world_mps)) + (sy * float(vy_world_mps))
    vy_body = (-sy * float(vx_world_mps)) + (cy * float(vy_world_mps))
    return vx_body, vy_body


def normalized_fusion_weights(
    gps_weight: float,
    imu_weight: float,
) -> Tuple[float, float]:
    gps = max(0.0, float(gps_weight)) if math.isfinite(gps_weight) else 0.0
    imu = max(0.0, float(imu_weight)) if math.isfinite(imu_weight) else 0.0
    total = gps + imu
    if total <= 1.0e-9:
        return 0.0, 1.0
    return gps / total, imu / total


def diag_covariance(
    x_value: float,
    y_value: float,
    z_value: float,
    roll_value: float,
    pitch_value: float,
    yaw_value: float,
) -> List[float]:
    covariance = [0.0] * 36
    covariance[0] = float(x_value)
    covariance[7] = float(y_value)
    covariance[14] = float(z_value)
    covariance[21] = float(roll_value)
    covariance[28] = float(pitch_value)
    covariance[35] = float(yaw_value)
    return covariance


def scaled_covariance(
    base_covariance: List[float],
    scale_xy: float,
    scale_yaw: float,
) -> List[float]:
    out = list(base_covariance)
    xy = max(1.0, float(scale_xy)) if math.isfinite(scale_xy) else 1.0
    yaw = max(1.0, float(scale_yaw)) if math.isfinite(scale_yaw) else 1.0
    out[0] *= xy
    out[7] *= xy
    out[35] *= yaw
    return out


def random_walk_step(
    *,
    current_value: float,
    sigma_per_sqrt_s: float,
    dt_s: float,
    rng: random.Random,
) -> float:
    sigma = clamp_non_negative(sigma_per_sqrt_s)
    dt = clamp_non_negative(dt_s)
    if sigma <= 0.0 or dt <= 0.0:
        return float(current_value)
    return float(current_value) + rng.gauss(0.0, sigma * math.sqrt(dt))


def add_gaussian_noise(value: float, stddev: float, rng: random.Random) -> float:
    sigma = clamp_non_negative(stddev)
    if sigma <= 0.0:
        return float(value)
    return float(value) + rng.gauss(0.0, sigma)


@dataclass(order=True)
class DelayedItem:
    release_time_s: float
    msg: Any = field(compare=False)


class DelayedMessageQueue:
    def __init__(self) -> None:
        self._queue: List[DelayedItem] = []

    def push(self, *, release_time_s: float, msg: Any) -> None:
        heapq.heappush(self._queue, DelayedItem(float(release_time_s), msg))

    def pop_ready(self, *, now_s: float) -> List[Any]:
        ready: List[Any] = []
        now = float(now_s)
        while self._queue and self._queue[0].release_time_s <= now:
            ready.append(heapq.heappop(self._queue).msg)
        return ready


@dataclass
class BiasState:
    position_x_m: float = 0.0
    position_y_m: float = 0.0
    position_z_m: float = 0.0
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    velocity_z_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    yaw_rad: float = 0.0


class PixhawkOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("pixhawk_odometry")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("output_odom_topic", "/odometry/pixhawk")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("latency_s", 0.03)
        self.declare_parameter("position_std_m", 0.03)
        self.declare_parameter("velocity_std_mps", 0.05)
        self.declare_parameter("yaw_std_rad", 0.008)
        self.declare_parameter("position_bias_rw_mpsqrt", 0.003)
        self.declare_parameter("velocity_bias_rw_mps2sqrt", 0.015)
        self.declare_parameter("yaw_bias_rw_rpssqrt", 0.0015)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("gps_velocity_lpf_alpha", 0.55)
        self.declare_parameter("imu_velocity_lpf_alpha", 0.35)
        self.declare_parameter("twist_weight_gps", 0.65)
        self.declare_parameter("twist_weight_imu", 0.35)
        self.declare_parameter("gps_timeout_s", 1.0)
        self.declare_parameter("covariance_growth_per_s", 0.5)
        self.declare_parameter("gps_min_dt_s", 0.05)
        self.declare_parameter("gps_max_dt_s", 2.5)
        self.declare_parameter("imu_remove_gravity", True)
        self.declare_parameter("imu_accel_max_abs_mps2", 15.0)
        self.declare_parameter("imu_gyro_max_abs_rps", 5.0)

        self.declare_parameter("pose_covariance_xy", 1.0)
        self.declare_parameter("pose_covariance_z", 1.0)
        self.declare_parameter("pose_covariance_roll", 1.0e6)
        self.declare_parameter("pose_covariance_pitch", 1.0e6)
        self.declare_parameter("pose_covariance_yaw", 0.02)
        self.declare_parameter("twist_covariance_vx", 1.0)
        self.declare_parameter("twist_covariance_vy", 1.0)
        self.declare_parameter("twist_covariance_vz", 0.25)
        self.declare_parameter("twist_covariance_roll_rate", 1.0e6)
        self.declare_parameter("twist_covariance_pitch_rate", 1.0e6)
        self.declare_parameter("twist_covariance_yaw_rate", 0.02)

        imu_topic = str(self.get_parameter("imu_topic").value)
        gps_topic = str(self.get_parameter("gps_topic").value)
        output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self._latency_s = clamp_non_negative(float(self.get_parameter("latency_s").value))
        self._position_std_m = clamp_non_negative(
            float(self.get_parameter("position_std_m").value)
        )
        self._velocity_std_mps = clamp_non_negative(
            float(self.get_parameter("velocity_std_mps").value)
        )
        self._yaw_std_rad = clamp_non_negative(float(self.get_parameter("yaw_std_rad").value))

        self._position_bias_rw_mpsqrt = clamp_non_negative(
            float(self.get_parameter("position_bias_rw_mpsqrt").value)
        )
        self._velocity_bias_rw_mps2sqrt = clamp_non_negative(
            float(self.get_parameter("velocity_bias_rw_mps2sqrt").value)
        )
        self._yaw_bias_rw_rpssqrt = clamp_non_negative(
            float(self.get_parameter("yaw_bias_rw_rpssqrt").value)
        )

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        self._gps_velocity_lpf_alpha = clamp_unit(
            float(self.get_parameter("gps_velocity_lpf_alpha").value),
            default=0.55,
        )
        self._imu_velocity_lpf_alpha = clamp_unit(
            float(self.get_parameter("imu_velocity_lpf_alpha").value),
            default=0.35,
        )
        self._twist_weight_gps = max(0.0, float(self.get_parameter("twist_weight_gps").value))
        self._twist_weight_imu = max(0.0, float(self.get_parameter("twist_weight_imu").value))
        self._gps_timeout_s = max(0.05, float(self.get_parameter("gps_timeout_s").value))
        self._covariance_growth_per_s = max(
            0.0, float(self.get_parameter("covariance_growth_per_s").value)
        )
        self._gps_min_dt_s = max(0.01, float(self.get_parameter("gps_min_dt_s").value))
        self._gps_max_dt_s = max(self._gps_min_dt_s, float(self.get_parameter("gps_max_dt_s").value))
        self._imu_remove_gravity = bool(self.get_parameter("imu_remove_gravity").value)
        self._imu_accel_max_abs_mps2 = max(
            0.1, float(self.get_parameter("imu_accel_max_abs_mps2").value)
        )
        self._imu_gyro_max_abs_rps = max(
            0.01, float(self.get_parameter("imu_gyro_max_abs_rps").value)
        )

        self._pose_covariance = diag_covariance(
            float(self.get_parameter("pose_covariance_xy").value),
            float(self.get_parameter("pose_covariance_xy").value),
            float(self.get_parameter("pose_covariance_z").value),
            float(self.get_parameter("pose_covariance_roll").value),
            float(self.get_parameter("pose_covariance_pitch").value),
            float(self.get_parameter("pose_covariance_yaw").value),
        )
        self._twist_covariance = diag_covariance(
            float(self.get_parameter("twist_covariance_vx").value),
            float(self.get_parameter("twist_covariance_vy").value),
            float(self.get_parameter("twist_covariance_vz").value),
            float(self.get_parameter("twist_covariance_roll_rate").value),
            float(self.get_parameter("twist_covariance_pitch_rate").value),
            float(self.get_parameter("twist_covariance_yaw_rate").value),
        )

        random_seed = int(self.get_parameter("random_seed").value)
        self._rng = random.Random(random_seed)
        self._bias = BiasState()

        self._origin_lat_deg: Optional[float] = None
        self._origin_lon_deg: Optional[float] = None

        self._last_gps_x_m: Optional[float] = None
        self._last_gps_y_m: Optional[float] = None
        self._last_gps_time_s: Optional[float] = None

        self._latest_gps_x_m: Optional[float] = None
        self._latest_gps_y_m: Optional[float] = None
        self._latest_gps_time_s: Optional[float] = None

        self._gps_velocity_x_mps: Optional[float] = None
        self._gps_velocity_y_mps: Optional[float] = None

        self._latest_yaw_rad: float = 0.0
        self._latest_yaw_valid = False
        self._latest_imu_time_s: Optional[float] = None

        self._latest_imu_acc_world_x_mps2: Optional[float] = None
        self._latest_imu_acc_world_y_mps2: Optional[float] = None
        self._latest_imu_wz_rps: Optional[float] = None

        self._imu_velocity_raw_x_mps: float = 0.0
        self._imu_velocity_raw_y_mps: float = 0.0
        self._imu_velocity_filt_x_mps: Optional[float] = None
        self._imu_velocity_filt_y_mps: Optional[float] = None
        self._imu_wz_filt_rps: Optional[float] = None

        self._pose_x_m: Optional[float] = None
        self._pose_y_m: Optional[float] = None
        self._last_update_time_s: Optional[float] = None

        self._delay_queue = DelayedMessageQueue()
        self._odom_pub = self.create_publisher(Odometry, output_odom_topic, 10)

        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps_fix, qos_profile_sensor_data)
        self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            "pixhawk_odometry ready "
            f"(imu={imu_topic}, gps={gps_topic}, output={output_odom_topic}, "
            f"publish_hz={publish_hz:.1f}, latency_s={self._latency_s:.3f}, seed={random_seed})"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _update_bias_state(self, dt_s: float) -> None:
        self._bias.position_x_m = random_walk_step(
            current_value=self._bias.position_x_m,
            sigma_per_sqrt_s=self._position_bias_rw_mpsqrt,
            dt_s=dt_s,
            rng=self._rng,
        )
        self._bias.position_y_m = random_walk_step(
            current_value=self._bias.position_y_m,
            sigma_per_sqrt_s=self._position_bias_rw_mpsqrt,
            dt_s=dt_s,
            rng=self._rng,
        )
        self._bias.position_z_m = random_walk_step(
            current_value=self._bias.position_z_m,
            sigma_per_sqrt_s=self._position_bias_rw_mpsqrt,
            dt_s=dt_s,
            rng=self._rng,
        )

        self._bias.velocity_x_mps = random_walk_step(
            current_value=self._bias.velocity_x_mps,
            sigma_per_sqrt_s=self._velocity_bias_rw_mps2sqrt,
            dt_s=dt_s,
            rng=self._rng,
        )
        self._bias.velocity_y_mps = random_walk_step(
            current_value=self._bias.velocity_y_mps,
            sigma_per_sqrt_s=self._velocity_bias_rw_mps2sqrt,
            dt_s=dt_s,
            rng=self._rng,
        )
        self._bias.velocity_z_mps = random_walk_step(
            current_value=self._bias.velocity_z_mps,
            sigma_per_sqrt_s=self._velocity_bias_rw_mps2sqrt,
            dt_s=dt_s,
            rng=self._rng,
        )

        self._bias.yaw_rate_rps = random_walk_step(
            current_value=self._bias.yaw_rate_rps,
            sigma_per_sqrt_s=self._yaw_bias_rw_rpssqrt,
            dt_s=dt_s,
            rng=self._rng,
        )
        self._bias.yaw_rad = wrap_angle(
            float(self._bias.yaw_rad) + float(self._bias.yaw_rate_rps) * float(dt_s)
        )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quaternion(q)
        self._latest_yaw_rad = float(yaw)
        self._latest_yaw_valid = True

        imu_stamp_s = stamp_to_seconds(msg.header.stamp)
        if (not math.isfinite(imu_stamp_s)) or imu_stamp_s <= 0.0:
            imu_stamp_s = self._now_s()
        self._latest_imu_time_s = imu_stamp_s

        raw_wz = float(msg.angular_velocity.z)
        if not math.isfinite(raw_wz):
            raw_wz = 0.0
        raw_wz = max(-self._imu_gyro_max_abs_rps, min(self._imu_gyro_max_abs_rps, raw_wz))
        self._latest_imu_wz_rps = raw_wz
        self._imu_wz_filt_rps = low_pass_filter(
            self._imu_wz_filt_rps,
            raw_wz,
            self._imu_velocity_lpf_alpha,
        )

        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(az)):
            return

        world_ax, world_ay, world_az = rotate_vector_body_to_world(
            ax,
            ay,
            az,
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        if self._imu_remove_gravity:
            world_az -= GRAVITY_MPS2

        _ = world_az  # Keep explicit for readability even in 2D mode.
        ax_clamped = max(
            -self._imu_accel_max_abs_mps2,
            min(self._imu_accel_max_abs_mps2, float(world_ax)),
        )
        ay_clamped = max(
            -self._imu_accel_max_abs_mps2,
            min(self._imu_accel_max_abs_mps2, float(world_ay)),
        )
        self._latest_imu_acc_world_x_mps2 = ax_clamped
        self._latest_imu_acc_world_y_mps2 = ay_clamped

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if (not math.isfinite(lat)) or (not math.isfinite(lon)):
            return
        if int(msg.status.status) == int(NavSatStatus.STATUS_NO_FIX):
            return

        if self._origin_lat_deg is None or self._origin_lon_deg is None:
            self._origin_lat_deg = lat
            self._origin_lon_deg = lon

        x_m, y_m = local_xy_from_gps(
            lat_deg=lat,
            lon_deg=lon,
            origin_lat_deg=float(self._origin_lat_deg),
            origin_lon_deg=float(self._origin_lon_deg),
        )

        gps_stamp_s = stamp_to_seconds(msg.header.stamp)
        if (not math.isfinite(gps_stamp_s)) or gps_stamp_s <= 0.0:
            gps_stamp_s = self._now_s()

        if (
            self._last_gps_x_m is not None
            and self._last_gps_y_m is not None
            and self._last_gps_time_s is not None
        ):
            derived = gps_velocity_from_positions(
                prev_x_m=self._last_gps_x_m,
                prev_y_m=self._last_gps_y_m,
                prev_t_s=self._last_gps_time_s,
                cur_x_m=x_m,
                cur_y_m=y_m,
                cur_t_s=gps_stamp_s,
                min_dt_s=self._gps_min_dt_s,
                max_dt_s=self._gps_max_dt_s,
            )
            if derived is not None:
                vx_gps, vy_gps = derived
                self._gps_velocity_x_mps = low_pass_filter(
                    self._gps_velocity_x_mps,
                    vx_gps,
                    self._gps_velocity_lpf_alpha,
                )
                self._gps_velocity_y_mps = low_pass_filter(
                    self._gps_velocity_y_mps,
                    vy_gps,
                    self._gps_velocity_lpf_alpha,
                )

        self._last_gps_x_m = x_m
        self._last_gps_y_m = y_m
        self._last_gps_time_s = gps_stamp_s

        self._latest_gps_x_m = x_m
        self._latest_gps_y_m = y_m
        self._latest_gps_time_s = gps_stamp_s

        if self._pose_x_m is None or self._pose_y_m is None:
            self._pose_x_m = x_m
            self._pose_y_m = y_m

    def _gps_is_fresh(self, now_s: float) -> bool:
        if self._latest_gps_time_s is None:
            return False
        return (float(now_s) - float(self._latest_gps_time_s)) <= float(self._gps_timeout_s)

    def _imu_is_available(self) -> bool:
        return (
            self._latest_imu_acc_world_x_mps2 is not None
            and self._latest_imu_acc_world_y_mps2 is not None
        )

    def _build_clean_odom(
        self,
        *,
        now_s: float,
        dt_s: float,
        v_world_x_mps: float,
        v_world_y_mps: float,
    ) -> Optional[Odometry]:
        if self._pose_x_m is None or self._pose_y_m is None:
            return None

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame

        yaw = float(self._latest_yaw_rad) if self._latest_yaw_valid else 0.0

        gps_fresh = self._gps_is_fresh(now_s)
        if gps_fresh and self._latest_gps_x_m is not None and self._latest_gps_y_m is not None:
            self._pose_x_m = float(self._latest_gps_x_m)
            self._pose_y_m = float(self._latest_gps_y_m)
        elif dt_s > 0.0:
            self._pose_x_m += float(v_world_x_mps) * float(dt_s)
            self._pose_y_m += float(v_world_y_mps) * float(dt_s)

        out.pose.pose.position.x = float(self._pose_x_m)
        out.pose.pose.position.y = float(self._pose_y_m)
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = quaternion_from_yaw(yaw)

        vx_body, vy_body = world_velocity_to_body(
            vx_world_mps=v_world_x_mps,
            vy_world_mps=v_world_y_mps,
            yaw_rad=yaw,
        )

        out.twist.twist.linear.x = float(vx_body)
        out.twist.twist.linear.y = float(vy_body)
        out.twist.twist.linear.z = 0.0

        wz = self._imu_wz_filt_rps if self._imu_wz_filt_rps is not None else 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = float(wz)

        gps_stale_s = 0.0
        if self._latest_gps_time_s is None:
            gps_stale_s = float(self._gps_timeout_s)
        else:
            gps_stale_s = max(0.0, float(now_s) - float(self._latest_gps_time_s))

        growth = 1.0 + max(
            0.0,
            (gps_stale_s - float(self._gps_timeout_s)) * float(self._covariance_growth_per_s),
        )
        out.pose.covariance = scaled_covariance(self._pose_covariance, growth, growth)
        out.twist.covariance = scaled_covariance(self._twist_covariance, growth, growth)

        return out

    def _build_noisy_odom(self, clean_msg: Odometry) -> Odometry:
        out = Odometry()
        out.header.stamp = clean_msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame

        out.pose.pose.position.x = add_gaussian_noise(
            float(clean_msg.pose.pose.position.x) + self._bias.position_x_m,
            self._position_std_m,
            self._rng,
        )
        out.pose.pose.position.y = add_gaussian_noise(
            float(clean_msg.pose.pose.position.y) + self._bias.position_y_m,
            self._position_std_m,
            self._rng,
        )
        out.pose.pose.position.z = add_gaussian_noise(
            float(clean_msg.pose.pose.position.z) + self._bias.position_z_m,
            self._position_std_m,
            self._rng,
        )

        clean_yaw = yaw_from_quaternion(clean_msg.pose.pose.orientation)
        noisy_yaw = wrap_angle(
            add_gaussian_noise(
                clean_yaw + self._bias.yaw_rad,
                self._yaw_std_rad,
                self._rng,
            )
        )
        out.pose.pose.orientation = quaternion_from_yaw(noisy_yaw)
        out.pose.covariance = list(clean_msg.pose.covariance)

        out.twist.twist.linear.x = add_gaussian_noise(
            float(clean_msg.twist.twist.linear.x) + self._bias.velocity_x_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.linear.y = add_gaussian_noise(
            float(clean_msg.twist.twist.linear.y) + self._bias.velocity_y_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.linear.z = add_gaussian_noise(
            float(clean_msg.twist.twist.linear.z) + self._bias.velocity_z_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = (
            float(clean_msg.twist.twist.angular.z) + float(self._bias.yaw_rate_rps)
        )
        out.twist.covariance = list(clean_msg.twist.covariance)
        return out

    def _on_timer(self) -> None:
        now_s = self._now_s()
        if self._last_update_time_s is None:
            self._last_update_time_s = now_s

        dt_s = max(0.0, float(now_s) - float(self._last_update_time_s))
        self._last_update_time_s = now_s

        self._update_bias_state(dt_s)

        if self._imu_is_available() and dt_s > 0.0:
            self._imu_velocity_raw_x_mps += float(self._latest_imu_acc_world_x_mps2) * float(dt_s)
            self._imu_velocity_raw_y_mps += float(self._latest_imu_acc_world_y_mps2) * float(dt_s)
            self._imu_velocity_filt_x_mps = low_pass_filter(
                self._imu_velocity_filt_x_mps,
                self._imu_velocity_raw_x_mps,
                self._imu_velocity_lpf_alpha,
            )
            self._imu_velocity_filt_y_mps = low_pass_filter(
                self._imu_velocity_filt_y_mps,
                self._imu_velocity_raw_y_mps,
                self._imu_velocity_lpf_alpha,
            )

        gps_fresh = self._gps_is_fresh(now_s)
        has_gps_velocity = (
            self._gps_velocity_x_mps is not None and self._gps_velocity_y_mps is not None and gps_fresh
        )
        has_imu_velocity = (
            self._imu_velocity_filt_x_mps is not None and self._imu_velocity_filt_y_mps is not None
        )

        gps_weight = self._twist_weight_gps if has_gps_velocity else 0.0
        imu_weight = self._twist_weight_imu if has_imu_velocity else 0.0
        w_gps, w_imu = normalized_fusion_weights(gps_weight, imu_weight)

        v_world_x_mps = (w_gps * float(self._gps_velocity_x_mps or 0.0)) + (
            w_imu * float(self._imu_velocity_filt_x_mps or 0.0)
        )
        v_world_y_mps = (w_gps * float(self._gps_velocity_y_mps or 0.0)) + (
            w_imu * float(self._imu_velocity_filt_y_mps or 0.0)
        )

        clean_msg = self._build_clean_odom(
            now_s=now_s,
            dt_s=dt_s,
            v_world_x_mps=v_world_x_mps,
            v_world_y_mps=v_world_y_mps,
        )

        if clean_msg is not None:
            noisy_msg = self._build_noisy_odom(clean_msg)
            self._delay_queue.push(release_time_s=now_s + self._latency_s, msg=noisy_msg)

        for msg in self._delay_queue.pop_ready(now_s=now_s):
            self._odom_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixhawkOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
