from __future__ import annotations

from copy import deepcopy
import math
import random
import sys
from functools import lru_cache
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import CmdVelFinal
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, PointCloud2


def _fallback_command_from_cmd_vel(
    linear_x: float,
    angular_z: float,
    brake_pct: int,
    max_speed_mps: float,
    max_reverse_mps: float,
    vx_deadband_mps: float,
    vx_min_effective_mps: float,
    max_abs_angular_z: float,
    invert_steer: bool,
    auto_drive_enabled: bool,
    reverse_brake_pct: int,
):
    del reverse_brake_pct

    max_speed = max(0.0, float(max_speed_mps))
    max_reverse = max(0.0, float(max_reverse_mps))
    deadband = max(0.0, float(vx_deadband_mps))
    min_effective = min(max(float(vx_min_effective_mps), 0.0), max_speed)

    linear = float(linear_x)
    speed = 0.0
    if linear > 0.0:
        speed = min(max(linear, 0.0), max_speed)
        if speed < deadband:
            speed = 0.0
        elif speed < min_effective:
            speed = min_effective
    elif linear < 0.0:
        reverse_speed = min(max(abs(linear), 0.0), max_reverse)
        speed = 0.0 if reverse_speed < deadband else -reverse_speed

    angular_scale = max(0.01, abs(float(max_abs_angular_z)))
    steer_ratio = min(max(float(angular_z) / angular_scale, -1.0), 1.0)
    steer_pct = int(round(steer_ratio * 100.0))
    if bool(invert_steer):
        steer_pct = -steer_pct

    clamped_brake = int(min(max(float(brake_pct), 0.0), 100.0))
    estop = clamped_brake > 0
    if estop:
        speed = 0.0
        steer_pct = 0

    class _DesiredCommand:
        def __init__(self) -> None:
            self.drive_enabled = bool(auto_drive_enabled)
            self.estop = estop
            self.speed_mps = speed
            self.steer_pct = steer_pct
            self.brake_pct = clamped_brake

    return _DesiredCommand()


@lru_cache(maxsize=1)
def _load_command_from_cmd_vel():
    try:
        from controller_server.control_logic import command_from_cmd_vel

        return command_from_cmd_vel
    except ModuleNotFoundError:
        controller_repo = Path(__file__).resolve().parents[2] / "controller_server"
        if controller_repo.exists():
            controller_repo_str = str(controller_repo)
            if controller_repo_str not in sys.path:
                sys.path.append(controller_repo_str)
            try:
                from controller_server.control_logic import command_from_cmd_vel

                return command_from_cmd_vel
            except (ModuleNotFoundError, TypeError):
                pass
    except TypeError:
        pass
    return _fallback_command_from_cmd_vel


def _stamp_to_nanoseconds(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _offset_geodetic_fix(
    latitude_deg: float,
    longitude_deg: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    meters_per_deg_lat = 111_320.0
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(latitude_deg)))))
    meters_per_deg_lon = meters_per_deg_lat * cos_lat
    return (
        float(latitude_deg) + float(north_m) / meters_per_deg_lat,
        float(longitude_deg) + float(east_m) / meters_per_deg_lon,
    )


class GazeboUtilsNode(Node):
    def __init__(self):
        super().__init__("gazebo_utils")
        self.declare_parameter("strip_prefix", True)

        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("imu_out_topic", "/imu/data")
        self.declare_parameter("imu_frame_id", "imu_link")

        self.declare_parameter("gps_in_topic", "/gps/fix_raw")
        self.declare_parameter("gps_out_topic", "/gps/fix")
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("use_realistic_gps", False)
        self.declare_parameter("gps_publish_rate_hz", 5.0)
        self.declare_parameter("gps_publish_jitter_stddev_s", 0.03)
        self.declare_parameter("gps_horizontal_noise_stddev_m", 0.35)
        self.declare_parameter("gps_vertical_noise_stddev_m", 0.75)
        self.declare_parameter("gps_bias_walk_stddev_m_per_sqrt_s", 0.02)
        self.declare_parameter("gps_random_seed", 0)

        self.declare_parameter("lidar_in_topic", "/scan_3d_raw")
        self.declare_parameter("lidar_out_topic", "/scan_3d")
        self.declare_parameter("lidar_frame_id", "lidar_link")

        self.declare_parameter(
            "ultrasound_rear_center_in_topic", "/ultrasound/rear_center_raw"
        )
        self.declare_parameter(
            "ultrasound_rear_center_out_topic", "/ultrasound/rear_center"
        )
        self.declare_parameter(
            "ultrasound_rear_center_frame_id", "rear_center_ultrasound"
        )
        self.declare_parameter(
            "ultrasound_rear_left_in_topic", "/ultrasound/rear_left_raw"
        )
        self.declare_parameter("ultrasound_rear_left_out_topic", "/ultrasound/rear_left")
        self.declare_parameter("ultrasound_rear_left_frame_id", "rear_left_ultrasound")
        self.declare_parameter(
            "ultrasound_rear_right_in_topic", "/ultrasound/rear_right_raw"
        )
        self.declare_parameter(
            "ultrasound_rear_right_out_topic", "/ultrasound/rear_right"
        )
        self.declare_parameter(
            "ultrasound_rear_right_frame_id", "rear_right_ultrasound"
        )
        self.declare_parameter(
            "ultrasound_front_left_in_topic", "/ultrasound/front_left_raw"
        )
        self.declare_parameter(
            "ultrasound_front_left_out_topic", "/ultrasound/front_left"
        )
        self.declare_parameter(
            "ultrasound_front_left_frame_id", "front_left_ultrasound"
        )
        self.declare_parameter(
            "ultrasound_front_right_in_topic", "/ultrasound/front_right_raw"
        )
        self.declare_parameter(
            "ultrasound_front_right_out_topic", "/ultrasound/front_right"
        )
        self.declare_parameter(
            "ultrasound_front_right_frame_id", "front_right_ultrasound"
        )
        self.declare_parameter("enable_ultrasound_bridge", True)

        self.declare_parameter("odom_in_topic", "/odom_raw")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_link_frame_id", "base_footprint")
        self.declare_parameter("enable_cmd_vel_final_bridge", False)
        self.declare_parameter("use_realistic_cmd_vel_bridge", False)
        self.declare_parameter("cmd_vel_final_in_topic", "/cmd_vel_final")
        self.declare_parameter("cmd_vel_gazebo_out_topic", "/cmd_vel_gazebo")
        self.declare_parameter("max_speed_mps", 4.0)
        self.declare_parameter("max_reverse_mps", 1.30)
        self.declare_parameter("vx_deadband_mps", 0.10)
        self.declare_parameter("vx_min_effective_mps", 0.75)
        self.declare_parameter("max_abs_angular_z", 0.4)
        self.declare_parameter("invert_steer_from_cmd_vel", False)
        self.declare_parameter("auto_drive_enabled", True)
        self.declare_parameter("reverse_brake_pct", 20)
        self.declare_parameter("sim_max_forward_mps", 4.0)
        self.declare_parameter("sim_max_reverse_mps", 1.30)
        self.declare_parameter("sim_max_abs_angular_z", 0.4)

        self.strip_prefix = self.get_parameter("strip_prefix").value
        self.use_realistic_gps = bool(self.get_parameter("use_realistic_gps").value)
        self.enable_ultrasound_bridge = bool(
            self.get_parameter("enable_ultrasound_bridge").value
        )
        self.enable_cmd_vel_final_bridge = bool(
            self.get_parameter("enable_cmd_vel_final_bridge").value
        )
        self.use_realistic_cmd_vel_bridge = bool(
            self.get_parameter("use_realistic_cmd_vel_bridge").value
        )
        self.cmd_vel_final_in_topic = str(
            self.get_parameter("cmd_vel_final_in_topic").value
        )
        self.cmd_vel_gazebo_out_topic = str(
            self.get_parameter("cmd_vel_gazebo_out_topic").value
        )
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.max_reverse_mps = max(
            0.0, float(self.get_parameter("max_reverse_mps").value)
        )
        self.vx_deadband_mps = max(
            0.0, float(self.get_parameter("vx_deadband_mps").value)
        )
        self.vx_min_effective_mps = min(
            max(0.0, float(self.get_parameter("vx_min_effective_mps").value)),
            self.max_speed_mps,
        )
        self.max_abs_angular_z = float(self.get_parameter("max_abs_angular_z").value)
        self.invert_steer_from_cmd_vel = bool(
            self.get_parameter("invert_steer_from_cmd_vel").value
        )
        self.auto_drive_enabled = bool(self.get_parameter("auto_drive_enabled").value)
        self.reverse_brake_pct = int(self.get_parameter("reverse_brake_pct").value)
        self.sim_max_forward_mps = max(
            0.0, float(self.get_parameter("sim_max_forward_mps").value)
        )
        self.sim_max_reverse_mps = max(
            0.0, float(self.get_parameter("sim_max_reverse_mps").value)
        )
        self.sim_max_abs_angular_z = abs(
            float(self.get_parameter("sim_max_abs_angular_z").value)
        )
        self.gps_publish_rate_hz = max(
            0.0, float(self.get_parameter("gps_publish_rate_hz").value)
        )
        self.gps_publish_jitter_stddev_s = max(
            0.0, float(self.get_parameter("gps_publish_jitter_stddev_s").value)
        )
        self.gps_horizontal_noise_stddev_m = max(
            0.0, float(self.get_parameter("gps_horizontal_noise_stddev_m").value)
        )
        self.gps_vertical_noise_stddev_m = max(
            0.0, float(self.get_parameter("gps_vertical_noise_stddev_m").value)
        )
        self.gps_bias_walk_stddev_m_per_sqrt_s = max(
            0.0, float(self.get_parameter("gps_bias_walk_stddev_m_per_sqrt_s").value)
        )
        gps_random_seed = int(self.get_parameter("gps_random_seed").value)
        self._gps_random = random.Random(None if gps_random_seed == 0 else gps_random_seed)
        self._gps_next_publish_time_ns = 0
        self._gps_last_publish_time_ns = None
        self._gps_bias_north_m = 0.0
        self._gps_bias_east_m = 0.0
        self._gps_bias_alt_m = 0.0

        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.gps_frame_id = self.get_parameter("gps_frame_id").value
        self.lidar_frame_id = self.get_parameter("lidar_frame_id").value
        self.ultrasound_rear_center_frame_id = self.get_parameter(
            "ultrasound_rear_center_frame_id"
        ).value
        self.ultrasound_rear_left_frame_id = self.get_parameter(
            "ultrasound_rear_left_frame_id"
        ).value
        self.ultrasound_rear_right_frame_id = self.get_parameter(
            "ultrasound_rear_right_frame_id"
        ).value
        self.ultrasound_front_left_frame_id = self.get_parameter(
            "ultrasound_front_left_frame_id"
        ).value
        self.ultrasound_front_right_frame_id = self.get_parameter(
            "ultrasound_front_right_frame_id"
        ).value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value

        imu_in_topic = self.get_parameter("imu_in_topic").value
        imu_out_topic = self.get_parameter("imu_out_topic").value
        gps_in_topic = self.get_parameter("gps_in_topic").value
        gps_out_topic = self.get_parameter("gps_out_topic").value
        lidar_in_topic = self.get_parameter("lidar_in_topic").value
        lidar_out_topic = self.get_parameter("lidar_out_topic").value
        ultrasound_rear_center_in_topic = self.get_parameter(
            "ultrasound_rear_center_in_topic"
        ).value
        ultrasound_rear_center_out_topic = self.get_parameter(
            "ultrasound_rear_center_out_topic"
        ).value
        ultrasound_rear_left_in_topic = self.get_parameter(
            "ultrasound_rear_left_in_topic"
        ).value
        ultrasound_rear_left_out_topic = self.get_parameter(
            "ultrasound_rear_left_out_topic"
        ).value
        ultrasound_rear_right_in_topic = self.get_parameter(
            "ultrasound_rear_right_in_topic"
        ).value
        ultrasound_rear_right_out_topic = self.get_parameter(
            "ultrasound_rear_right_out_topic"
        ).value
        ultrasound_front_left_in_topic = self.get_parameter(
            "ultrasound_front_left_in_topic"
        ).value
        ultrasound_front_left_out_topic = self.get_parameter(
            "ultrasound_front_left_out_topic"
        ).value
        ultrasound_front_right_in_topic = self.get_parameter(
            "ultrasound_front_right_in_topic"
        ).value
        ultrasound_front_right_out_topic = self.get_parameter(
            "ultrasound_front_right_out_topic"
        ).value
        odom_in_topic = self.get_parameter("odom_in_topic").value
        odom_out_topic = self.get_parameter("odom_out_topic").value

        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, gps_out_topic, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_out_topic, 10)
        self.ultrasound_rear_center_pub = None
        self.ultrasound_rear_left_pub = None
        self.ultrasound_rear_right_pub = None
        self.ultrasound_front_left_pub = None
        self.ultrasound_front_right_pub = None
        if self.enable_ultrasound_bridge:
            self.ultrasound_rear_center_pub = self.create_publisher(
                LaserScan, ultrasound_rear_center_out_topic, 10
            )
            self.ultrasound_rear_left_pub = self.create_publisher(
                LaserScan, ultrasound_rear_left_out_topic, 10
            )
            self.ultrasound_rear_right_pub = self.create_publisher(
                LaserScan, ultrasound_rear_right_out_topic, 10
            )
            self.ultrasound_front_left_pub = self.create_publisher(
                LaserScan, ultrasound_front_left_out_topic, 10
            )
            self.ultrasound_front_right_pub = self.create_publisher(
                LaserScan, ultrasound_front_right_out_topic, 10
            )
        self.odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)

        self.create_subscription(Imu, imu_in_topic, self._imu_cb, 10)
        self.create_subscription(NavSatFix, gps_in_topic, self._gps_cb, 10)
        self.create_subscription(PointCloud2, lidar_in_topic, self._lidar_cb, 10)
        if self.enable_ultrasound_bridge:
            self.create_subscription(
                LaserScan,
                ultrasound_rear_center_in_topic,
                self._ultrasound_rear_center_cb,
                10,
            )
            self.create_subscription(
                LaserScan,
                ultrasound_rear_left_in_topic,
                self._ultrasound_rear_left_cb,
                10,
            )
            self.create_subscription(
                LaserScan,
                ultrasound_rear_right_in_topic,
                self._ultrasound_rear_right_cb,
                10,
            )
            self.create_subscription(
                LaserScan,
                ultrasound_front_left_in_topic,
                self._ultrasound_front_left_cb,
                10,
            )
            self.create_subscription(
                LaserScan,
                ultrasound_front_right_in_topic,
                self._ultrasound_front_right_cb,
                10,
            )
        self.create_subscription(Odometry, odom_in_topic, self._odom_cb, 10)

        self.cmd_vel_gazebo_pub = None
        if self.enable_cmd_vel_final_bridge:
            self.cmd_vel_gazebo_pub = self.create_publisher(
                Twist, self.cmd_vel_gazebo_out_topic, 10
            )
            self.create_subscription(
                CmdVelFinal,
                self.cmd_vel_final_in_topic,
                self._cmd_vel_final_cb,
                10,
            )
            self.get_logger().info(
                "Gazebo cmd bridge enabled "
                f"({self.cmd_vel_final_in_topic} -> {self.cmd_vel_gazebo_out_topic})"
            )
        else:
            self.get_logger().info("Gazebo cmd bridge disabled")

    def _strip(self, frame_id: str) -> str:
        if not self.strip_prefix:
            return frame_id
        if "::" in frame_id:
            return frame_id.split("::")[-1]
        return frame_id

    def _resolve_frame(self, incoming: str, override: str) -> str:
        if override:
            return override
        return self._strip(incoming)

    def _imu_cb(self, msg: Imu):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.imu_frame_id)
        self.imu_pub.publish(msg)

    def _get_reference_time_ns(self, msg: NavSatFix) -> int:
        now_ns = 0
        try:
            now_ns = int(self.get_clock().now().nanoseconds)
        except AttributeError:
            pass
        return max(now_ns, _stamp_to_nanoseconds(msg))

    def _advance_gps_bias(self, dt_s: float) -> None:
        if self.gps_bias_walk_stddev_m_per_sqrt_s <= 0.0:
            return
        sigma = self.gps_bias_walk_stddev_m_per_sqrt_s * math.sqrt(max(0.0, dt_s))
        self._gps_bias_north_m += self._gps_random.gauss(0.0, sigma)
        self._gps_bias_east_m += self._gps_random.gauss(0.0, sigma)
        self._gps_bias_alt_m += self._gps_random.gauss(0.0, sigma)

    def _schedule_next_gps_publish(self, reference_time_ns: int) -> None:
        if self.gps_publish_rate_hz <= 0.0:
            self._gps_next_publish_time_ns = reference_time_ns
            return
        base_period_s = 1.0 / self.gps_publish_rate_hz
        jitter_s = self._gps_random.gauss(0.0, self.gps_publish_jitter_stddev_s)
        period_s = max(0.02, base_period_s + jitter_s)
        self._gps_next_publish_time_ns = reference_time_ns + int(period_s * 1_000_000_000)

    def _apply_realistic_gps(self, msg: NavSatFix, reference_time_ns: int) -> NavSatFix | None:
        if reference_time_ns < self._gps_next_publish_time_ns:
            return None

        if self._gps_last_publish_time_ns is None:
            dt_s = 0.0
        else:
            dt_s = max(
                0.0,
                (reference_time_ns - self._gps_last_publish_time_ns) / 1_000_000_000.0,
            )
        self._gps_last_publish_time_ns = reference_time_ns
        self._advance_gps_bias(dt_s)

        noisy_msg = deepcopy(msg)
        north_m = self._gps_bias_north_m + self._gps_random.gauss(
            0.0, self.gps_horizontal_noise_stddev_m
        )
        east_m = self._gps_bias_east_m + self._gps_random.gauss(
            0.0, self.gps_horizontal_noise_stddev_m
        )
        noisy_msg.latitude, noisy_msg.longitude = _offset_geodetic_fix(
            noisy_msg.latitude,
            noisy_msg.longitude,
            north_m,
            east_m,
        )
        noisy_msg.altitude = (
            float(noisy_msg.altitude)
            + self._gps_bias_alt_m
            + self._gps_random.gauss(0.0, self.gps_vertical_noise_stddev_m)
        )

        horizontal_variance = self.gps_horizontal_noise_stddev_m**2
        vertical_variance = self.gps_vertical_noise_stddev_m**2
        noisy_msg.position_covariance = [
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
        noisy_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._schedule_next_gps_publish(reference_time_ns)
        return noisy_msg

    def _gps_cb(self, msg: NavSatFix):
        out_msg = msg
        if self.use_realistic_gps:
            out_msg = self._apply_realistic_gps(msg, self._get_reference_time_ns(msg))
            if out_msg is None:
                return
        out_msg.header.frame_id = self._resolve_frame(out_msg.header.frame_id, self.gps_frame_id)
        self.gps_pub.publish(out_msg)

    def _lidar_cb(self, msg: PointCloud2):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.lidar_frame_id)
        self.lidar_pub.publish(msg)

    def _ultrasound_rear_center_cb(self, msg: LaserScan):
        if self.ultrasound_rear_center_pub is None:
            return
        msg.header.frame_id = self._resolve_frame(
            msg.header.frame_id, self.ultrasound_rear_center_frame_id
        )
        self.ultrasound_rear_center_pub.publish(msg)

    def _ultrasound_rear_left_cb(self, msg: LaserScan):
        if self.ultrasound_rear_left_pub is None:
            return
        msg.header.frame_id = self._resolve_frame(
            msg.header.frame_id, self.ultrasound_rear_left_frame_id
        )
        self.ultrasound_rear_left_pub.publish(msg)

    def _ultrasound_rear_right_cb(self, msg: LaserScan):
        if self.ultrasound_rear_right_pub is None:
            return
        msg.header.frame_id = self._resolve_frame(
            msg.header.frame_id, self.ultrasound_rear_right_frame_id
        )
        self.ultrasound_rear_right_pub.publish(msg)

    def _ultrasound_front_left_cb(self, msg: LaserScan):
        if self.ultrasound_front_left_pub is None:
            return
        msg.header.frame_id = self._resolve_frame(
            msg.header.frame_id, self.ultrasound_front_left_frame_id
        )
        self.ultrasound_front_left_pub.publish(msg)

    def _ultrasound_front_right_cb(self, msg: LaserScan):
        if self.ultrasound_front_right_pub is None:
            return
        msg.header.frame_id = self._resolve_frame(
            msg.header.frame_id, self.ultrasound_front_right_frame_id
        )
        self.ultrasound_front_right_pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.odom_frame_id)
        msg.child_frame_id = self._resolve_frame(msg.child_frame_id, self.base_link_frame_id)
        self.odom_pub.publish(msg)

    def _publish_cmd_vel_gazebo(self, linear_x: float, angular_z: float) -> None:
        if self.cmd_vel_gazebo_pub is None:
            return
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_gazebo_pub.publish(twist)

    def _translate_cmd_vel_final_to_gazebo(self, msg: CmdVelFinal) -> tuple[float, float]:
        if not self.use_realistic_cmd_vel_bridge:
            if int(msg.brake_pct) > 0:
                return (0.0, 0.0)
            return (float(msg.twist.linear.x), float(msg.twist.angular.z))

        command_from_cmd_vel = _load_command_from_cmd_vel()
        desired_command = command_from_cmd_vel(
            linear_x=msg.twist.linear.x,
            angular_z=msg.twist.angular.z,
            brake_pct=msg.brake_pct,
            max_speed_mps=self.max_speed_mps,
            max_reverse_mps=self.max_reverse_mps,
            vx_deadband_mps=self.vx_deadband_mps,
            vx_min_effective_mps=self.vx_min_effective_mps,
            max_abs_angular_z=self.max_abs_angular_z,
            invert_steer=self.invert_steer_from_cmd_vel,
            auto_drive_enabled=self.auto_drive_enabled,
            reverse_brake_pct=self.reverse_brake_pct,
        )
        if (
            bool(desired_command.estop)
            or int(desired_command.brake_pct) > 0
            or not bool(desired_command.drive_enabled)
        ):
            return (0.0, 0.0)

        linear_x = max(
            -self.sim_max_reverse_mps,
            min(self.sim_max_forward_mps, float(desired_command.speed_mps)),
        )
        angular_z = float(desired_command.steer_pct) / 100.0 * self.sim_max_abs_angular_z
        return (linear_x, angular_z)

    def _cmd_vel_final_cb(self, msg: CmdVelFinal) -> None:
        linear_x, angular_z = self._translate_cmd_vel_final_to_gazebo(msg)
        self._publish_cmd_vel_gazebo(linear_x, angular_z)


def main():
    rclpy.init()
    node = GazeboUtilsNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
