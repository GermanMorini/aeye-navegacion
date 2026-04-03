import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64

from message_filters import ApproximateTimeSynchronizer, Subscriber


_EARTH_R = 6_378_137.0  # metres

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


def _ll_to_enu_delta(lat_ref_deg, lon_ref_deg, lat_deg, lon_deg):
    """Return (dx_east_m, dy_north_m) from reference point to target point."""
    lat_ref = math.radians(lat_ref_deg)
    dy = math.radians(lat_deg - lat_ref_deg) * _EARTH_R
    dx = math.radians(lon_deg - lon_ref_deg) * _EARTH_R * math.cos(lat_ref)
    return dx, dy


def _yaw_to_quaternion(yaw_rad):
    """Pure yaw quaternion → (x, y, z, w)."""
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class DualGpsHeadingSim(Node):
    def __init__(self):
        super().__init__("dual_gps_heading")

        self.declare_parameter("gps_right_topic", "/gps/fix")   # gps_link
        self.declare_parameter("gps_left_topic", "/gps2/fix")   # gps_link2
        self.declare_parameter("raw_heading_imu_topic", "")
        self.declare_parameter("heading_imu_topic", "/dual_gps/heading")
        self.declare_parameter("heading_deg_topic", "/dual_gps/heading_deg")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("corrected_yaw_offset_rad", math.pi / 2.0)
        self.declare_parameter("odom_heading_topic", "")
        self.declare_parameter("odom_heading_max_hz", 20.0)
        # Minimum projected baseline [m] below which heading is discarded.
        # With 0.5 m physical baseline, GPS noise ~0.05 m, values < 0.3 m
        # indicate outlier fixes.
        self.declare_parameter("min_baseline_m", 0.3)
        self.declare_parameter("sync_slop_s", 0.15)
        # Yaw covariance [rad²].  With 0.5 m baseline and ~0.055 m pos σ:
        #   σ_yaw ≈ sqrt(2)*0.055/0.5 ≈ 0.155 rad  →  cov ≈ 0.024 rad²
        self.declare_parameter("heading_covariance", 0.025)

        gps_right = self.get_parameter("gps_right_topic").value
        gps_left = self.get_parameter("gps_left_topic").value
        raw_heading_imu = self.get_parameter("raw_heading_imu_topic").value
        heading_imu = self.get_parameter("heading_imu_topic").value
        heading_deg = self.get_parameter("heading_deg_topic").value
        odom_heading_topic = self.get_parameter("odom_heading_topic").value
        self._frame = self.get_parameter("output_frame").value
        self._corrected_yaw_offset = float(
            self.get_parameter("corrected_yaw_offset_rad").value
        )
        odom_heading_max_hz = max(
            1.0, float(self.get_parameter("odom_heading_max_hz").value)
        )
        self._odom_heading_min_period_s = 1.0 / odom_heading_max_hz
        self._last_odom_heading_pub_ns = 0
        self._min_bl = self.get_parameter("min_baseline_m").value
        self._cov = self.get_parameter("heading_covariance").value
        slop = self.get_parameter("sync_slop_s").value

        self._pub_raw_imu = (
            self.create_publisher(Imu, raw_heading_imu, _SENSOR_QOS)
            if raw_heading_imu
            else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, heading_imu, _SENSOR_QOS)
            if heading_imu
            else None
        )
        self._pub_deg = self.create_publisher(Float64, heading_deg, 10) if heading_deg else None

        self._sync = None
        if odom_heading_topic:
            self.create_subscription(
                Odometry, odom_heading_topic, self._on_odom_heading, _SENSOR_QOS
            )
        else:
            sub_r = Subscriber(self, NavSatFix, gps_right, qos_profile=_SENSOR_QOS)
            sub_l = Subscriber(self, NavSatFix, gps_left, qos_profile=_SENSOR_QOS)
            self._sync = ApproximateTimeSynchronizer(
                [sub_r, sub_l], queue_size=20, slop=slop
            )
            self._sync.registerCallback(self._on_pair)

        self.get_logger().info(
            f"dual_gps_heading: ({gps_right}, {gps_left}) "
            f"→ raw={raw_heading_imu or 'disabled'} corrected={heading_imu or 'disabled'} "
            f"odom_heading_topic={odom_heading_topic or 'disabled'}"
        )

    def _publish_heading(self, stamp, raw_heading_rad: float) -> None:
        corrected_heading_rad = raw_heading_rad + self._corrected_yaw_offset
        corrected_heading_rad = (corrected_heading_rad + math.pi) % (2 * math.pi) - math.pi

        def _build_heading_msg(yaw_rad: float) -> Imu:
            qx, qy, qz, qw = _yaw_to_quaternion(yaw_rad)
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self._frame
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            cov = [0.0] * 9
            cov[8] = self._cov
            imu_msg.orientation_covariance = cov
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0
            return imu_msg

        if self._pub_raw_imu is not None:
            self._pub_raw_imu.publish(_build_heading_msg(raw_heading_rad))
        if self._pub_imu is not None:
            self._pub_imu.publish(_build_heading_msg(corrected_heading_rad))
        if self._pub_deg is not None:
            deg_msg = Float64()
            deg_msg.data = math.degrees(corrected_heading_rad)
            self._pub_deg.publish(deg_msg)

    def _on_odom_heading(self, msg: Odometry):
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec)
        if self._last_odom_heading_pub_ns != 0:
            elapsed_ns = stamp_ns - self._last_odom_heading_pub_ns
            min_period_ns = int(self._odom_heading_min_period_s * 1_000_000_000.0)
            if elapsed_ns > 0 and elapsed_ns < min_period_ns:
                return
        raw_heading_rad = _quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        # Emulate ublox moving-baseline raw heading for a lateral antenna pair:
        # the raw vector is right->left, i.e. vehicle heading + 90 deg.
        raw_heading_rad += math.pi / 2.0
        raw_heading_rad = (raw_heading_rad + math.pi) % (2 * math.pi) - math.pi
        self._last_odom_heading_pub_ns = stamp_ns
        self._publish_heading(msg.header.stamp, raw_heading_rad)

    def _on_pair(self, fix_right: NavSatFix, fix_left: NavSatFix):
        """fix_right = gps_link (body right), fix_left = gps_link2 (body left)."""
        if fix_right.status.status < 0 or fix_left.status.status < 0:
            return

        # ENU vector from right antenna (base) to left antenna (rover), matching
        # the ublox moving-baseline convention used in real hardware.
        dx_e, dy_n = _ll_to_enu_delta(
            fix_right.latitude, fix_right.longitude,
            fix_left.latitude, fix_left.longitude,
        )

        baseline_m = math.hypot(dx_e, dy_n)
        if baseline_m < self._min_bl:
            self.get_logger().debug(
                f"Baseline {baseline_m:.3f} m < {self._min_bl} m — skipping"
            )
            return

        raw_heading_rad = math.atan2(dy_n, dx_e)
        raw_heading_rad = (raw_heading_rad + math.pi) % (2 * math.pi) - math.pi
        self._publish_heading(fix_right.header.stamp, raw_heading_rad)


def main(args=None):
    rclpy.init(args=args)
    node = DualGpsHeadingSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
