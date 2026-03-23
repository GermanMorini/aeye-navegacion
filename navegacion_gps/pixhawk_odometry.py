from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field
from typing import Any, List, Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp_non_negative(value: float, default: float = 0.0) -> float:
    if not math.isfinite(value):
        return float(default)
    return max(float(default), float(value))


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

        self.declare_parameter("input_odom_topic", "/odom_raw")
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

        input_odom_topic = str(self.get_parameter("input_odom_topic").value)
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

        self._latest_msg: Optional[Odometry] = None
        self._last_update_time_s: Optional[float] = None

        self._delay_queue = DelayedMessageQueue()
        self._odom_pub = self.create_publisher(Odometry, output_odom_topic, 10)
        self.create_subscription(Odometry, input_odom_topic, self._on_input_odom, 10)
        self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            "pixhawk_odometry ready "
            f"({input_odom_topic} -> {output_odom_topic}, "
            f"publish_hz={publish_hz:.1f}, latency_s={self._latency_s:.3f}, seed={random_seed})"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _on_input_odom(self, msg: Odometry) -> None:
        self._latest_msg = msg

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

    def _build_noisy_odom(self, in_msg: Odometry) -> Odometry:
        out = Odometry()
        out.header.stamp = in_msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame

        in_x = float(in_msg.pose.pose.position.x)
        in_y = float(in_msg.pose.pose.position.y)
        in_z = float(in_msg.pose.pose.position.z)

        if not (math.isfinite(in_x) and math.isfinite(in_y) and math.isfinite(in_z)):
            return out

        out.pose.pose.position.x = add_gaussian_noise(
            in_x + self._bias.position_x_m,
            self._position_std_m,
            self._rng,
        )
        out.pose.pose.position.y = add_gaussian_noise(
            in_y + self._bias.position_y_m,
            self._position_std_m,
            self._rng,
        )
        out.pose.pose.position.z = add_gaussian_noise(
            in_z + self._bias.position_z_m,
            self._position_std_m,
            self._rng,
        )

        in_yaw = yaw_from_quaternion(in_msg.pose.pose.orientation)
        noisy_yaw = wrap_angle(
            add_gaussian_noise(
                in_yaw + self._bias.yaw_rad,
                self._yaw_std_rad,
                self._rng,
            )
        )
        out.pose.pose.orientation = quaternion_from_yaw(noisy_yaw)
        out.pose.covariance = list(self._pose_covariance)

        vx = float(in_msg.twist.twist.linear.x)
        vy = float(in_msg.twist.twist.linear.y)
        vz = float(in_msg.twist.twist.linear.z)
        wz = float(in_msg.twist.twist.angular.z)

        out.twist.twist.linear.x = add_gaussian_noise(
            vx + self._bias.velocity_x_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.linear.y = add_gaussian_noise(
            vy + self._bias.velocity_y_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.linear.z = add_gaussian_noise(
            vz + self._bias.velocity_z_mps,
            self._velocity_std_mps,
            self._rng,
        )
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = float(wz) + float(self._bias.yaw_rate_rps)
        out.twist.covariance = list(self._twist_covariance)

        return out

    def _on_timer(self) -> None:
        now_s = self._now_s()
        if self._last_update_time_s is None:
            self._last_update_time_s = now_s
        dt_s = max(0.0, now_s - float(self._last_update_time_s))
        self._last_update_time_s = now_s

        if self._latest_msg is not None:
            self._update_bias_state(dt_s)
            noisy_msg = self._build_noisy_odom(self._latest_msg)
            self._delay_queue.push(release_time_s=now_s + self._latency_s, msg=noisy_msg)
            self._latest_msg = None

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
