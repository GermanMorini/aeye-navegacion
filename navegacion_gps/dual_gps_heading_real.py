import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


def _quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw_rad):
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class DualGpsHeadingReal(Node):
    def __init__(self):
        super().__init__("heading_offset_relay")

        self.declare_parameter("input_topic", "/ublox_rover/navheading")
        self.declare_parameter("output_topic", "/dual_gps/heading")
        self.declare_parameter("yaw_offset_rad", math.pi / 2.0)
        self.declare_parameter("output_frame", "base_link")

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._offset = float(self.get_parameter("yaw_offset_rad").value)
        self._frame = str(self.get_parameter("output_frame").value)

        self._pub = self.create_publisher(Imu, out_topic, _SENSOR_QOS)
        self.create_subscription(Imu, in_topic, self._on_heading, _SENSOR_QOS)

        self.get_logger().info(
            f"heading_offset_relay: {in_topic} -> {out_topic} "
            f"(offset={math.degrees(self._offset):.1f} deg)"
        )

    def _on_heading(self, msg: Imu) -> None:
        yaw = _quat_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        yaw_corrected = (yaw + self._offset + math.pi) % (2 * math.pi) - math.pi
        ox, oy, oz, ow = _yaw_to_quaternion(yaw_corrected)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame
        out.orientation.x = ox
        out.orientation.y = oy
        out.orientation.z = oz
        out.orientation.w = ow
        out.orientation_covariance = list(msg.orientation_covariance)

        if msg.angular_velocity_covariance[0] >= 0.0:
            out.angular_velocity = msg.angular_velocity
            out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        else:
            out.angular_velocity_covariance[0] = -1.0

        if msg.linear_acceleration_covariance[0] >= 0.0:
            out.linear_acceleration = msg.linear_acceleration
            out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        else:
            out.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DualGpsHeadingReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
