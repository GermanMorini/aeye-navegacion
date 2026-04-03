import math

from nav_msgs.msg import Odometry
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

_POSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


def _is_finite_quaternion(msg: Imu) -> bool:
    return all(
        math.isfinite(value)
        for value in (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
    )


class ImuPoseRepublisher(Node):
    def __init__(self):
        super().__init__("imu_pose_republisher")

        self.declare_parameter("input_topic", "/imu/data")
        self.declare_parameter("output_topic", "/imu/pose")
        self.declare_parameter("output_frame", "")
        self.declare_parameter("odom_topic", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._latest_position = None
        self._latest_position_frame = ""

        self._pub = self.create_publisher(PoseStamped, output_topic, _POSE_QOS)
        self.create_subscription(Imu, input_topic, self._on_imu, _SENSOR_QOS)
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._on_odom, _SENSOR_QOS)

        self.get_logger().info(
            f"imu_pose_republisher: {input_topic} -> {output_topic} "
            f"(odom_topic={odom_topic or 'disabled'})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_position = msg.pose.pose.position
        self._latest_position_frame = msg.header.frame_id

    def _on_imu(self, msg: Imu) -> None:
        if not _is_finite_quaternion(msg):
            return

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = (
            self._output_frame
            or self._latest_position_frame
            or msg.header.frame_id
            or "base_link"
        )
        if self._latest_position is not None:
            out.pose.position = self._latest_position
        out.pose.orientation = msg.orientation
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
