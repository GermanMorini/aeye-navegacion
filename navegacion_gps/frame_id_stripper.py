import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2


class FrameIdStripper(Node):
    def __init__(self):
        super().__init__("frame_id_stripper")
        self.declare_parameter("strip_prefix", True)

        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("imu_out_topic", "/imu/data")
        self.declare_parameter("imu_frame_id", "imu_link")

        self.declare_parameter("gps_in_topic", "/gps/fix_raw")
        self.declare_parameter("gps_out_topic", "/gps/fix")
        self.declare_parameter("gps_frame_id", "gps_link")

        self.declare_parameter("lidar_in_topic", "/scan_3d_raw")
        self.declare_parameter("lidar_out_topic", "/scan_3d")
        self.declare_parameter("lidar_frame_id", "lidar_link")

        self.declare_parameter("odom_in_topic", "/odom_raw")
        self.declare_parameter("odom_out_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_link_frame_id", "base_footprint")

        self.strip_prefix = self.get_parameter("strip_prefix").value

        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.gps_frame_id = self.get_parameter("gps_frame_id").value
        self.lidar_frame_id = self.get_parameter("lidar_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value

        imu_in_topic = self.get_parameter("imu_in_topic").value
        imu_out_topic = self.get_parameter("imu_out_topic").value
        gps_in_topic = self.get_parameter("gps_in_topic").value
        gps_out_topic = self.get_parameter("gps_out_topic").value
        lidar_in_topic = self.get_parameter("lidar_in_topic").value
        lidar_out_topic = self.get_parameter("lidar_out_topic").value
        odom_in_topic = self.get_parameter("odom_in_topic").value
        odom_out_topic = self.get_parameter("odom_out_topic").value

        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, gps_out_topic, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_out_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)

        self.create_subscription(Imu, imu_in_topic, self._imu_cb, 10)
        self.create_subscription(NavSatFix, gps_in_topic, self._gps_cb, 10)
        self.create_subscription(PointCloud2, lidar_in_topic, self._lidar_cb, 10)
        self.create_subscription(Odometry, odom_in_topic, self._odom_cb, 10)

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

    def _gps_cb(self, msg: NavSatFix):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.gps_frame_id)
        self.gps_pub.publish(msg)

    def _lidar_cb(self, msg: PointCloud2):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.lidar_frame_id)
        self.lidar_pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        msg.header.frame_id = self._resolve_frame(msg.header.frame_id, self.odom_frame_id)
        msg.child_frame_id = self._resolve_frame(msg.child_frame_id, self.base_link_frame_id)
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = FrameIdStripper()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
