from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from interfaces.msg import CmdVelFinal
from navegacion_gps.gazebo_utils import GazeboUtilsNode


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeCmdBridgeNode:
    def __init__(self, enabled: bool) -> None:
        self.cmd_vel_gazebo_pub = _FakePublisher() if enabled else None

    _publish_cmd_vel_gazebo = GazeboUtilsNode._publish_cmd_vel_gazebo


class _FakeFrameNode:
    _strip = GazeboUtilsNode._strip
    _resolve_frame = GazeboUtilsNode._resolve_frame

    def __init__(self, strip_prefix: bool) -> None:
        self.strip_prefix = strip_prefix
        self.odom_frame_id = "odom"
        self.base_link_frame_id = "base_footprint"
        self.odom_pub = _FakePublisher()


def test_cmd_vel_final_without_brake_is_forwarded() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    msg = CmdVelFinal()
    msg.twist.linear.x = 1.1
    msg.twist.angular.z = -0.4
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    assert len(node.cmd_vel_gazebo_pub.messages) == 1
    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert isinstance(published, Twist)
    assert float(published.linear.x) == 1.1
    assert float(published.angular.z) == -0.4


def test_cmd_vel_final_with_brake_publishes_zero_twist() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.9
    msg.twist.angular.z = 0.2
    msg.brake_pct = 35

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 0.0
    assert float(published.angular.z) == 0.0


def test_cmd_vel_bridge_disabled_does_not_publish() -> None:
    node = _FakeCmdBridgeNode(enabled=False)
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.8
    msg.twist.angular.z = 0.1
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    assert node.cmd_vel_gazebo_pub is None


def test_odom_callback_preserves_frame_normalization_logic() -> None:
    node = _FakeFrameNode(strip_prefix=True)
    msg = Odometry()
    msg.header.frame_id = "model::odom"
    msg.child_frame_id = "model::base_footprint"

    GazeboUtilsNode._odom_cb(node, msg)

    assert len(node.odom_pub.messages) == 1
    published = node.odom_pub.messages[-1]
    assert published.header.frame_id == "odom"
    assert published.child_frame_id == "base_footprint"
