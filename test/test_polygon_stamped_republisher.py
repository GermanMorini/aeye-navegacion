from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PolygonStamped

from navegacion_gps.polygon_stamped_republisher import PolygonStampedRepublisher


class _FakeClock:
    def now(self):
        return SimpleNamespace(
            to_msg=lambda: Time(sec=42, nanosec=123_000_000)
        )


def test_refresh_stamp_updates_header_stamp_without_changing_frame() -> None:
    node = PolygonStampedRepublisher.__new__(PolygonStampedRepublisher)
    node.get_clock = lambda: _FakeClock()

    msg = PolygonStamped()
    msg.header.frame_id = "base_footprint"
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 500

    refreshed = node._refresh_stamp(msg)

    assert refreshed.header.frame_id == "base_footprint"
    assert refreshed.header.stamp.sec == 42
    assert refreshed.header.stamp.nanosec == 123_000_000
    assert msg.header.stamp.sec == 10
