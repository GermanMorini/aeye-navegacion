import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from navegacion_gps.nav_command_server import NavCommandServerNode


def test_build_loop_restart_poses_for_many_items():
    poses = [1, 2, 3, 4]
    restarted = NavCommandServerNode._build_loop_restart_poses(poses)
    assert restarted == [2, 3, 4, 1]


def test_build_loop_restart_poses_for_two_items():
    poses = [10, 20]
    restarted = NavCommandServerNode._build_loop_restart_poses(poses)
    assert restarted == [20, 10]


def test_build_loop_restart_poses_for_zero_or_one():
    assert NavCommandServerNode._build_loop_restart_poses([]) == []
    assert NavCommandServerNode._build_loop_restart_poses([7]) == [7]


class _FakeLogger:
    def __init__(self):
        self.info_msgs = []
        self.warn_msgs = []

    def info(self, msg: str) -> None:
        self.info_msgs.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warn_msgs.append(str(msg))


class _FakeResultFuture:
    def __init__(self, status: int, missed_waypoints=None):
        if missed_waypoints is None:
            missed_waypoints = []
        self._result = SimpleNamespace(
            status=int(status),
            result=SimpleNamespace(missed_waypoints=list(missed_waypoints)),
        )

    def result(self):
        return self._result


class _FakeLoopNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._current_goal_handle = object()
        self._loop_enabled = True
        self._loop_waypoint_poses = [1, 2]
        self._loop_original_poses = [1, 2, 3]
        self._loop_restart_poses = [2, 3, 1]
        self._manual_enabled = False
        self._is_navigating = True
        self._auto_mode = "loop"

        self._send_ok = True
        self._send_err = ""
        self.sent_calls = []
        self.telemetry_forced = []
        self.brake_calls = []
        self.logger = _FakeLogger()

    def _send_nav_goal_for_poses(self, poses, loop_enabled, reason):
        self.sent_calls.append((list(poses), bool(loop_enabled), str(reason)))
        return bool(self._send_ok), str(self._send_err)

    def _publish_telemetry(self, force=False):
        self.telemetry_forced.append(bool(force))

    def _clear_loop_config_locked(self) -> None:
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_restart_poses = []
        self._loop_enabled = False

    def _publish_brake_sequence(self, brake_pct: int) -> None:
        self.brake_calls.append(int(brake_pct))

    def get_logger(self):
        return self.logger


class _FakeAsyncResultFuture:
    def add_done_callback(self, _callback) -> None:
        return None


class _FakeGoalHandle:
    def __init__(self) -> None:
        self.accepted = True

    def get_result_async(self):
        return _FakeAsyncResultFuture()


class _FakeGoalFuture:
    def __init__(self, result) -> None:
        self.result = result


class _FakeActionClient:
    def __init__(self) -> None:
        self.last_goal = None
        self.last_timeout = None
        self._goal_handle = _FakeGoalHandle()

    def wait_for_server(self, timeout_sec: float) -> bool:
        self.last_timeout = float(timeout_sec)
        return True

    def send_goal_async(self, goal):
        self.last_goal = goal
        return _FakeGoalFuture(self._goal_handle)


class _FakeSendNode:
    _prepare_poses_for_nav2 = NavCommandServerNode._prepare_poses_for_nav2
    _send_follow_waypoints_goal = NavCommandServerNode._send_follow_waypoints_goal
    _send_navigate_through_poses_goal = NavCommandServerNode._send_navigate_through_poses_goal

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.map_frame = "map"
        self._current_goal_handle = None
        self._loop_waypoint_poses = []
        self._loop_enabled = False
        self._is_navigating = False
        self._auto_mode = "idle"
        self._nav_result_event_id = 0
        self._follow_waypoints_client = _FakeActionClient()
        self._navigate_through_poses_client = _FakeActionClient()
        self.logger = _FakeLogger()

    def _wait_for_future(self, future, timeout_sec: float):
        _ = timeout_sec
        return future.result

    def _publish_telemetry(self, force: bool = False) -> None:
        _ = force

    def _on_nav_action_result_done(self, _action_name, _future) -> None:
        return None

    def get_logger(self):
        return self.logger


class _FakeClockNow:
    def __init__(self, sec: int = 999, nanosec: int = 123) -> None:
        self._stamp = Time(sec=int(sec), nanosec=int(nanosec))

    def to_msg(self) -> Time:
        return Time(sec=int(self._stamp.sec), nanosec=int(self._stamp.nanosec))


class _FakeClock:
    def now(self) -> _FakeClockNow:
        return _FakeClockNow()


class _FakeTransformBuffer:
    def __init__(self, transformed_pose: PoseStamped | None = None) -> None:
        self.transformed_pose = transformed_pose
        self.calls = []

    def transform(self, pose: PoseStamped, target_frame: str, timeout):
        self.calls.append((pose, str(target_frame), timeout))
        if self.transformed_pose is None:
            raise AssertionError("unexpected transform call without prepared pose")
        return self.transformed_pose


class _FakeTransformNode:
    _transform_pose_to_map = NavCommandServerNode._transform_pose_to_map

    def __init__(self, map_frame: str, transformed_pose: PoseStamped | None = None) -> None:
        self.map_frame = str(map_frame)
        self.tf_lookup_timeout_s = 0.5
        self._tf_buffer = _FakeTransformBuffer(transformed_pose=transformed_pose)
        self._last_fromll_error = None

    def get_clock(self):
        return _FakeClock()


class _FakeBuildPoseNode:
    _build_pose_from_ll = NavCommandServerNode._build_pose_from_ll

    def __init__(self, fromll_output_frame: str) -> None:
        self.fromll_output_frame = str(fromll_output_frame)
        self.captured_pose = None

    def _call_from_ll(self, lat: float, lon: float):
        assert lat == -31.0
        assert lon == -64.0
        return (1.5, -2.5, 0.0)

    def _project_geographic_yaw_to_fromll(self, lat: float, lon: float, yaw_deg: float, converted):
        assert lat == -31.0
        assert lon == -64.0
        assert yaw_deg == 90.0
        assert converted == (1.5, -2.5, 0.0)
        return 12.0

    def _transform_pose_to_map(self, pose: PoseStamped):
        self.captured_pose = pose
        return pose


def _pose(frame_id: str, sec: int, nanosec: int = 0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = str(frame_id)
    msg.header.stamp = Time(sec=int(sec), nanosec=int(nanosec))
    msg.pose.orientation.w = 1.0
    return msg


def test_prepare_poses_for_nav2_clones_and_normalizes_stamps_and_frame() -> None:
    node = SimpleNamespace(map_frame="map")
    original_a = _pose("map", sec=159, nanosec=123)
    original_b = _pose("", sec=160, nanosec=456)

    prepared = NavCommandServerNode._prepare_poses_for_nav2(node, [original_a, original_b])

    assert len(prepared) == 2
    assert prepared[0] is not original_a
    assert prepared[1] is not original_b
    assert prepared[0].header.stamp.sec == 0
    assert prepared[0].header.stamp.nanosec == 0
    assert prepared[1].header.stamp.sec == 0
    assert prepared[1].header.stamp.nanosec == 0
    assert prepared[0].header.frame_id == "map"
    assert prepared[1].header.frame_id == "map"

    assert original_a.header.stamp.sec == 159
    assert original_a.header.stamp.nanosec == 123
    assert original_b.header.stamp.sec == 160
    assert original_b.header.stamp.nanosec == 456
    assert original_b.header.frame_id == ""


def test_send_navigate_through_poses_goal_uses_prepared_pose_copies() -> None:
    node = _FakeSendNode()
    original_poses = [_pose("map", sec=159, nanosec=1), _pose("", sec=160, nanosec=2)]

    ok, err = NavCommandServerNode._send_navigate_through_poses_goal(
        node,
        original_poses,
        loop_enabled=True,
        reason="loop_restart_rotated",
    )

    assert ok is True
    assert err == "goal accepted"
    sent_poses = node._navigate_through_poses_client.last_goal.poses
    assert len(sent_poses) == 2
    assert sent_poses[0] is not original_poses[0]
    assert sent_poses[1] is not original_poses[1]
    assert sent_poses[0].header.stamp.sec == 0
    assert sent_poses[1].header.stamp.sec == 0
    assert sent_poses[1].header.frame_id == "map"
    assert original_poses[0].header.stamp.sec == 159
    assert original_poses[1].header.stamp.sec == 160


def test_send_follow_waypoints_goal_uses_prepared_pose_copies() -> None:
    node = _FakeSendNode()
    original_poses = [_pose("", sec=170, nanosec=10)]

    ok, err = NavCommandServerNode._send_follow_waypoints_goal(
        node,
        original_poses,
        loop_enabled=False,
        reason="set_goal_service",
    )

    assert ok is True
    assert err == "goal accepted"
    sent_poses = node._follow_waypoints_client.last_goal.poses
    assert len(sent_poses) == 1
    assert sent_poses[0] is not original_poses[0]
    assert sent_poses[0].header.stamp.sec == 0
    assert sent_poses[0].header.stamp.nanosec == 0
    assert sent_poses[0].header.frame_id == "map"
    assert original_poses[0].header.stamp.sec == 170


def test_transform_pose_to_map_skips_tf_when_pose_already_in_target_frame() -> None:
    node = _FakeTransformNode(map_frame="map")
    pose = _pose("map", sec=170, nanosec=10)

    transformed = NavCommandServerNode._transform_pose_to_map(node, pose)

    assert transformed is pose
    assert transformed.header.frame_id == "map"
    assert transformed.header.stamp.sec == 999
    assert transformed.header.stamp.nanosec == 123
    assert node._tf_buffer.calls == []


def test_transform_pose_to_map_uses_tf_buffer_when_fromll_output_frame_differs() -> None:
    pose = _pose("map", sec=170, nanosec=10)
    expected = _pose("odom", sec=0, nanosec=0)
    expected.pose.position.x = 12.0
    expected.pose.position.y = -4.0
    node = _FakeTransformNode(map_frame="odom", transformed_pose=expected)

    transformed = NavCommandServerNode._transform_pose_to_map(node, pose)

    assert transformed is expected
    assert transformed.header.frame_id == "odom"
    assert transformed.header.stamp.sec == 999
    assert transformed.header.stamp.nanosec == 123
    assert len(node._tf_buffer.calls) == 1
    assert node._tf_buffer.calls[0][0] is pose
    assert node._tf_buffer.calls[0][1] == "odom"


def test_build_pose_from_ll_uses_fromll_output_frame_for_intermediate_pose() -> None:
    node = _FakeBuildPoseNode(fromll_output_frame="map")

    pose = NavCommandServerNode._build_pose_from_ll(node, -31.0, -64.0, 90.0)

    assert pose is node.captured_pose
    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == 1.5
    assert pose.pose.position.y == -2.5
    assert pose.pose.orientation.w != 0.0


def test_send_nav_goal_for_poses_multi_pose_path_uses_zero_stamp_latest() -> None:
    node = _FakeSendNode()
    original_poses = [_pose("map", sec=200, nanosec=1), _pose("map", sec=201, nanosec=2)]

    ok, err = NavCommandServerNode._send_nav_goal_for_poses(
        node,
        original_poses,
        loop_enabled=True,
        reason="loop_restart_rotated",
    )

    assert ok is True
    assert err == "goal accepted"
    sent_poses = node._navigate_through_poses_client.last_goal.poses
    assert len(sent_poses) == 2
    assert sent_poses[0].header.stamp.sec == 0
    assert sent_poses[1].header.stamp.sec == 0
    assert original_poses[0].header.stamp.sec == 200
    assert original_poses[1].header.stamp.sec == 201


def test_result_callback_restarts_with_rotated_path_on_each_success():
    node = _FakeLoopNode()

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert node.sent_calls[0] == ([2, 3, 1], True, "loop_restart_rotated")
    assert node._is_navigating is True
    assert node._auto_mode == "loop"

    node._current_goal_handle = object()
    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert node.sent_calls[1] == ([2, 3, 1], True, "loop_restart_rotated")
    assert node._is_navigating is True
    assert node._auto_mode == "loop"


def test_result_callback_stops_loop_when_status_not_succeeded():
    node = _FakeLoopNode()

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert node._loop_enabled is False


def test_result_callback_stops_loop_when_restart_send_fails():
    node = _FakeLoopNode()
    node._send_ok = False
    node._send_err = "goal rejected by NavigateThroughPoses"

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert len(node.sent_calls) == 1
    assert node._loop_enabled is False
    assert node._loop_original_poses == []
    assert node._loop_restart_poses == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert any("Loop restart failed" in msg for msg in node.logger.warn_msgs)


def test_result_callback_point_to_point_stops_on_success():
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._loop_original_poses = []
    node._loop_restart_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"


def test_result_callback_point_to_point_manual_mode_does_not_brake():
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._manual_enabled = True
    node._loop_original_poses = []
    node._loop_restart_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == []
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
