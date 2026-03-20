import math

from geometry_msgs.msg import Pose

from navegacion_gps.goal_pose_to_follow_path_v2 import build_ackermann_path
from navegacion_gps.goal_pose_to_follow_path_v2 import minimum_distance_to_path_xy
from navegacion_gps.goal_pose_to_follow_path_v2 import quaternion_from_yaw
from navegacion_gps.goal_pose_to_follow_path_v2 import yaw_from_quaternion


def test_yaw_quaternion_roundtrip() -> None:
    yaw_rad = 0.73
    assert math.isclose(
        yaw_from_quaternion(quaternion_from_yaw(yaw_rad)),
        yaw_rad,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )


def test_build_ackermann_path_includes_start_and_goal() -> None:
    start_pose = Pose()
    start_pose.orientation = quaternion_from_yaw(0.0)
    goal_pose = Pose()
    goal_pose.position.x = 2.0
    goal_pose.position.y = 0.0
    goal_pose.orientation = quaternion_from_yaw(0.0)

    path = build_ackermann_path(
        start_pose=start_pose,
        goal_pose=goal_pose,
        frame_id="odom",
        step_distance_m=0.5,
        min_intermediate_poses=2,
    )

    assert path.header.frame_id == "odom"
    assert len(path.poses) >= 4
    assert math.isclose(path.poses[0].pose.position.x, 0.0)
    assert math.isclose(path.poses[-1].pose.position.x, 2.0)


def test_build_ackermann_path_uses_goal_orientation_at_end() -> None:
    start_pose = Pose()
    start_pose.orientation = quaternion_from_yaw(0.0)
    goal_pose = Pose()
    goal_pose.position.x = 0.2
    goal_pose.position.y = 0.1
    goal_pose.orientation = quaternion_from_yaw(1.2)

    path = build_ackermann_path(
        start_pose=start_pose,
        goal_pose=goal_pose,
        frame_id="odom",
        step_distance_m=1.0,
        min_intermediate_poses=0,
    )

    assert math.isclose(
        yaw_from_quaternion(path.poses[-1].pose.orientation),
        1.2,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )


def test_build_ackermann_path_respects_start_heading() -> None:
    start_pose = Pose()
    start_pose.orientation = quaternion_from_yaw(0.3)
    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 1.0
    goal_pose.orientation = quaternion_from_yaw(0.8)

    path = build_ackermann_path(
        start_pose=start_pose,
        goal_pose=goal_pose,
        frame_id="odom",
        step_distance_m=0.2,
        min_intermediate_poses=2,
    )

    assert math.isclose(
        yaw_from_quaternion(path.poses[0].pose.orientation),
        0.3,
        rel_tol=0.0,
        abs_tol=1.0e-6,
    )


def test_minimum_distance_to_path_xy_is_zero_on_path() -> None:
    start_pose = Pose()
    start_pose.orientation = quaternion_from_yaw(0.0)
    goal_pose = Pose()
    goal_pose.position.x = 2.0
    goal_pose.orientation = quaternion_from_yaw(0.0)

    path = build_ackermann_path(
        start_pose=start_pose,
        goal_pose=goal_pose,
        frame_id="odom",
        step_distance_m=0.5,
        min_intermediate_poses=2,
    )

    probe_pose = Pose()
    probe_pose.position.x = float(path.poses[1].pose.position.x)
    probe_pose.position.y = float(path.poses[1].pose.position.y)

    assert math.isclose(
        minimum_distance_to_path_xy(path, probe_pose),
        0.0,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )


def test_minimum_distance_to_path_xy_detects_lateral_offset() -> None:
    start_pose = Pose()
    start_pose.orientation = quaternion_from_yaw(0.0)
    goal_pose = Pose()
    goal_pose.position.x = 4.0
    goal_pose.orientation = quaternion_from_yaw(0.0)

    path = build_ackermann_path(
        start_pose=start_pose,
        goal_pose=goal_pose,
        frame_id="odom",
        step_distance_m=0.5,
        min_intermediate_poses=2,
    )

    probe_pose = Pose()
    probe_pose.position.x = 2.0
    probe_pose.position.y = 1.0

    assert minimum_distance_to_path_xy(path, probe_pose) > 0.9
