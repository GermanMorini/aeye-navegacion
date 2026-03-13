from navegacion_gps.nav_command_server import NavCommandServerNode


def test_build_loop_restart_poses_for_many_items():
    poses = [1, 2, 3, 4]
    reduced = NavCommandServerNode._build_loop_restart_poses(poses)
    assert reduced == [4, 1]


def test_build_loop_restart_poses_for_two_items():
    poses = [10, 20]
    reduced = NavCommandServerNode._build_loop_restart_poses(poses)
    assert reduced == [20, 10]


def test_build_loop_restart_poses_for_zero_or_one():
    assert NavCommandServerNode._build_loop_restart_poses([]) == []
    assert NavCommandServerNode._build_loop_restart_poses([7]) == [7]
