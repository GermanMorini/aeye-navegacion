from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_real_global_v2_launch_reuses_real_stack_with_global_navigation() -> None:
    launch_contents = _read("launch/real_global_v2.launch.py")

    assert "mavros.launch.py" in launch_contents
    assert "rs16.launch.py" in launch_contents
    assert 'executable="pointcloud_to_laserscan_node"' in launch_contents
    assert 'executable="controller_server_node"' in launch_contents
    assert "localization_global_v2.launch.py" in launch_contents
    assert "nav_global_v2.launch.py" in launch_contents
    assert "no_go_editor.launch.py" in launch_contents
    assert '"fromll_frame": "map"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"approx_fromll_fallback_enabled": True' in launch_contents
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lat"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lon"' in launch_contents
    assert 'DeclareLaunchArgument("datum_yaw_deg"' in launch_contents
    assert 'DeclareLaunchArgument("use_rviz", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("rviz_config", default_value=default_rviz)' in launch_contents
    assert "rviz_global_v2.rviz" in launch_contents


def test_localization_global_v2_launch_supports_datum_overrides() -> None:
    launch_contents = _read("launch/localization_global_v2.launch.py")

    assert 'DeclareLaunchArgument("datum_lat"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lon"' in launch_contents
    assert 'DeclareLaunchArgument("datum_yaw_deg"' in launch_contents
    assert "OpaqueFunction(function=_build_navsat_transform)" in launch_contents
    assert '"wait_for_datum": False' in launch_contents
    assert '"datum": [datum_lat, datum_lon, datum_yaw_rad]' in launch_contents
    assert "math.radians(datum_yaw_deg)" in launch_contents


def test_rviz_real_global_v2_launch_targets_global_config_for_local_pc() -> None:
    launch_contents = _read("launch/rviz_real_global_v2.launch.py")

    assert "rviz_global_v2.rviz" in launch_contents
    assert 'DeclareLaunchArgument(\n                "use_sim_time"' in launch_contents
    assert 'default_value="False"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "launch_robot_state_publisher"' in launch_contents
    assert "condition=IfCondition(launch_robot_state_publisher)" in launch_contents
    assert 'executable="rviz2"' in launch_contents
