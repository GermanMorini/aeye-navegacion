from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read_launch_file(name: str) -> str:
    return (PACKAGE_ROOT / "launch" / name).read_text(encoding="utf-8")


def test_sim_global_v2_launch_exposes_the_expected_public_api() -> None:
    launch_contents = _read_launch_file("sim_global_v2.launch.py")

    assert 'DeclareLaunchArgument("use_sim_time", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("use_rviz", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("rviz_config", default_value=default_rviz)' in launch_contents
    assert 'DeclareLaunchArgument("gps_profile", default_value="ideal")' in launch_contents
    assert 'DeclareLaunchArgument("world", default_value=default_world)' in launch_contents
    assert 'DeclareLaunchArgument("nav_start_delay_s", default_value="4.0")' in launch_contents
    assert '"global_localization_profile"' not in launch_contents


def test_sim_global_v2_launch_uses_global_profile_defaults() -> None:
    launch_contents = _read_launch_file("sim_global_v2.launch.py")

    assert "sim_nav_v2_base.launch.py" in launch_contents
    assert '"use_global_localization": "True"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"fromll_frame": "map"' in launch_contents
    assert '"use_keepout": "False"' in launch_contents
    assert '"nav2_params_file": nav2_params_file' in launch_contents
    assert '"global_localization_params_file": global_localization_params_file' in launch_contents
    assert '"gps_frame_id": "base_footprint"' in launch_contents


def test_sim_global_v2_launch_uses_only_the_corrected_global_localization_config() -> None:
    launch_contents = _read_launch_file("sim_global_v2.launch.py")

    assert "global_localization_v2.yaml" in launch_contents
    assert "global_localization_v2_baseline.yaml" not in launch_contents
    assert 'gps_frame_id": "base_footprint"' in launch_contents
    assert 'gps_link' not in launch_contents


def test_global_localization_launch_wires_navsat_and_map_ekf() -> None:
    launch_contents = _read_launch_file("localization_global_v2.launch.py")

    assert "localization_v2.launch.py" in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert '("odometry/gps", "/odometry/gps")' in launch_contents
    assert '("odometry/filtered", "/odometry/local")' in launch_contents
    assert "GLOBAL_LOCALIZATION_START_DELAY_S = 2.0" in launch_contents
    assert "TimerAction(" in launch_contents


def test_global_launch_uses_global_rviz_and_params() -> None:
    launch_contents = _read_launch_file("sim_global_v2.launch.py")
    params_contents = (PACKAGE_ROOT / "config" / "nav2_global_v2_params.yaml").read_text(
        encoding="utf-8"
    )
    rviz_contents = (PACKAGE_ROOT / "config" / "rviz_global_v2.rviz").read_text(
        encoding="utf-8"
    )

    assert "rviz_global_v2.rviz" in launch_contents
    assert "nav2_global_v2_params.yaml" in launch_contents
    assert "global_frame: map" in params_contents
    assert "local_frame: odom" in params_contents
    assert "odom_topic: /odometry/local" in params_contents
    assert 'filters: ["keepout_filter"]' in params_contents
    assert "Fixed Frame: map" in rviz_contents
    assert "Value: /odometry/gps" in rviz_contents


def test_global_profile_keeps_legacy_helpers_out_of_the_main_chain() -> None:
    base_contents = _read_launch_file("sim_nav_v2_base.launch.py")

    assert 'executable="cmd_vel_ackermann_bridge_v2"' not in base_contents
    assert 'executable="sim_drive_telemetry"' not in base_contents
    assert 'DeclareLaunchArgument("gps_frame_id", default_value="gps_link")' in base_contents
    assert '{"gps_frame_id": gps_frame_id}' in base_contents


def test_global_localization_config_uses_twist_only_local_odom_and_odometry_yaw() -> None:
    stable_contents = (PACKAGE_ROOT / "config" / "global_localization_v2.yaml").read_text(
        encoding="utf-8"
    )

    assert "use_odometry_yaw: true" in stable_contents
    assert (
        "odom0_config: [false, false, false,\n"
        "                   false, false, false,\n"
        "                   true,  true,  false,\n"
        "                   false, false, true,"
    ) in stable_contents
