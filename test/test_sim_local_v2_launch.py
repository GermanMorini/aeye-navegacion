from pathlib import Path


def test_sim_local_v2_launch_uses_realistic_command_chain() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'executable="nav_command_server"' in launch_contents
    assert 'package="controller_server"' in launch_contents
    assert '"transport_backend": "sim_gazebo"' in launch_contents
    assert '"cmd_vel_final_topic": "/cmd_vel_final"' in launch_contents
    assert '"forward_cmd_vel_safe_without_goal": True' in launch_contents
    assert '"fromll_output_frame": "map"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"fromll_frame": "odom"' not in launch_contents


def test_sim_local_v2_launch_exposes_optional_bridge_mode() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_cmd_vel_ackermann_bridge", default_value="False")' in launch_contents
    assert 'executable="cmd_vel_ackermann_bridge_v2"' in launch_contents
    assert 'executable="sim_drive_telemetry"' in launch_contents
    assert 'DeclareLaunchArgument("invert_measured_steer_sign", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True")' in launch_contents


def test_sim_local_v2_launch_forwards_ekf_global_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert '"ekf_local": ekf_local' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert '"ekf_global": ekf_global' in launch_contents


def test_sim_local_v2_launch_forwards_ukf_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert '"ukf": ukf' in launch_contents


def test_sim_local_v2_launch_exposes_datum_setter_toggle() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("datum_setter", default_value="false")' in launch_contents
    assert 'executable="datum_setter"' in launch_contents
    assert "PythonExpression([\"'\", datum_setter, \"'.lower() == 'true'\"])" in launch_contents


def test_sim_local_v2_launch_forces_keepout_mask_frame_to_map() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"keepout_mask_frame": "map"' in launch_contents


def test_nav_local_v2_launch_defaults_keepout_mask_frame_to_map() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "nav_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("keepout_mask_frame", default_value="map")' in launch_contents
    assert '"keepout_mask_frame": keepout_mask_frame' in launch_contents


def test_sim_local_v2_launch_sets_imu_yaw_auto_calibration_defaults() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"imu_auto_calibrate_yaw_from_odom": True' in launch_contents
    assert '"imu_yaw_offset_rad": 0.0' in launch_contents
    assert '"imu_yaw_calib_odom_topic": "/odom_raw"' in launch_contents
    assert '"imu_yaw_calib_speed_threshold_mps": 0.05' in launch_contents
    assert '"imu_yaw_calib_timeout_s": 3.0' in launch_contents


def test_sim_local_v2_launch_disables_aggressive_gps_heading_features_by_default() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch" / "sim_local_v2.launch.py"
    )
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument(\n'
        '                "gps_course_heading_enable_consistency_filters",\n'
        '                default_value="false",\n'
        "            )"
    ) in launch_contents
    assert (
        'DeclareLaunchArgument(\n'
        '                "gps_course_heading_enable_offset_compensation",\n'
        '                default_value="false",\n'
        "            )"
    ) in launch_contents
    assert '"enable_consistency_filters": ParameterValue(' in launch_contents
    assert '"enable_offset_compensation": ParameterValue(' in launch_contents
