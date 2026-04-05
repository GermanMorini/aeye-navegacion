from pathlib import Path


def test_simulacion_launch_wraps_sim_local_v2() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "sim_local_v2.launch.py" in launch_contents
    assert 'DeclareLaunchArgument("use_cmd_vel_ackermann_bridge", default_value="False")' in launch_contents


def test_simulacion_launch_uses_requested_defaults() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "rviz_nav2_full.rviz" in launch_contents
    assert "rviz_ekf_local_tuning.rviz" in launch_contents
    assert "rviz_ekf_global_tuning.rviz" in launch_contents
    assert "rviz_local_v2.rviz" in launch_contents
    assert "nav2_no_map_params.yaml" in launch_contents
    assert "collision_monitor.yaml" in launch_contents


def test_simulacion_launch_exposes_rviz_conf_selector() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument(\n'
        '                "rviz_conf",\n'
        '                default_value="full",\n'
        '                choices=["full", "local", "global"],\n'
        "            )"
    ) in launch_contents
    assert '"rviz_config": selected_rviz_config' in launch_contents
    assert '"rviz_config": rviz_config' not in launch_contents


def test_simulacion_launch_does_not_expose_localization_overrides() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("localization_params_file"' not in launch_contents
    assert 'DeclareLaunchArgument("ekf_node_name"' not in launch_contents
    assert '"localization_params_file":' not in launch_contents
    assert '"ekf_node_name":' not in launch_contents


def test_simulacion_launch_no_longer_contains_legacy_stack() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "navigation_launch.py" not in launch_contents
    assert "gazebo_utils" not in launch_contents


def test_simulacion_launch_includes_web_zone_server_stack() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "no_go_editor.launch.py" in launch_contents
    assert 'DeclareLaunchArgument("launch_web_zone_server", default_value="True")' in launch_contents
    assert "condition=IfCondition(launch_web_zone_server)" in launch_contents
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert "resolved_map_frame = PythonExpression(" in launch_contents
    assert '"map_frame": resolved_map_frame' in launch_contents
    assert '"zones_fromll_output_frame": resolved_map_frame' in launch_contents


def test_simulacion_launch_exposes_ekf_global_toggle() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert '"ekf_local": ekf_local' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert '"ekf_global": ekf_global' in launch_contents
    assert 'DeclareLaunchArgument("enable_navsat_transform", default_value="False")' in launch_contents
    assert '"enable_navsat_transform": enable_navsat_transform' in launch_contents
    assert 'DeclareLaunchArgument("use_sim_global_overlay", default_value="False")' in launch_contents
    assert '"use_sim_global_overlay": use_sim_global_overlay' in launch_contents
    assert "resolved_nav_start_delay_s = PythonExpression(" in launch_contents
    assert '"nav_start_delay_s": resolved_nav_start_delay_s' in launch_contents
    assert "rviz_overlay_global = os.path.join" in launch_contents
    assert "use_sim_global_overlay" in launch_contents
    assert "rviz_local_v2.rviz" in launch_contents


def test_simulacion_launch_forwards_spawn_yaw_override() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("spawn_yaw_rad", default_value="auto")' in launch_contents
    assert '"spawn_yaw_rad": spawn_yaw_rad' in launch_contents


def test_simulacion_launch_forwards_ukf_toggle() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert '"ukf": ukf' in launch_contents


def test_simulacion_launch_forwards_datum_setter_toggle() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("datum_setter", default_value="false")' in launch_contents
    assert '"datum_setter": datum_setter' in launch_contents
    assert 'DeclareLaunchArgument("gps_horizontal_variance", default_value="25.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_vertical_variance", default_value="36.0")' in launch_contents
    assert "resolved_gps_horizontal_variance = PythonExpression(" in launch_contents
    assert "resolved_gps_vertical_variance = PythonExpression(" in launch_contents
    assert '"gps_horizontal_variance": resolved_gps_horizontal_variance' in launch_contents
    assert '"gps_vertical_variance": resolved_gps_vertical_variance' in launch_contents


def test_simulacion_launch_declares_dual_gps_heading_before_custom_urdf() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    declare_idx = launch_contents.index(
        'DeclareLaunchArgument("use_dual_gps_heading", default_value="true")'
    )
    custom_urdf_idx = launch_contents.index('DeclareLaunchArgument(\n                "custom_urdf",')
    assert declare_idx < custom_urdf_idx


def test_launch_sim_global_v2_wrapper_spawns_robot_facing_east() -> None:
    wrapper_path = Path(__file__).resolve().parents[3] / "tools" / "launch_sim_global_v2.sh"
    wrapper_contents = wrapper_path.read_text(encoding="utf-8")

    assert 'RVIZ_CONF="${RVIZ_CONF:-global}"' in wrapper_contents
    assert 'SPAWN_YAW_RAD="${SPAWN_YAW_RAD:-0.0}"' in wrapper_contents
    assert "spawn_yaw_rad:=${SPAWN_YAW_RAD}" in wrapper_contents
    assert 'LAUNCH_WEB_ZONE_SERVER="${LAUNCH_WEB_ZONE_SERVER:-False}"' in wrapper_contents
    assert 'USE_SIM_GLOBAL_OVERLAY="${USE_SIM_GLOBAL_OVERLAY:-True}"' in wrapper_contents
    assert "use_sim_global_overlay:=${USE_SIM_GLOBAL_OVERLAY}" in wrapper_contents
    assert 'USE_DUAL_GPS_HEADING="${USE_DUAL_GPS_HEADING:-true}"' in wrapper_contents
    assert 'GZ_HEADLESS="${GZ_HEADLESS:-false}"' in wrapper_contents
    assert "gz_headless:=${GZ_HEADLESS}" in wrapper_contents


def test_launch_sim_local_v2_wrapper_forces_dual_gps_heading() -> None:
    wrapper_path = Path(__file__).resolve().parents[3] / "tools" / "launch_sim_local_v2.sh"
    wrapper_contents = wrapper_path.read_text(encoding="utf-8")

    assert 'EXTRA_ARGS="${*:-}"' in wrapper_contents
    assert "sim_local_v2.launch.py use_dual_gps_heading:=true" in wrapper_contents
