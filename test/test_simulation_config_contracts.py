from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_localization_v2_launch_uses_fixed_dual_ekf_odom_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "localization_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "dual_ekf_navsat_params.yaml" in launch_contents
    assert "dual_ekf_navsat_sim_global_overlay.yaml" in launch_contents
    assert 'DeclareLaunchArgument("use_sim_global_overlay", default_value="False")' in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert "if ekf_local:" in launch_contents
    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("enable_navsat_transform", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert 'ackermann_odom_topic = "/wheel/odometry" if ekf_local else "/odometry/local"' in launch_contents
    assert '"odom_topic": ackermann_odom_topic' in launch_contents
    assert "publish_odom_tf = not ekf_local" in launch_contents
    assert '"periodic_log_enabled": False' in launch_contents
    assert '"publish_odom_tf": publish_odom_tf' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert '("odometry/filtered", "/odometry/local")' in launch_contents
    assert launch_contents.count('("odometry/filtered", "/odometry/local")') >= 2
    assert "if ekf_global or enable_navsat_transform:" in launch_contents
    assert "if ekf_global:" in launch_contents
    assert 'DeclareLaunchArgument("global_start_delay_s", default_value="0.0")' in launch_contents
    assert "TimerAction(period=global_start_delay_s, actions=global_nodes)" in launch_contents
    assert "if use_sim_time and use_sim_global_overlay:" in launch_contents
    assert 'executable = "ukf_node" if use_ukf else "ekf_node"' in launch_contents
    assert "pixhawk_input_odom_topic" not in launch_contents
    assert 'DeclareLaunchArgument(\n                "localization_params_file",' not in launch_contents
    assert 'DeclareLaunchArgument(\n                "ekf_node_name",' not in launch_contents


def test_nav_local_v2_does_not_use_frame_override_overlay() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav_local_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "nav2_no_map_local_frame_overrides.yaml" not in launch_contents
    assert "configured_nav2_frame_overrides" not in launch_contents
    assert "nav2_local_v2_keepout_overrides.yaml" not in launch_contents
    assert "nav2_local_v2_no_keepout_overrides.yaml" not in launch_contents
    assert "configured_nav2_overrides" not in launch_contents


def test_nav_local_v2_rewrites_keepout_enable_flags_from_launch_arg() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav_local_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        '"local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )
    assert (
        '"global_costmap.global_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )


def test_nav2_only_launch_disables_velocity_smoother() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"use_velocity_smoother": "False"' in launch_contents


def test_nav2_only_launch_defaults_keepout_mask_frame_to_odom() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "map_frame",' in launch_contents
    assert 'default_value="odom"' in launch_contents
    assert '"bt_navigator.ros__parameters.global_frame": resolved_map_frame' in launch_contents
    assert '"behavior_server.ros__parameters.global_frame": resolved_map_frame' in launch_contents
    assert '"global_costmap.global_costmap.ros__parameters.global_frame": resolved_map_frame' in launch_contents
    assert '"frame_id": resolved_map_frame' in launch_contents
    assert "not in ('', 'auto') else 'odom'" in launch_contents


def test_nav2_only_launch_exposes_use_keepout_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "use_keepout",' in launch_contents
    assert 'default_value="True"' in launch_contents
    assert "condition=IfCondition(use_keepout)" in launch_contents
    assert (
        '"local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )
    assert (
        '"global_costmap.global_costmap.ros__parameters.keepout_filter.enabled": use_keepout'
        in launch_contents
    )


def test_nav2_no_map_direct_frame_values() -> None:
    nav2_config_path = PACKAGE_ROOT / "config" / "nav2_no_map_params.yaml"
    nav2_config_contents = nav2_config_path.read_text(encoding="utf-8")

    assert "bt_navigator:" in nav2_config_contents
    assert "behavior_server:" in nav2_config_contents
    assert "global_costmap:" in nav2_config_contents
    assert "global_frame: map" in nav2_config_contents
    assert nav2_config_contents.count("odom_topic: /odometry/local") == 3
    assert "odom_topic: /odometry/filtered" not in nav2_config_contents


def test_nav2_no_map_uses_sim_german_speed_floor_for_ackermann() -> None:
    nav2_config_path = PACKAGE_ROOT / "config" / "nav2_no_map_params.yaml"
    nav2_config_contents = nav2_config_path.read_text(encoding="utf-8")

    assert "xy_goal_tolerance: 0.8" in nav2_config_contents
    assert nav2_config_contents.count("minimum_turning_radius: 2.6") == 2
    assert "desired_linear_vel: 1.2" in nav2_config_contents
    assert "lookahead_dist: 2.8" in nav2_config_contents
    assert "min_lookahead_dist: 1.0" in nav2_config_contents
    assert "max_lookahead_dist: 5.0" in nav2_config_contents
    assert "lookahead_time: 2.2" in nav2_config_contents
    assert "approach_velocity_scaling_dist: 2.0" in nav2_config_contents
    assert "regulated_linear_scaling_min_radius: 5.0" in nav2_config_contents
    assert "max_robot_pose_search_dist: 5.0" in nav2_config_contents
    assert "min_approach_linear_velocity: 1.0" in nav2_config_contents
    assert "regulated_linear_scaling_min_speed: 1.0" in nav2_config_contents
    assert "keep_goal_orientation: true" in nav2_config_contents


def test_dual_ekf_local_uses_wheel_and_pixhawk_odometry_topics() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "ekf_filter_node_odom:" in ekf_config_contents
    assert "/wheel/odometry" in ekf_config_contents
    assert "ekf_filter_node_map:" in ekf_config_contents
    assert "odom0: /odometry/local" in ekf_config_contents
    assert "odom1: /odometry/gps" in ekf_config_contents
    assert "false, false, true," in ekf_config_contents


def test_real_launch_includes_datum_setter_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "datum_setter",' in launch_contents
    assert "default_value=\"true\"" in launch_contents
    assert 'executable="datum_setter"' in launch_contents
    assert '"set_datum_service": "/datum_setter/set_datum"' in launch_contents
    assert '"get_datum_service": "/datum_setter/get_datum"' in launch_contents
    assert '"datum_service": "/datum"' in launch_contents
    assert '"datum_service_fallback": "/navsat_transform/datum"' in launch_contents
    assert "PythonExpression([\"'\", datum_setter, \"'.lower() == 'true'\"])" in launch_contents


def test_nav_command_server_launches_expose_goal_arrival_radius() -> None:
    sim_launch_path = PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    real_launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"

    sim_launch_contents = sim_launch_path.read_text(encoding="utf-8")
    real_launch_contents = real_launch_path.read_text(encoding="utf-8")

    assert "resolved_goal_arrival_radius_m = PythonExpression(" in sim_launch_contents
    assert "'0.5' if '" in sim_launch_contents
    assert '"goal_arrival_radius_m": ParameterValue(' in sim_launch_contents
    assert '"goal_arrival_radius_m": 0.8' in real_launch_contents


def test_sim_launch_defaults_deweight_simulated_gps_for_global_ekf() -> None:
    sim_local_launch_path = PACKAGE_ROOT / "launch" / "sim_local_v2.launch.py"
    simulacion_launch_path = PACKAGE_ROOT / "launch" / "simulacion.launch.py"

    sim_local_contents = sim_local_launch_path.read_text(encoding="utf-8")
    simulacion_contents = simulacion_launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("gps_horizontal_variance", default_value="25.0")' in sim_local_contents
    assert 'DeclareLaunchArgument("gps_vertical_variance", default_value="36.0")' in sim_local_contents
    assert '"gps_horizontal_variance": ParameterValue(' in sim_local_contents
    assert '"gps_vertical_variance": ParameterValue(' in sim_local_contents
    assert 'DeclareLaunchArgument("gps_horizontal_variance", default_value="25.0")' in simulacion_contents
    assert 'DeclareLaunchArgument("gps_vertical_variance", default_value="36.0")' in simulacion_contents
    assert "resolved_gps_horizontal_variance = PythonExpression(" in simulacion_contents
    assert "resolved_gps_vertical_variance = PythonExpression(" in simulacion_contents
    assert "'900.0' if '" in simulacion_contents
    assert simulacion_contents.count("'900.0' if '") >= 2
    assert '"gps_horizontal_variance": resolved_gps_horizontal_variance' in simulacion_contents
    assert '"gps_vertical_variance": resolved_gps_vertical_variance' in simulacion_contents


def test_sim_launch_uses_global_odom_projection_for_web_gateway() -> None:
    simulacion_launch_path = PACKAGE_ROOT / "launch" / "simulacion.launch.py"
    simulacion_contents = simulacion_launch_path.read_text(encoding="utf-8")

    assert "web_odom_topic = PythonExpression(" in simulacion_contents
    assert "web_project_odom_to_geodetic = PythonExpression(" in simulacion_contents
    assert "web_launch_zones_manager = PythonExpression(" in simulacion_contents
    assert '"odom_topic": web_odom_topic' in simulacion_contents
    assert '"project_odom_to_geodetic": web_project_odom_to_geodetic' in simulacion_contents
    assert '"reload_zones_on_connect": "false"' in simulacion_contents
    assert '"launch_zones_manager": web_launch_zones_manager' in simulacion_contents
    assert "'/odometry/local' if '" in simulacion_contents
    assert "else ('/odometry/global' if '" in simulacion_contents
    assert "enable_navsat_transform" in simulacion_contents
    assert "'.lower() == 'true' or '" in simulacion_contents


def test_sim_launch_uses_overlay_rviz_when_sim_global_overlay_is_enabled() -> None:
    simulacion_launch_path = PACKAGE_ROOT / "launch" / "simulacion.launch.py"
    simulacion_contents = simulacion_launch_path.read_text(encoding="utf-8")

    assert "rviz_overlay_global = os.path.join" in simulacion_contents
    assert "rviz_local_v2.rviz" in simulacion_contents
    assert "'.lower() == 'true' and '" in simulacion_contents
    assert "' == 'global' else '" in simulacion_contents


def test_sim_global_overlay_keeps_relative_local_pose_for_map_ekf() -> None:
    overlay_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_sim_global_overlay.yaml"
    overlay_contents = overlay_path.read_text(encoding="utf-8")

    assert "ekf_filter_node_map:" in overlay_contents
    assert "dynamic_process_noise_covariance: false" in overlay_contents
    assert "odom0: /wheel/odometry" in overlay_contents
    assert "odom0_config: [false, false, false," in overlay_contents
    assert "odom0_relative: false" in overlay_contents
    assert "odom1: /odom" in overlay_contents
    assert "odom1_config: [true,  true,  false," in overlay_contents
    assert "odom1_relative: false" in overlay_contents
    assert "navsat_transform:" in overlay_contents
    assert "odom_topic: /odom" in overlay_contents


def test_real_launch_includes_ackermann_odometry_by_default() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "ackermann_odometry",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert "PythonExpression([\"'\", ackermann_odometry, \"'.lower() == 'true'\"])" in launch_contents
    assert '"telemetry_topic": "/controller/drive_telemetry"' in launch_contents
    assert "'/wheel/odometry' if '" in launch_contents
    assert "else '/odometry/local'" in launch_contents
    assert '"publish_odom_tf": ParameterValue(' in launch_contents
    assert "PythonExpression([\"'\", ekf_local, \"'.lower() != 'true'\"])" in launch_contents


def test_real_launch_includes_zones_manager_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "zones_manager",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'executable="zones_manager"' in launch_contents
    assert "PythonExpression([\"'\", zones_manager, \"'.lower() == 'true'\"])" in launch_contents
    assert '"use_keepout": PythonExpression(["\'", zones_manager, "\'.lower() == \'true\'"])' in launch_contents


def test_real_launch_exposes_dual_ekf_toggles_and_no_controller_server() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "ekf_local",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "ekf_global",' in launch_contents
    assert 'DeclareLaunchArgument(\n        "ukf",' in launch_contents
    assert "if ekf_local:" in launch_contents
    assert "if ekf_global:" in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert 'executable = "ukf_node" if use_ukf else "ekf_node"' in launch_contents
    assert "condition=IfCondition(use_navsat)" not in launch_contents
    assert 'controller_server' not in launch_contents


def test_real_launch_enables_dual_gps_heading_by_default() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument(\n        "use_dual_gps_heading",\n        default_value="true",'
        in launch_contents
    )
    assert 'os.path.join(gps_wpf_dir, "launch", "dual_gps_heading_hw.launch.py")' in launch_contents
    assert 'PythonExpression(["\'", use_dual_gps_heading, "\'.lower() == \'true\'"])' in launch_contents
    assert "def _validate_real_dual_gps_heading_required(context):" in launch_contents
    assert "use_dual_gps_heading:=true" in launch_contents
    assert "uses ublox_gps heading" in launch_contents


def test_real_launch_auto_resolves_map_frame_from_ekf_global_toggle() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "map_frame",' in launch_contents
    assert 'default_value="auto"' in launch_contents
    assert "resolved_map_frame = PythonExpression(" in launch_contents
    assert "else ('map' if '" in launch_contents
    assert "'.lower() == 'true' else 'odom'))" in launch_contents
    assert '"map_frame": resolved_map_frame,' in launch_contents
    assert '"fromll_target_frame": resolved_map_frame,' in launch_contents


def test_real_launch_includes_tf_consistency_fail_fast_validation() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "def _validate_tf_configuration(context, nav2_params_file: str):" in launch_contents
    assert "navigation frame resolves to 'map'" in launch_contents
    assert "Para modo map: usar `ekf_global:=True`." in launch_contents
    assert "ekf_local:=False and ackermann_odometry:=False" in launch_contents
    assert "Si desactivas `ekf_local`, mantener `ackermann_odometry:=true`." in launch_contents
    assert "OpaqueFunction(" in launch_contents
    assert "_validate_tf_configuration(context, nav2_params_file)" in launch_contents


def test_real_launch_starts_nav2_after_tf_providers_in_global_only_mode() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    nav2_index = launch_contents.index("ld.add_action(nav2_only_cmd)")
    ackermann_index = launch_contents.index("ld.add_action(ackermann_odometry_cmd)")
    ekf_index = launch_contents.index("ld.add_action(ekf_nodes_cmd)")

    assert nav2_index > ackermann_index
    assert nav2_index > ekf_index


def test_dual_ekf_navsat_waits_for_runtime_datum() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "wait_for_datum: true" in ekf_config_contents
    assert "\n    datum:" not in ekf_config_contents


def test_rviz_goal_tools_route_through_nav_command_server() -> None:
    rviz_files = [
        PACKAGE_ROOT / "config" / "rviz_nav2_full.rviz",
        PACKAGE_ROOT / "config" / "rviz_local_v2.rviz",
        PACKAGE_ROOT / "config" / "rviz_ekf_local_tuning.rviz",
        PACKAGE_ROOT / "config" / "rviz_ekf_global_tuning.rviz",
    ]

    for rviz_path in rviz_files:
        contents = rviz_path.read_text(encoding="utf-8")
        assert "Value: /nav_command_server/goal_pose" in contents
        assert "Value: /goal_pose" not in contents
