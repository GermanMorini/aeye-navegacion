from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_localization_v2_launch_uses_fixed_dual_ekf_odom_node() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "localization_v2.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "dual_ekf_navsat_params.yaml" in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert "condition=IfCondition(ekf_local)" in launch_contents
    assert 'DeclareLaunchArgument("ekf_local", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("ekf_global", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("ukf", default_value="False")' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert "publish_odom_tf = ParameterValue(" in launch_contents
    assert '"publish_odom_tf": publish_odom_tf' in launch_contents
    assert 'executable="pixhawk_odometry"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert '("odometry/filtered", "/odometry/global")' in launch_contents
    assert "condition=IfCondition(ekf_global)" in launch_contents
    assert "localization_filter_executable = PythonExpression(" in launch_contents
    assert "\"'ukf_node' if '\"" in launch_contents
    assert "\"'.lower() == 'true' else 'ekf_node'\"" in launch_contents
    assert 'DeclareLaunchArgument("pixhawk_gps_topic", default_value="/gps/fix")' in launch_contents
    assert 'default_value="/odometry/pixhawk"' in launch_contents
    assert '"gps_topic": pixhawk_gps_topic' in launch_contents
    assert '"imu_topic": imu_topic' in launch_contents
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


def test_nav2_only_launch_disables_velocity_smoother() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert '"use_velocity_smoother": "False"' in launch_contents


def test_nav2_only_launch_defaults_keepout_mask_frame_to_odom() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "nav2_only.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "map_frame",' in launch_contents
    assert 'default_value="odom"' in launch_contents
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


def test_dual_ekf_local_uses_wheel_and_pixhawk_odometry_topics() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "ekf_filter_node_odom:" in ekf_config_contents
    assert "/wheel/odometry" in ekf_config_contents
    assert "ekf_filter_node_map:" in ekf_config_contents
    assert "odom1: /odometry/gps" in ekf_config_contents


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


def test_real_launch_includes_ackermann_odometry_by_default() -> None:
    launch_path = PACKAGE_ROOT / "launch" / "real.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "ackermann_odometry",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'executable="ackermann_odometry"' in launch_contents
    assert "PythonExpression([\"'\", ackermann_odometry, \"'.lower() == 'true'\"])" in launch_contents
    assert '"telemetry_topic": "/controller/drive_telemetry"' in launch_contents
    assert '"odom_topic": "/wheel/odometry"' in launch_contents
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
    assert "condition=IfCondition(ekf_local)" in launch_contents
    assert "condition=IfCondition(ekf_global)" in launch_contents
    assert 'name="ekf_filter_node_odom"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert "localization_filter_executable = PythonExpression(" in launch_contents
    assert "\"'ukf_node' if '\"" in launch_contents
    assert "\"'.lower() == 'true' else 'ekf_node'\"" in launch_contents
    assert "condition=IfCondition(use_navsat)" not in launch_contents
    assert 'controller_server' not in launch_contents


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
    assert "nav2_no_map_params.yaml is using frame 'map'" in launch_contents
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
    ekf_map_index = launch_contents.index("ld.add_action(ekf_map_cmd)")
    navsat_index = launch_contents.index("ld.add_action(navsat_transform_cmd)")

    assert nav2_index > ackermann_index
    assert nav2_index > ekf_map_index
    assert nav2_index > navsat_index


def test_dual_ekf_navsat_waits_for_runtime_datum() -> None:
    ekf_config_path = PACKAGE_ROOT / "config" / "dual_ekf_navsat_params.yaml"
    ekf_config_contents = ekf_config_path.read_text(encoding="utf-8")

    assert "wait_for_datum: true" in ekf_config_contents
    assert "\n    datum:" not in ekf_config_contents
