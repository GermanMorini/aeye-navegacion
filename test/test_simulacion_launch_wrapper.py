from pathlib import Path


def test_simulacion_launch_wraps_sim_local_v2() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "sim_local_v2.launch.py" in launch_contents
    assert 'DeclareLaunchArgument("use_cmd_vel_ackermann_bridge", default_value="False")' in launch_contents


def test_simulacion_launch_uses_requested_defaults() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert "rviz_ekf_local_tuning.rviz" in launch_contents
    assert "rviz_ekf_global_tuning.rviz" in launch_contents
    assert "nav2_no_map_params.yaml" in launch_contents
    assert "collision_monitor.yaml" in launch_contents


def test_simulacion_launch_exposes_rviz_conf_selector() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "simulacion.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("rviz_conf", default_value="local", choices=["local", "global"])' in launch_contents
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
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert '"map_frame": "odom"' in launch_contents
