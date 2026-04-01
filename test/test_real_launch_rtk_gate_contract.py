from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_real_launch_exposes_rtk_gate_arguments() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "real.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument(\n        "require_rtk_before_navigation",' in launch_contents
    assert 'default_value="true"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "rtk_gate_timeout_s",' in launch_contents
    assert 'default_value="60.0"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "rtk_status_topic",' in launch_contents
    assert 'default_value="/gps/rtk_status"' in launch_contents
    assert 'DeclareLaunchArgument(\n        "fix_type_topic",' in launch_contents
    assert 'default_value="/gps/fix_type"' in launch_contents


def test_real_launch_runs_navigation_startup_behind_rtk_gate() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "real.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'executable="rtk_start_gate"' in launch_contents
    assert 'name="rtk_start_gate"' in launch_contents
    assert 'OnProcessExit(' in launch_contents
    assert 'EmitEvent(event=Shutdown(reason=f"RTK gate timeout after {timeout_s}s"))' in launch_contents
    assert 'OpaqueFunction(\n        function=_build_navigation_startup,' in launch_contents
    assert 'TimerAction(period=5.0, actions=list(delayed_start_actions))' in launch_contents
    assert '"timeout_s": ParameterValue(' in launch_contents

