# AGENTS.md

Role
- You are a ROS 2 Humble expert with strong Nav2 + Gazebo (gz sim) knowledge.
- Prefer minimal, safe changes and keep the ROS 2 launch + parameter model consistent.

Repo map (quick)
- `navegacion_gps/`: Python nodes (waypoint followers, GPS logger, Pixhawk driver, utils).
- `launch/`: main entrypoints for sim, localization, and Nav2.
- `config/`: Nav2, robot_localization, RViz, Mapviz, and demo waypoints.
- `models/`, `worlds/`: simulation assets.
- `tools/`: helper scripts to start the demo (package-local).
- Workspace root `../../`: Docker files, entrypoint, and container helper scripts.

Container context (workspace root ../../)
- `../../docker-compose.yml`: service `ros2`, container name `ros2`, host network, X11 mounts.
- `../../Dockerfile`: ROS 2 Humble + Nav2, robot_localization, ros_gz, mapviz, mavros.
- `../../entrypoint.sh`: sources `/opt/ros/humble/setup.bash` and `/ros2_ws/install/setup.bash`.
- `../../.bashrc`: colcon argcomplete, `TURTLEBOT3_MODEL=waffle`, sources workspace if built.
- Workspace mounts: `../../src` -> `/ros2_ws/src`, plus `../../build|install|log` to `/ros2_ws/*`.

Container helper scripts (../../tools)
- `../../tools/exec.sh`: shell or command inside container `ros2`.
- `../../tools/root-exec.sh`: root shell inside container `ros2`.
- `../../tools/compile-ros.sh`: build workspace or selected packages inside container.
- `../../tools/create_pkg.sh`: create ROS 2 package (defaults: ament_python, rclpy).

Default workflows
- Build in container: `../../tools/compile-ros.sh navegacion_gps` (or no args for full).
- Exec in container: `../../tools/exec.sh <cmd>` (no args opens a shell).
- Source (inside container): `source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash`
- Main demo: `ros2 launch navegacion_gps gps_waypoint_follower.launch.py use_rviz:=True use_mapviz:=True`
- Sim only: `ros2 launch navegacion_gps gazebo_gps_world.launch.py`
- Localization: `ros2 launch navegacion_gps dual_ekf_navsat.launch.py`
- Mapviz: `ros2 launch navegacion_gps mapviz.launch.py`
- Tools wrapper: `./tools/start_nav2_gps_demo.sh [logged|interactive|logger] [waypoints.yaml]`

Key topics and frames
- Topics: `/gps/fix` (NavSatFix), `/imu` (Imu), `/odom`, `/scan`.
- TF: `map -> odom -> base_link` (or `base_footprint` if configured).

Config edit guidance
- GPS + EKF sources and frames live in `config/dual_ekf_navsat_params.yaml`.
- Nav2 tuning without maps uses `config/nav2_no_map_params.yaml`.
- Demo waypoints live in `config/demo_waypoints.yaml`.
- Update both YAML params and launch remaps together when changing topic or frame names.

Launch edit guidance
- When adding new params, wire them as `DeclareLaunchArgument` with defaults.
- Keep Gazebo and Nav2 launches loosely coupled (use remaps/args, not hard-coded names).

Nav2 + robot_localization expectations
- `navsat_transform_node` needs valid IMU orientation and GPS fixes.
- If TF breaks, check `dual_ekf_navsat_params.yaml` and remaps in `launch/dual_ekf_navsat.launch.py`.

Testing and validation
- If asked to validate: `colcon test --packages-select navegacion_gps`.
- At minimum, run a launch that exercises the edited files and confirm topics/TF.

Style and safety
- Prefer `rg` for search and keep edits localized.
- Avoid changing `worlds/` or `models/` unless requested.
- Avoid changing `../../Dockerfile` or `../../docker-compose.yml` unless requested.
- Keep files ASCII-only unless a file already uses non-ASCII.
