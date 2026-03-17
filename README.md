# navegacion_gps

Paquete de navegación del workspace. Integra Nav2, `robot_localization`, herramientas para zonas de exclusión y un backend ROS para control manual y snapshots de navegación.

## Ejecutables reales
- `gazebo_utils`
- `zones_manager`
- `nav_command_server`
- `nav_snapshot_server`

Este checkout no incluye los antiguos nodos de waypoints interactivos, logger GUI ni teleop de teclado.

## Launches reales
- `ros2 launch navegacion_gps simulacion.launch.py`
  - Gazebo Sim + bridge ROS/GZ + robot_localization + Nav2 + zonas + backend web opcional
- `ros2 launch navegacion_gps real.launch.py`
  - robot_localization + Nav2 + sensores reales opcionales + zonas + backend web opcional
- `ros2 launch navegacion_gps rviz_real.launch.py`
  - RViz + `robot_state_publisher` usando el URDF real

## Flujo de control
- Nav2 publica `/cmd_vel`.
- `nav2_collision_monitor` publica `/cmd_vel_safe`.
- `nav_command_server` recibe `/cmd_vel_safe` y comandos manuales en `/cmd_vel_teleop`.
- `nav_command_server` publica `/cmd_vel_final` (`interfaces/msg/CmdVelFinal`).
- En simulación, `gazebo_utils` puede adaptar `/cmd_vel_final` a `/cmd_vel_gazebo`.
- En hardware real, `controller_server` consume `/cmd_vel_final`.

## Nodos del paquete
### `zones_manager`
- Gestiona zonas no transitables a partir de GeoJSON.
- Genera y recarga la máscara `keepout`.
- Servicios:
  - `/zones_manager/set_geojson`
  - `/zones_manager/get_state`
  - `/zones_manager/reload_from_disk`

### `nav_command_server`
- Backend ROS para órdenes geográficas y control manual.
- Publica telemetría en `/nav_command_server/telemetry`.
- Servicios:
  - `/nav_command_server/set_goal_ll`
  - `/nav_command_server/cancel_goal`
  - `/nav_command_server/brake`
  - `/nav_command_server/set_manual_mode`
  - `/nav_command_server/get_state`
- Acciones cliente:
  - `follow_waypoints`
  - `navigate_through_poses`

### `nav_snapshot_server`
- Compone snapshots PNG del estado local de navegación.
- Servicio:
  - `/nav_snapshot_server/get_nav_snapshot`

### `gazebo_utils`
- Normaliza `frame_id` y tópicos de sensores bridged desde Gazebo Sim.
- Puede puentear `/cmd_vel_final` hacia `/cmd_vel_gazebo` en simulación.

## Tópicos y frames principales
- Entradas de localización:
  - `/imu/data`
  - `/gps/fix`
  - `/odom`
- Salidas de localización:
  - `/odometry/local`
  - `/odometry/gps`
- Percepción:
  - `/scan_3d`
  - `/scan`
- Control:
  - `/cmd_vel_safe` (`geometry_msgs/msg/Twist`)
  - `/cmd_vel_teleop` (`interfaces/msg/CmdVelFinal`)
  - `/cmd_vel_final` (`interfaces/msg/CmdVelFinal`)

Frames esperados:
- `map -> odom -> base_footprint`

## Dependencias funcionales
- Nav2:
  - `nav2_bringup`
  - `nav2_collision_monitor`
  - `nav2_map_server`
  - `nav2_lifecycle_manager`
- Localización:
  - `robot_localization`
- Simulación:
  - `ros_gz_sim`
  - `ros_gz_bridge`
- Costmaps:
  - `pointcloud_to_laserscan`

## Uso dentro del contenedor
Build:
```bash
./tools/compile-ros.sh navegacion_gps
```

Real:
```bash
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch navegacion_gps real.launch.py"
```

Simulación:
```bash
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch navegacion_gps simulacion.launch.py"
```

RViz para real:
```bash
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch navegacion_gps rviz_real.launch.py"
```

## Notas
- `mapviz_gps.mvc` existe en la raíz del workspace y se copia en la imagen Docker, pero este paquete ya no expone un `mapviz.launch.py` dedicado.
- Si actualizas nombres de tópicos o frames, cambia también launches, YAML de Nav2 y YAML de `robot_localization`.
