## navegacion_gps

Demo de navegacion GPS con ROS 2 + Nav2. Usa robot_localization (EKF + navsat_transform) para fusionar GPS/IMU/odometria y Nav2 para seguir waypoints geograficos (lat/lon/yaw). Incluye simulacion con Gazebo y herramientas para registrar y seguir waypoints.

### Requisitos
- ROS 2 Humble (o compatible).
- Paquetes ROS 2: `navigation2`, `nav2_bringup`, `nav2_simple_commander`, `nav2_msgs`, `robot_localization`, `mapviz`, `mapviz_plugins`, `tile_map`, `geographic_msgs`, `ros_gz_sim`, `ros_gz_bridge`.
- Simulacion: `gz sim` (Gazebo).
- Python: `python3-yaml`, `tkinter` (para el logger GUI). `pymavlink` se usa en `sensores`.

### Estructura del proyecto
- `navegacion_gps/`: nodos Python.
  - `interactive_waypoint_follower.py`: sigue waypoints clickeados en Mapviz (frame `wgs84`).
  - `yaml_waypoints_from_ll.py`: convierte YAML lat/lon con `/fromLL` y manda todos juntos al `follow_waypoints`.
  - `gps_waypoint_logger.py`: GUI para registrar waypoints GPS.
  - `frame_id_stripper.py`: reescribe `frame_id` de sensores/odom para quitar prefijos.
  - `utils/gps_utils.py`: conversiones GeoPose y quaternion.
- `launch/`: lanzadores principales.
  - `simulacion.launch.py`: stack completo (Gazebo + EKF + Nav2 + RViz/Mapviz).
  - `navegacion.launch.py`: stack Nav2 sin simulacion (EKF + Nav2 + RViz/Mapviz).
  - `dual_ekf_navsat.launch.py`: EKF + navsat_transform (robot_localization).
  - `mapviz.launch.py`: Mapviz + initialize_origin (usa `config/mapviz_gps.mvc`).
- `config/`: parametros Nav2 y robot_localization.
- `worlds/`: mundos de simulacion (`pasillos_obstaculos.world`, `default.sdf`, `vacio.world`).
- `tools/`: scripts de inicio.
- `docker-compose.yml`, `Dockerfile`: entorno en contenedor (ver notas abajo).

### Flujo general
1) `simulacion.launch.py` levanta Gazebo, el robot y el bridge.
2) `dual_ekf_navsat.launch.py` (incluido) fusiona `/odom`, `/imu/data`, `/gps/fix` y publica odometrias.
3) `navigation_launch.py` (Nav2) usa esos frames para planificar y seguir rutas.
4) Opcionalmente: RViz y/o Mapviz para visualizacion e interaccion.

### Topicos y frames esperados
Minimo para operar con GPS:
- `/gps/fix` (`sensor_msgs/NavSatFix`)
- `/imu/data` (`sensor_msgs/Imu`)
- `/odom` (odometria base)
- `/scan_3d` (PointCloud2 para costmaps)
- `/odometry/local` (odometria filtrada del EKF)
- `/cmd_vel` (entrada de control) y `/cmd_vel_steer` (bridge hacia Gazebo)

Frames tipicos:
- `map` -> `odom` -> `base_footprint` -> `base_link`
- `base_footprint` se usa en `dual_ekf_navsat_params.yaml` (ajustar si tu robot usa otro frame).

### Dimensiones del robot (urdf/cuatri.urdf)
- Distancia entre ejes (wheelbase): 0.94 m.
- Separacion entre ruedas traseras (track): 0.75 m.
- Radio de rueda: 0.24 m.
- Cuerpo (base_link) visual: 1.20 x 0.60 x 0.25 m.

### Formato de waypoints
Archivo YAML como `config/default-waypoints.yaml`:
```yaml
waypoints:
  - latitude: 38.161491054181276
    longitude: -122.45464431092836
    yaw: 0.0
  - latitude: 38.161587576524845
    longitude: -122.4547994038464
    yaw: 1.57
```

### Uso en host (sin contenedor)
```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch navegacion_gps simulacion.launch.py use_rviz:=True use_mapviz:=True
```
Sin simulacion:
```bash
ros2 launch navegacion_gps navegacion.launch.py use_rviz:=True use_mapviz:=True
```
Por defecto el `collision_monitor` se lanza (`use_collision_monitor:=True`); para desactivarlo:
```bash
ros2 launch navegacion_gps simulacion.launch.py use_collision_monitor:=False
```

Enviar waypoints:
```bash
ros2 run navegacion_gps yaml_waypoints_from_ll /ruta/a/waypoints.yaml
ros2 run navegacion_gps interactive_waypoint_follower
ros2 run navegacion_gps gps_waypoint_logger
```

### Mapviz
Lanza Mapviz con:
```bash
ros2 launch navegacion_gps simulacion.launch.py use_mapviz:=True
```
El config `config/mapviz_gps.mvc` incluye tiles y el click en `wgs84` (`/clicked_point`).

Teleop manual:
```bash
ros2 run navegacion_gps teleop
```
Nota: el modelo Ackermann no gira en el sitio; manten velocidad lineal (W/X)
y cambia direccion con A/D.

### Uso con el script de tools
El script detecta si el contenedor esta corriendo y entra con `docker exec`.
```bash
./tools/start_nav2_gps_demo.sh
./tools/start_nav2_gps_demo.sh logged /ruta/a/waypoints.yaml
./tools/start_nav2_gps_demo.sh interactive
./tools/start_nav2_gps_demo.sh logger config/default-waypoints.yaml
```

Si tu contenedor o workspace interno tienen otro nombre:
```bash
CONTAINER=mi_contenedor WS_IN_CONTAINER=/ros2_ws ./tools/start_nav2_gps_demo.sh
```

### Ajustes para GPS real (Pixhawk u otro)
Si tus topicos o frames no coinciden, ajusta:
- `config/dual_ekf_navsat_params.yaml` (sources: `odom0`, `imu0`, `odom1`).
- Remaps en `launch/dual_ekf_navsat.launch.py`.

Verifica que tu nodo publique:
- `NavSatFix` en `/gps/fix`.
- `Imu` en `/imu/data` con orientacion valida.
Si queres usar el lector de Pixhawk:
```bash
ros2 run sensores sensores --ros-args \
  -p serial_port:=/dev/ttyACM0 -p baudrate:=921600
```
O lanzar todo junto (driver + localization + nav2):
```bash
ros2 launch navegacion_gps gps_waypoint_follower_pixhawk.launch.py \
  serial_port:=/dev/ttyACM0 baudrate:=921600
```

### Notas sobre Docker
El `docker-compose.yml` y el `Dockerfile` actuales fueron creados para otro paquete
(`ros2_humble`). Para usar este repo dentro del contenedor:
- Monta el repo en `/ros2_ws/src/navegacion_gps`.
- Compila con `colcon build --packages-select navegacion_gps`.

Ejemplo de volumen en `docker-compose.yml`:
```
- ./:/ros2_ws/src/navegacion_gps:rw
```

### Troubleshooting
- `package not found`: asegurate de haber corrido `colcon build` y `source install/setup.bash`.
- No hay GPS: valida `ros2 topic echo /gps/fix` y `ros2 topic echo /imu/data`.
- TF roto: revisa frames en `dual_ekf_navsat_params.yaml` y el arbol TF con `ros2 run tf2_tools view_frames`.
- Mapviz sin tiles: requiere acceso a red para descargar mapas.
- Mapviz no publica `wgs84`: revisa `initialize_origin` y el frame `origin`.
