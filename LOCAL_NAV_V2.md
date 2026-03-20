# Navegacion Local V2 Ackermann

## Resumen
La `v2` es una base de navegacion local para el robot Ackermann que corre completamente en `odom`.
La capa local actual ya no usa `goal_pose_to_follow_path_v2` ni `velocity_smoother`: hoy la navegacion se apoya en un stack Nav2 nativo sobre `odom`, con localizacion Ackermann + EKF local + costmaps rolling + collision monitor.

Objetivos de esta fase:

- levantar una simulacion local reproducible;
- validar localizacion y seguimiento de paths cortos;
- tunear la base local antes de reintroducir una capa global.

Queda explicitamente fuera de esta fase:

- `map -> odom`;
- `navsat_transform`;
- goals geograficos LL;
- fusion GPS en la capa local;
- navegacion global basada en `map`.

## Arquitectura general
### Flujo en simulacion
Launch principal:

```bash
ros2 launch navegacion_gps sim_local_v2.launch.py
```

Composicion efectiva:

1. `sim_v2_base.launch.py`
2. `localization_v2.launch.py`
3. `keepout_filter_mask_server`
4. `keepout_costmap_filter_info_server`
5. `lifecycle_manager_keepout_filters`
6. `nav_local_v2.launch.py`
7. `rviz_local_v2.rviz`

### Flujo en robot real
Launch principal:

```bash
ros2 launch navegacion_gps real_local_v2.launch.py
```

Composicion efectiva:

1. `mavros.launch.py`
2. `rs16.launch.py`
3. `vehicle_controller_server`
4. `pointcloud_to_laserscan`
5. `localization_v2.launch.py`
6. `nav_local_v2.launch.py`
7. `rviz_local_v2.rviz`

## Nodos principales de la v2
| Nodo | Rol |
| --- | --- |
| `sim_drive_telemetry` | Construye `DriveTelemetry` en simulacion a partir de `/odom_raw` y `/joint_states`. |
| `sim_sensor_normalizer_v2` | Normaliza frames y covarianzas de IMU, GPS, nube y odometria bridged desde Gazebo. |
| `cmd_vel_ackermann_bridge_v2` | Convierte `/cmd_vel_safe` en `/cmd_vel_gazebo` reutilizando la logica de control real. |
| `ackermann_odometry` | Integra odometria planar Ackermann y publica `/wheel/odometry` y `/vehicle/twist`. |
| `ekf_filter_node_local_v2` | Fusiona wheel odom + IMU y publica `/odometry/local` y TF `odom -> base_footprint`. |
| `planner_server` | Calcula el plan global-local en `odom` usando el costmap rolling. |
| `smoother_server` | Suaviza el plan generado antes de entregarlo al controlador. |
| `controller_server` | Sigue el path con `RegulatedPurePursuitController` y publica `/cmd_vel`. |
| `bt_navigator` | Orquesta `ComputePath`, `SmoothPath`, `FollowPath` y recoveries del BT. |
| `behavior_server` | Ejecuta behaviors de recuperacion (`BackUp`, `DriveOnHeading`, `Wait`). |
| `waypoint_follower` | Expone la navegacion por waypoints para el perfil local-v2. |
| `collision_monitor` | Aplica stop preventivo y publica `/cmd_vel_safe`. |
| `keepout_filter_mask_server` | Publica la mascara keepout estatica usada por los costmaps de `sim_local_v2`. |
| `keepout_costmap_filter_info_server` | Publica metadata del filtro keepout en `/costmap_filter_info`. |
| `polygon_stamped_republisher` | Republlica `/stop_zone_raw` hacia `/stop_zone` para RViz y consumers legacy. |

## Sensores y señales usadas por la navegacion local
### En robot real
- `/controller/drive_telemetry`
- `/imu/data`
- `/scan_3d` -> `/scan`

### En simulacion
- `/odom_raw`
- `/joint_states`
- `/imu/data_raw` -> `/imu/data`
- `/scan_3d_raw` -> `/scan_3d` -> `/scan`
- `/gps/fix_raw` -> `/gps/fix` solo para observabilidad

### Decisiones explicitas de la v2
- La capa local no usa `/local_position/odom`.
- La capa local no usa `/odom` legacy como entrada principal.
- GPS queda fuera de `odom -> base_footprint`.
- Todo el stack Nav2 local opera en `odom`.

## TF y frames
Arbol esperado:

```text
odom
└── base_footprint
    └── base_link
        ├── imu_link
        ├── gps_link
        ├── lidar_link
        ├── front_left_steer_link
        ├── front_right_steer_link
        ├── front_left_wheel_link
        ├── front_right_wheel_link
        ├── rear_left_wheel_link
        └── rear_right_wheel_link
```

### Que publica cada parte
- `robot_localization` publica `odom -> base_footprint`.
- `robot_state_publisher` publica la cadena fija del modelo a partir del URDF.
- Gazebo publica `/odom_raw`, pero la referencia operativa de navegacion es `/odometry/local`.
- `sim_sensor_normalizer_v2` corrige `frame_id` y covarianzas; no inventa un TF nuevo.

### Por que `odom` queda fijo
En la `v2`, `world_frame = odom` en `robot_localization`. Eso significa:

- `odom` es el frame local fijo donde arranca el robot;
- el que se mueve es `base_footprint`;
- no hay `map -> odom` en esta fase;
- por eso en RViz es normal que `odom` quede quieto y el robot se mueva respecto de el.

### Tabla de frames
| Frame | Productor | Uso |
| --- | --- | --- |
| `odom` | `ekf_filter_node_local_v2` | referencia local fija de navegacion |
| `base_footprint` | `ekf_filter_node_local_v2` + URDF | base plana del robot para costmaps y control |
| `base_link` | `robot_state_publisher` | base mecanica del modelo |
| `imu_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | IMU para EKF |
| `gps_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | GPS solo observabilidad en `v2` |
| `lidar_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | percepcion local |

## Localizacion local
### `ackermann_odometry`
Entrada:

- `interfaces/msg/DriveTelemetry`

Salidas:

- `/wheel/odometry` (`nav_msgs/Odometry`)
- `/vehicle/twist` (`geometry_msgs/TwistWithCovarianceStamped`)

Modelo usado:

1. toma `speed_mps_measured`;
2. aplica signo con `reverse_requested`;
3. convierte `steer_deg_measured` a radianes;
4. calcula:

```text
yaw_rate = v * tan(delta) / wheelbase
```

5. integra pose planar con un paso tipo midpoint:

```text
x, y, yaw <- integrate_planar(x, y, yaw, v, yaw_rate, dt)
```

La `v2` local usa esta odometria como base del movimiento longitudinal y del yaw estimado por el modelo Ackermann.

### EKF local (`localization_v2.yaml`)
Configuracion base:

- `two_d_mode: true`
- `world_frame: odom`
- `odom_frame: odom`
- `base_link_frame: base_footprint`

Entradas fusionadas:

- `/wheel/odometry`
- `/imu/data`

De `/wheel/odometry` se usan:

- pose planar `x`, `y`, `yaw`
- `linear.x`
- `linear.y`
- `angular.z`

De `/imu/data` se usa:

- `angular.z`

La IMU no se usa para pose absoluta; aporta principalmente yaw rate. La pose local sigue siendo independiente de GPS.

### Covarianzas y tuning expuesto por launch
`localization_v2.launch.py` expone:

- `pose_covariance_xy`
- `pose_covariance_yaw`
- `twist_covariance_vx`
- `twist_covariance_vy`
- `twist_covariance_yaw_rate`
- `wheelbase_m`

Defaults actuales:

- `pose_covariance_xy = 0.05`
- `pose_covariance_yaw = 0.1`
- `twist_covariance_vx = 0.05`
- `twist_covariance_vy = 0.01`
- `twist_covariance_yaw_rate = 0.1`
- `wheelbase_m = 0.94`

## Navegacion local y pipeline de comandos
La navegacion local actual usa Nav2 nativo sobre `odom`. El goal operativo ya no pasa por `goal_pose_to_follow_path_v2`.

Cadena nominal de control en simulacion:

```text
RViz Nav2 goal / bt_navigator
-> planner_server
-> smoother_server
-> controller_server
-> /cmd_vel
-> collision_monitor
-> /cmd_vel_safe
-> cmd_vel_ackermann_bridge_v2
-> /cmd_vel_gazebo
-> /cmd_vel_steer (Gazebo)
```

### Topics relevantes
- `/plan`: plan visible de Nav2
- `/cmd_vel`: salida del `controller_server`
- `/cmd_vel_safe`: salida del `collision_monitor`
- `/cmd_vel_gazebo`: salida del bridge Ackermann
- `/odometry/local`: odometria filtrada de referencia para control y costmaps

### `cmd_vel_ackermann_bridge_v2`
Este nodo:

- consume `/cmd_vel_safe`;
- reutiliza `controller_server.control_logic.command_from_cmd_vel`;
- aplica deadband, velocidad minima efectiva y clamps de velocidad/steering;
- convierte el comando al contrato esperado por el plugin Ackermann de Gazebo.

En la simulacion, `bridge_config_v2.yaml` bridgea `/cmd_vel_gazebo` hacia Gazebo como `/cmd_vel_steer`.

### `collision_monitor`
Usa:

- entrada `/cmd_vel`
- salida `/cmd_vel_safe`
- frame base `base_footprint`
- frame de odometria `odom`
- fuente de observacion `/scan`

Poligonos actuales:

- `footprint`
- `stop_zone`

`collision_monitor` publica la zona de stop en `/stop_zone_raw`. Luego `polygon_stamped_republisher` la republia en `/stop_zone` para visualizacion y consumers que esperan `PolygonStamped` estable.

## Keepout y stop zone
### Keepout estatico en `sim_local_v2`
`sim_local_v2` levanta un keepout estatico en `odom` con estos nodos:

- `keepout_filter_mask_server`
- `keepout_costmap_filter_info_server`
- `lifecycle_manager_keepout_filters`

La mascara proviene de:

- `config/keepout_mask.yaml`

Y se publica en:

- `/keepout_filter_mask`
- `/costmap_filter_info`

Los costmaps local y global de `nav2_local_v2_params.yaml` usan `keepout_filter`, por eso este perfil necesita los publishers del filtro aunque no tenga `map`.

### Stop zone
Ademas del keepout de costmap, `collision_monitor` mantiene una `stop_zone` reactiva para frenado inmediato basada en `/scan`.

Topics asociados:

- `/stop_zone_raw`
- `/stop_zone`

## Parametros y configuraciones usadas
Archivos principales:

- `config/localization_v2.yaml`
- `config/nav2_local_v2_params.yaml`
- `config/collision_monitor_v2.yaml`
- `config/rviz_local_v2.rviz`
- `config/keepout_mask.yaml`

### Parametros clave
| Parametro | Default actual | Donde se usa |
| --- | --- | --- |
| `wheelbase_m` | `0.94` | `ackermann_odometry`, modelo Ackermann |
| `vx_deadband_mps` | `0.01` | `cmd_vel_ackermann_bridge_v2` |
| `vx_min_effective_mps` | `0.5` | `cmd_vel_ackermann_bridge_v2` |
| `xy_goal_tolerance` | `1.2` | `PositionGoalChecker` |
| `desired_linear_vel` | `1.2` | `RegulatedPurePursuitController` |
| `lookahead_dist` | `1.6` | `RegulatedPurePursuitController` |
| `minimum_turning_radius` | `2.2` | `SmacPlannerHybrid` |

### Defaults de sim
- `use_sim_time = True`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `nav_start_delay_s = 4.0`
- mundo por defecto: `vacio.world`

### Defaults de real
- `use_sim_time = False`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `invert_steer_from_cmd_vel = True` en `vehicle_controller_server`

### Parametros a recalibrar con el robot
- `wheelbase_m`
- footprint y `stop_zone`
- covarianzas de wheel odom
- `vx_min_effective_mps`
- `lookahead_dist`
- `desired_linear_vel`
- `minimum_turning_radius`
- tolerancia de goal

## Operacion y debugging
### Como lanzar
Dentro del contenedor:

```bash
ros2 launch navegacion_gps sim_local_v2.launch.py
ros2 launch navegacion_gps real_local_v2.launch.py
```

Desde el host, usando scripts del workspace:

```bash
./tools/launch_sim_local_v2.sh
./tools/launch_real_local_v2.sh
```

Checklist de validacion en robot real:

- [REAL_LOCAL_V2_CHECKLIST.md](/home/gmorini/Documentos/codigo/ros2/workspace/src/navegacion_gps/REAL_LOCAL_V2_CHECKLIST.md)

### Que mirar primero
- `/odometry/local`
- `/wheel/odometry`
- `/plan`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/cmd_vel_gazebo`
- `/keepout_filter_mask`
- `/costmap_filter_info`
- `/stop_zone_raw`
- `/stop_zone`

### RViz
El plan visible relevante en `rviz_local_v2.rviz` es `/plan`.
El flujo historico basado en `/goal_pose_path` ya no describe la navegacion local actual.
Para la operacion del stack actual, el objetivo debe interpretarse como un goal de Nav2 y no como el path generado por el helper antiguo.

### Fallas tipicas
**No se mueve**

- revisar `/cmd_vel`, `/cmd_vel_safe` y `/cmd_vel_gazebo`;
- revisar `collision_monitor`;
- revisar que `cmd_vel_ackermann_bridge_v2` este consumiendo `/cmd_vel_safe`;
- revisar `vx_min_effective_mps`.

**No recibe keepout**

- revisar `/keepout_filter_mask` y `/costmap_filter_info`;
- revisar `keepout_filter_mask_server` y `keepout_costmap_filter_info_server`;
- revisar que el `frame_id` del filtro sea `odom` en simulacion.

**Oscila o abre demasiado la curva**

- revisar `lookahead_dist`;
- revisar `desired_linear_vel`;
- revisar `minimum_turning_radius`;
- revisar el plan en `/plan` y no solo la trayectoria ejecutada.

**No llega al goal como se espera**

- revisar `xy_goal_tolerance`;
- revisar `use_final_approach_orientation` del planner;
- revisar el BT y el `smoother_server`.

### Checklist rapido de validacion
Sim:

1. levantar `sim_local_v2`;
2. confirmar TF `odom -> base_footprint`;
3. confirmar `/wheel/odometry` y `/odometry/local`;
4. confirmar `/keepout_filter_mask` y `/costmap_filter_info`;
5. enviar un goal y mirar `/plan`;
6. verificar `/cmd_vel`, `/cmd_vel_safe` y `/cmd_vel_gazebo`.

Real:

1. levantar `real_local_v2`;
2. verificar `/controller/drive_telemetry`;
3. verificar `/wheel/odometry` y `/odometry/local`;
4. probar con ruedas levantadas;
5. validar avance, giro y reversa;
6. ajustar covarianzas y wheelbase si hace falta.

## Contratos publicos de la v2
Contratos ROS relevantes de esta fase:

- `/controller/drive_telemetry`
- `/wheel/odometry`
- `/vehicle/twist`
- `/odometry/local`
- `/plan`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/cmd_vel_gazebo`
- `/keepout_filter_mask`
- `/costmap_filter_info`
- `/stop_zone_raw`
- `/stop_zone`

## Limitaciones actuales
- La capa sigue siendo local-only; no hay `map -> odom`.
- La calidad del seguimiento sigue dependiendo del tuning de planner, smoother, controller y planta.
- La planta simulada y el robot real todavia requieren ajuste fino de footprint, covarianzas y limites.
- El perfil `real_local_v2` y el perfil de simulacion no son identicos en todos los auxiliares, pero comparten el mismo contrato local de odometria y navegacion en `odom`.
