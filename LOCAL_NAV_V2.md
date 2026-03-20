# Navegacion Local V2 Ackermann

## Resumen
La `v2` es una base nueva de navegacion local para el robot Ackermann. Se ejecuta en paralelo a la `v1` y arranca sin planner global ni GPS en la capa local. La idea es tener una base local simple, observable y alineada con el robot real:

- odometria Ackermann derivada de telemetria de ruedas y direccion;
- EKF local en `odom`;
- Nav2 local-only;
- simulacion y robot real consumiendo el mismo contrato de señales.

La `v2` esta pensada para tres tareas:

- levantar una simulacion local reproducible;
- validar localizacion y seguimiento de paths cortos;
- tunear la base local antes de reintroducir navegacion global.

## Objetivo y alcance
La `v2` resuelve la navegacion local del robot sin depender del EKF global del Pixhawk ni de GPS para `odom -> base_footprint`.

Queda explicitamente fuera de esta fase:

- planner global;
- `map -> odom`;
- `navsat_transform`;
- goals geograficos LL;
- fusion GPS en la capa local.

Diferencias principales contra la `v1`:

- la localizacion local ya no toma `/local_position/odom` ni `/odom` legacy como base;
- la odometria local sale de ruedas/direccion medidas;
- la navegacion usa solo `FollowPath` sobre `odom`;
- la cadena de comandos local es mas corta y mas facil de depurar;
- la simulacion `v2` separa mejor sensores, localizacion y Nav2.

## Arquitectura general
### Flujo en simulacion
Launch principal:

```bash
ros2 launch navegacion_gps sim_local_v2.launch.py
```

Composicion:

1. `sim_v2_base.launch.py`
2. `localization_v2.launch.py`
3. `nav_local_v2.launch.py`
4. `rviz_local_v2.rviz`

### Flujo en robot real
Launch principal:

```bash
ros2 launch navegacion_gps real_local_v2.launch.py
```

Composicion:

1. `mavros.launch.py`
2. `rs16.launch.py`
3. `controller_server_node`
4. `localization_v2.launch.py`
5. `nav_local_v2.launch.py`
6. `rviz_local_v2.rviz`

### Nodos principales de la v2
| Nodo | Rol |
| --- | --- |
| `sim_drive_telemetry` | Construye `DriveTelemetry` en simulacion a partir de `/odom_raw` y `/joint_states`. |
| `sim_sensor_normalizer_v2` | Normaliza frames y covarianzas de IMU, GPS, nube y odometria bridged desde Gazebo. |
| `cmd_vel_ackermann_bridge_v2` | Convierte `/cmd_vel_safe` en `/cmd_vel_gazebo` reutilizando la logica de control real. |
| `ackermann_odometry` | Integra odometria planar Ackermann y publica `/wheel/odometry` y `/vehicle/twist`. |
| `ekf_filter_node_local_v2` | Fusiona wheel odom + IMU y publica `/odometry/local` y TF `odom -> base_footprint`. |
| `controller_server` | Ejecuta `FollowPath` con `RegulatedPurePursuitController`. |
| `velocity_smoother` | Suaviza `/cmd_vel_nav` antes de `collision_monitor`. |
| `collision_monitor` | Aplica stop preventivo y publica `/cmd_vel_safe`. |
| `goal_pose_to_follow_path_v2` | Traduce `2D Goal Pose` a `FollowPath`, publica path de debug y replanifica si pierde captura. |

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
- La capa local **no usa** `/local_position/odom`.
- La capa local **no usa** `/odom` legacy como entrada principal.
- GPS queda fuera de `odom -> base_footprint`.
- Si mas adelante vuelve una capa global, GPS deberia entrar en `map -> odom`, no en la base local.

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
- Gazebo publica `/odom_raw`, pero en `v2` no publica TF de odometria al resto del sistema.
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

La `v2` local usa esta odometria como base del movimiento longitudinal y de yaw estimado por el modelo Ackermann.

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
- `linear.y` fijado por la salida odom
- `angular.z`

De `/imu/data` se usa:

- `angular.z`

La IMU en esta configuracion no se usa para pose absoluta; aporta principalmente yaw rate. La pose local sigue siendo independiente de GPS.

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

## Cadena de control y smoothing
Cadena nominal de mando:

```text
/goal_pose
-> goal_pose_to_follow_path_v2
-> /follow_path
-> controller_server
-> /cmd_vel_nav
-> velocity_smoother
-> /cmd_vel_smoothed
-> collision_monitor
-> /cmd_vel_safe
-> cmd_vel_ackermann_bridge_v2
-> /cmd_vel_gazebo
-> plugin Ackermann de Gazebo
```

### `cmd_vel_ackermann_bridge_v2`
Este nodo:

- consume `/cmd_vel_safe`;
- reutiliza `controller_server.control_logic.command_from_cmd_vel`;
- aplica deadband, velocidad minima efectiva, clamp de velocidad y steering;
- convierte el comando al contrato esperado por el plugin Ackermann de Gazebo.

En la simulacion, `/cmd_vel_gazebo` se bridgea a `/cmd_vel_steer`.

### `velocity_smoother`
Configuracion relevante:

- `feedback: OPEN_LOOP`
- `max_velocity: [0.5, 0.0, 0.4]`
- `min_velocity: [-0.5, 0.0, -0.4]`

La `v2` quedo en `OPEN_LOOP` por una razon concreta: `CLOSED_LOOP` introducia comportamiento inconsistente en esta cadena Ackermann local, especialmente combinando:

- feedback basado en `/odometry/local`;
- velocidad minima efectiva del robot;
- conversion `cmd_vel -> steering`.

Con `OPEN_LOOP`, la cadena `cmd_vel_nav -> cmd_vel_safe` queda coherente y mas predecible para esta arquitectura.

### `collision_monitor`
Usa:

- entrada `/cmd_vel_smoothed`
- salida `/cmd_vel_safe`
- frame base `base_footprint`
- frame de odometria `odom`
- fuente de observacion `/scan`

Poligonos actuales:

- `footprint`
- `stop_zone`

## Navegacion local y goals
La `v2` no usa planner global. Usa `FollowPath` sobre `odom`.

### `goal_pose_to_follow_path_v2`
Este nodo:

- escucha `/goal_pose` desde RViz;
- toma la pose actual desde `/odometry/local`;
- genera un path Ackermann-like;
- publica ese path en `/goal_pose_path`;
- envia el path como `nav2_msgs/action/FollowPath`;
- monitorea si el robot pierde captura del path;
- cancela y replanifica si la distancia al path supera el umbral configurado.

### Overlay de debug
Publica:

- `/local_nav_v2/path_tracking_debug`

Muestra en RViz:

- punto mas cercano del robot al path;
- segmento robot-path;
- texto con:
  - `d`: distancia lateral al path;
  - `yaw_err`: error angular entre robot y path;
  - `nav_wz`: yaw rate pedido por Nav2;
  - `safe_wz`: yaw rate despues de smoother + collision monitor;
  - `odom_wz`: yaw rate ejecutado por la planta.

### Interpretacion practica
- `nav_wz` alto y `safe_wz` parecido: el controller esta pidiendo girar fuerte y la cadena lo deja pasar.
- `nav_wz` alto y `safe_wz` mucho mas chico: el limiter/smoother esta recortando el comando.
- `safe_wz` y `odom_wz` con mismo signo pero magnitud distinta: la planta no logra toda la curvatura pedida.
- `d` creciendo y `yaw_err` creciendo: el robot esta perdiendo captura del path.

## Parametros y configuraciones usadas
Archivos principales:

- `config/localization_v2.yaml`
- `config/nav2_local_v2_params.yaml`
- `config/collision_monitor_v2.yaml`
- `config/rviz_local_v2.rviz`

### Parametros clave
| Parametro | Default actual | Donde se usa |
| --- | --- | --- |
| `wheelbase_m` | `0.94` | `ackermann_odometry`, modelo Ackermann |
| `vx_deadband_mps` | `0.01` | bridge/control real |
| `vx_min_effective_mps` | `0.5` | bridge/control real |
| `xy_goal_tolerance` | `0.5` | `PositionGoalChecker` |
| `desired_linear_vel` | `0.5` | `RegulatedPurePursuitController` |
| `lookahead_dist` | `0.8` | `RegulatedPurePursuitController` |
| `feedback` | `OPEN_LOOP` | `velocity_smoother` |

### Defaults de sim
- `use_sim_time = True`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `goal_pose_use_goal_orientation = False`
- mundo por defecto: `vacio.world`

### Defaults de real
- `use_sim_time = False`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `invert_steer_from_cmd_vel = True` en `controller_server_node`
- `goal_pose_use_goal_orientation = False`

### Parametros a recalibrar con el robot
- `wheelbase_m`
- footprint y `stop_zone`
- covarianzas de wheel odom
- `vx_min_effective_mps`
- `lookahead_dist`
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

### Que mirar primero
- `/odometry/local`
- `/wheel/odometry`
- `/goal_pose_path`
- `/cmd_vel_nav`
- `/cmd_vel_safe`
- `/local_nav_v2/path_tracking_debug`

### Fallas tipicas
**No sigue el path**

- revisar `d`, `yaw_err`, `nav_wz`, `safe_wz`, `odom_wz`;
- revisar si `safe_wz` esta siendo recortado;
- revisar si `odom_wz` responde al signo correcto.

**Oscila**

- bajar agresividad del seguimiento;
- revisar `lookahead_dist`;
- revisar `desired_linear_vel`;
- revisar curvatura del path generado.

**Replanifica demasiado**

- revisar `path_replan_distance_m`;
- revisar si el robot realmente pierde captura;
- revisar si la planta no alcanza la curvatura pedida.

**No llega a moverse**

- revisar `vx_min_effective_mps`;
- revisar si `safe_wz` y `safe_x` salen de Nav2;
- revisar `collision_monitor`;
- revisar que no haya `stop_hold` activo.

**Signs inconsistentes**

- comparar `nav_wz`, `safe_wz` y `odom_wz`;
- revisar `invert_steer_from_cmd_vel`;
- revisar la conversion de steering en simulacion y en real por separado.

### Checklist rapido de validacion
Sim:

1. levantar `sim_local_v2`;
2. confirmar TF `odom -> base_footprint`;
3. confirmar `/wheel/odometry` y `/odometry/local`;
4. mandar un `2D Goal Pose`;
5. mirar `Goal Path` y `Path Tracking Debug`;
6. revisar que `nav_wz`, `safe_wz` y `odom_wz` sean coherentes.

Real:

1. levantar `real_local_v2`;
2. verificar `/controller/drive_telemetry`;
3. verificar `/wheel/odometry` y `/odometry/local`;
4. probar con ruedas levantadas;
5. validar avance, giro y reversa;
6. ajustar covarianzas y wheelbase si hace falta.

## Contratos publicos de la v2
La documentacion de esta fase deja explicitados estos contratos ROS:

- `/controller/drive_telemetry`
- `/wheel/odometry`
- `/vehicle/twist`
- `/goal_pose_path`
- `/local_nav_v2/path_tracking_debug`

No se agrega una API ROS nueva por esta tarea de documentacion.

## Limitaciones actuales
- Todavia hay oscilaciones.
- `2D Goal Pose` sigue siendo una aproximacion local, no un planner global.
- La capa global aun no fue reintroducida.
- La planta simulada y el robot real todavia requieren tuning fino de footprint, covarianzas y limites.
- El comportamiento observado hoy es razonable para validacion local, pero no es todavia la arquitectura final de navegacion completa.
