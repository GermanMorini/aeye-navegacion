# Checklist de Validacion Antes de Usar `real_local_v2`

## Preparacion
- Confirmar que no haya otra navegacion corriendo en el robot.
- Confirmar que Pixhawk, RS16 y controlador esten conectados.
- Confirmar acceso a `/dev/ttyACM0` y `/dev/serial0`.
- Confirmar que el robot tenga espacio libre para prueba.
- Para la primera validacion, preferir ruedas levantadas.
- Confirmar que el deploy en el robot corresponda a la branch `feature/sim-nav-v2-ackermann`.

## Arranque Basico
Lanzar:

```bash
./tools/launch_real_local_v2.sh
```

Verificar que levanten:

- `mavros`
- `rs16`
- `vehicle_controller_server`
- `ackermann_odometry`
- `ekf_filter_node_local_v2`
- `controller_server`
- `velocity_smoother`
- `collision_monitor`
- `goal_pose_to_follow_path_v2`

Verificar que RViz abra con la config local `v2`.

## Sensores y Topics
Confirmar mensajes validos en:

- `/controller/drive_telemetry`
- `/imu/data`
- `/scan_3d`
- `/scan`

En `/controller/drive_telemetry` verificar:

- velocidad valida
- steering valido
- timestamps frescos
- telemetria consistente con el estado del robot

## TF y Localizacion
Verificar TF:

- `odom -> base_footprint`
- `base_footprint -> base_link`
- `base_link -> imu_link`
- `base_link -> lidar_link`

Confirmar:

- `odom` queda fijo
- el robot se mueve respecto a `odom`

Verificar publicacion en:

- `/wheel/odometry`
- `/vehicle/twist`
- `/odometry/local`

## Signos y Sentido de Giro
Con ruedas levantadas:

- mandar un comando corto o un `2D Goal Pose` pequeno
- verificar que `angular.z > 0` produzca giro en el sentido esperado

Comparar:

- `/cmd_vel_nav`
- `/cmd_vel_safe`
- `/wheel/odometry`

Si el sentido no coincide, revisar:

- `invert_steer_from_cmd_vel`
- convencion de steering medida
- signo de yaw en la odometria

## Cadena de Control
Verificar flujo:

- `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_safe`

Confirmar:

- `collision_monitor` no frena por error al arrancar
- al terminar o cancelar un goal el robot queda en stop
- no hay residuos de comando luego del goal

## Prueba Local Minima
Mandar un `2D Goal Pose` corto y simple.

Verificar en RViz:

- `Goal Path`
- `Path Tracking Debug`
- traza del robot

Mirar especialmente:

- `d`
- `yaw_err`
- `nav_wz`
- `safe_wz`
- `odom_wz`

## Prueba en Piso
Orden recomendado:

1. recta corta
2. curva suave
3. goal cruzado mas largo

Durante la prueba vigilar:

- oscilacion
- sobrepaso
- replan innecesario
- stops espurios
- perdida de seguimiento del path

## Parametros a Tener Listos Para Ajustar
- `wheelbase_m`
- `vx_min_effective_mps`
- covarianzas de `ackermann_odometry`
- footprint
- `lookahead_dist`
- `xy_goal_tolerance`

## Criterio Minimo de Aceptacion
- steering con signo correcto
- `/wheel/odometry` coherente con el movimiento
- `/odometry/local` estable sin GPS
- `2D Goal Pose` corto ejecutado sin abortos raros
- stop correcto al terminar o cancelar
