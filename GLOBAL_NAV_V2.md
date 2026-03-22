# Navegacion Global V2 Ackermann

## Estado actual
Este documento describe la implementacion efectiva actual de `sim_global_v2` en este checkout. Complementa a [IMPLEMENTATION_PLAN_GLOBAL_NAV_V2.md](IMPLEMENTATION_PLAN_GLOBAL_NAV_V2.md), que sigue siendo el documento de arquitectura objetivo.

La implementacion actual cubre la fase 1 de simulacion:

- launch separado `sim_global_v2.launch.py`
- base comun compartida con `sim_local_v2`
- `navsat_transform`
- EKF global
- TF `map -> odom`
- goals LL y goals RViz sobre `map`
- RViz global dedicado
- perfiles de GPS simulados por preset

No cubre todavia:

- `real_global_v2`
- keepout global georreferenciado
- tuning de drift largo para validacion outdoor
- cambios a `simulacion.launch.py` o a la `v1`

## Problema critico corregido: runaway de `map -> odom`
Durante la implementacion de `sim_global_v2` aparecio un problema severo: el robot quedaba quieto en `odom`, el GPS sintetico quedaba estable, pero la transform `map -> odom` empezaba a caminar sola y terminaba desplazando todo el frame global decenas de metros en segundos.

### Sintomas observados
- en RViz el robot parecia "irse solo" aunque el modelo estuviera quieto en Gazebo;
- las flechas de `/odometry/gps` se veian estables, pero el frame `map` seguia rotando y trasladandose;
- el drift checker reportaba runaway puro de la fusion global:
  - `baseline`: `map->odom ≈ 104.899 m / 10 s`
  - `baseline`: `odom->base ≈ 0`
  - `baseline`: `/odometry/gps ≈ 0`
- la atribucion del chequeo cerraba siempre en `likely_origin=fusion_with_odometry_local`.

### Diagnostico
La cadena defectuosa no estaba en el GPS sintetico base ni en la odometria local. El problema era una inconsistencia de frames en la fusion global:

1. `sim_sensor_normalizer_v2` ya estaba generando un GPS sintetico a partir de `/odom_raw + datum`.
2. Esa señal sintetica representaba la pose del robot como plataforma, no la posicion fisica desplazada del sensor GNSS en el chasis.
3. Sin embargo, el `frame_id` publicado para `/gps/fix` seguia siendo `gps_link`.
4. En el URDF, `gps_link` no coincide con `base_footprint`: tiene un offset fijo de aproximadamente `x=0.76 m`, `y=0.04 m`, `z=0.62 m`.
5. `navsat_transform` y el EKF global intentaban reconciliar una medicion absoluta que decia "estoy en la pose del robot" pero etiquetada como "estoy en la pose del sensor desplazado".
6. En el perfil defectuoso tambien se reutilizaba yaw absoluto de `/odometry/local` dentro del EKF global y `navsat_transform.use_odometry_yaw=true`, asi que el offset del sensor se rotaba en el frame global.
7. El resultado era una correccion espuria continua en `map -> odom`, aunque:
   - `/odometry/local` estuviera quieto;
   - `/odometry/gps` no tuviera drift real;
   - el robot no se moviera fisicamente.

### Evidencia concreta
Con el perfil defectuoso, `/odometry/gps` quedaba clavado cerca de:

- `x = -0.76`
- `y = -0.04`

Ese delta coincide con el offset del `gps_link` del URDF. No era ruido aleatorio: era una inconsistencia estructural entre la semantica de la medicion sintetica y el frame con el que se publicaba.

### Solucion aplicada
La correccion final tiene tres partes y las tres quedaron en la implementacion actual:

1. `sim_global_v2` ya no expone perfiles de localizacion global alternativos.
   Solo queda el camino corregido.
2. El GPS sintetico de la cadena global se publica con `frame_id=base_footprint`.
   Esto hace que la semantica del mensaje coincida con la verdad base usada para construirlo.
3. El EKF global y `navsat_transform` conservan el ajuste correcto:
   - `ekf_filter_node_map` reutiliza solo `twist` de `/odometry/local`;
   - `navsat_transform.use_odometry_yaw=false`;
   - `localization_global_v2.launch.py` retrasa el arranque global para esperar TF local valido.

### Resultado de la correccion
Despues de publicar el GPS sintetico con `frame_id=base_footprint` y de dejar solo el perfil corregido:

- `stable`, 30 s en reposo:
  - `map->odom ≈ 1.5e-13 m`
  - `map->base ≈ 1.5e-13 m`
  - `odom->base ≈ 2.5e-16 m`
  - `/odometry/gps = 0`
  - estado `OK`

Eso elimina el runaway observado y deja la cadena global `v2` coherente para validacion de goals en `map`, `fromLL` y comportamiento comparativo entre `ideal`, `m8n` y `f9p_rtk`.

## Launches y archivos principales
Entrypoint global:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py
```

Script del workspace:

```bash
./tools/launch_sim_global_v2.sh
```

Archivos principales de la implementacion:

- `launch/sim_global_v2.launch.py`
- `launch/sim_nav_v2_base.launch.py`
- `launch/localization_global_v2.launch.py`
- `config/global_localization_v2.yaml`
- `config/nav2_global_v2_params.yaml`
- `config/rviz_global_v2.rviz`

## Arquitectura implementada
### Base comun de simulacion
`sim_local_v2` y `sim_global_v2` reutilizan el mismo bringup base:

1. `sim_v2_base.launch.py`
2. `sim_sensor_normalizer_v2`
3. `vehicle_controller_server` con `transport_backend=sim_gazebo`
4. `nav_command_server`
5. localizacion segun perfil
6. `nav_local_v2.launch.py`
7. RViz opcional

Esto mantiene la misma cadena de mando local en ambos perfiles:

```text
/cmd_vel
-> /cmd_vel_safe
-> nav_command_server
-> /cmd_vel_final
-> vehicle_controller_server
-> /cmd_vel_gazebo
-> Gazebo
-> /controller/drive_telemetry
-> ackermann_odometry
-> /odometry/local
```

### Perfil `sim_local_v2`
`sim_local_v2` hoy es un wrapper del bringup base con estos presets:

- `use_global_localization=False`
- `map_frame=odom`
- `fromll_frame=odom`
- `gps_profile=ideal`
- keepout local habilitado por default

### Perfil `sim_global_v2`
`sim_global_v2` es un wrapper del mismo bringup base con estos presets:

- `use_global_localization=True`
- `map_frame=map`
- `fromll_frame=map`
- `use_keepout=False`
- `nav2_params_file=config/nav2_global_v2_params.yaml`
- `gps_frame_id=base_footprint`

La localizacion global agrega:

- EKF local en `odom`
- `navsat_transform`
- `/odometry/gps`
- EKF global que publica `map -> odom`

El arbol TF esperado queda:

```text
map
└── odom
    └── base_footprint
```

## Contratos y configuracion global
### `nav_command_server`
La implementacion mantiene el mismo nodo y los mismos servicios publicos. El cambio para el perfil global es solo de preset:

- `map_frame=map`
- `fromll_frame=map`
- `gps_topic=/gps/fix`
- `forward_cmd_vel_safe_without_goal=True`

Eso hace que los goals LL se conviertan a `map` y no a `odom`.

### Nav2 global
`config/nav2_global_v2_params.yaml` refleja el modo global:

- `bt_navigator.global_frame=map`
- `bt_navigator.local_frame=odom`
- `behavior_server.global_frame=map`
- `behavior_server.local_frame=odom`
- `controller_server.odom_topic=/odometry/local`
- `global_costmap.global_frame=map`
- `local_costmap.global_frame=odom`

En esta fase el keepout global esta deshabilitado:

- `local_costmap.filters=["keepout_filter"]`
- `global_costmap.filters=["keepout_filter"]`
- `keepout_filter.enabled=false` en ambos costmaps

### RViz global
`config/rviz_global_v2.rviz` usa:

- `Fixed Frame = map`
- TF `map -> odom -> base_footprint`
- `/plan`
- `/odometry/local`
- `/odometry/gps`
- costmaps local y global

## GPS simulado por presets
`sim_global_v2` expone una API publica chica:

- `use_sim_time`
- `use_rviz`
- `rviz_config`
- `gps_profile`
- `world`
- `nav_start_delay_s`

El parametro `gps_profile` se resuelve dentro de `sim_sensor_normalizer_v2`, sin reintroducir `gazebo_utils` en la cadena `v2`.

### Base sintetica comun
La implementacion actual ya no toma al `NavSatFix` bridged desde Gazebo como verdad base del perfil.

En `sim_global_v2`, `sim_sensor_normalizer_v2`:

- toma `/odom_raw` como ground truth planar;
- carga el datum fijo desde `config/global_localization_v2.yaml`;
- genera un `NavSatFix` sintetico base a partir de `x/y/z`;
- publica `/gps/fix` a partir de esa base y del preset activo.

El topic `/gps/fix_raw` sigue existiendo como señal cruda del simulador, pero no define el comportamiento de `ideal`, `m8n` ni `f9p_rtk`.

## Localizacion global `v2`
La implementacion actual deja un unico perfil operativo para la cadena global. No quedan presets alternativos de fusion expuestos al usuario para evitar reintroducir el comportamiento defectuoso.

Configuracion efectiva:

- `ekf_filter_node_map` reutiliza solo `twist` local de `/odometry/local`;
- `navsat_transform.use_odometry_yaw=false`;
- `localization_global_v2.launch.py` retrasa el arranque de `navsat_transform` y `ekf_filter_node_map` para esperar TF local valido;
- `sim_global_v2.launch.py` fuerza `gps_frame_id=base_footprint` para que el GPS sintetico no herede el offset fisico de `gps_link`.

### Preset `ideal`
Objetivo:

- validar wiring, TF y flujo global sin ruido del sensor `navsat` del URDF

Comportamiento:

- usa el GPS sintetico base derivado de `/odom_raw + datum`;
- no agrega ruido artificial ni bias walk;
- covarianza forzada a preset chico:
  - `sigma_xy = 0.02 m`
  - `sigma_z = 0.05 m`

Interpretacion:

- `ideal` ya no depende del ruido del plugin GPS de Gazebo;
- no es covarianza cero;
- evita artefactos numericos;
- hace que RViz y el EKF global reflejen un baseline realmente ideal.

### Preset `m8n`
Objetivo:

- representar un GNSS comun sin RTK

Comportamiento:

- parte del mismo GPS sintetico base que `ideal`;
- ruido horizontal: `1.8 m`
- ruido vertical: `3.5 m`
- `publish_rate_hz = 5.0`
- jitter y bias walk activados
- covarianza alineada con ese mismo preset

### Preset `f9p_rtk`
Objetivo:

- aproximar un receptor RTK tipo F9P

Comportamiento:

- parte del mismo GPS sintetico base que `ideal`;
- ruido horizontal: `0.15 m`
- ruido vertical: `0.25 m`
- `publish_rate_hz = 8.0`
- jitter y bias walk bajos
- covarianza alineada con ese mismo preset

### Fallback base
El normalizador mantiene el fallback base para casos sin preset valido o mensajes auxiliares sin covarianza:

- horizontal: `2.5`
- vertical: `4.0`

Ese fallback no define la semantica de `ideal`, `m8n` o `f9p_rtk`. Los presets se construyen sobre la misma base sintetica y pisan explicitamente su covarianza.

## Validacion implementada
Tests automaticos agregados o ajustados:

- `test/test_sim_local_v2_launch.py`
- `test/test_sim_global_v2_launch.py`
- `test/test_sim_sensor_normalizer_v2.py`

Cobertura actual:

- `sim_local_v2` sigue siendo wrapper local-only
- `sim_global_v2` expone la API publica esperada
- el launch global cablea `navsat_transform` y EKF global
- Nav2 global queda en `map/odom`
- no se reintroducen helpers legacy
- la cadena global ya no expone el preset defectuoso que provocaba runaway
- los presets `ideal`, `m8n` y `f9p_rtk` resuelven y aplican su comportamiento esperado

## Comandos utiles
Mostrar argumentos del launch global:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py --show-args
```

Ejecutar con GPS ideal:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py gps_profile:=ideal
```

Ejecutar con perfil GNSS medio:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py gps_profile:=m8n
```

Ejecutar con perfil RTK:

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py gps_profile:=f9p_rtk
```

Chequeo de deriva en reposo sobre una sesion activa de `sim_global_v2`:

```bash
./tools/check_sim_global_drift.sh
./tools/check_sim_global_drift.sh --duration-s 30 --sample-interval-s 1.0 --output /tmp/sim_global_drift.json
```

El reporte resume:

- deriva de `map -> odom`
- deriva de `map -> base_footprint`
- deriva de `odom -> base_footprint`
- deriva de `/odometry/gps`
- consistencia entre `fromLL`, `/odometry/gps` y la fusion global
