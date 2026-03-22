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

## Problema critico corregido: runaway de `map -> odom` en reposo
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
La correccion base que elimino el runaway en reposo tiene tres partes:

1. `sim_global_v2` ya no expone perfiles de localizacion global alternativos.
   Solo queda el camino corregido.
2. El GPS sintetico de la cadena global se publica con `frame_id=base_footprint`.
   Esto hace que la semantica del mensaje coincida con la verdad base usada para construirlo.
3. `localization_global_v2.launch.py` retrasa el arranque global para esperar TF local valido.

### Resultado de la correccion
Despues de publicar el GPS sintetico con `frame_id=base_footprint` y de dejar solo el perfil corregido:

- `stable`, 30 s en reposo:
  - `map->odom ≈ 1.5e-13 m`
  - `map->base ≈ 1.5e-13 m`
  - `odom->base ≈ 2.5e-16 m`
  - `/odometry/gps = 0`
  - estado `OK`

Eso elimino el runaway observado en reposo y dejo la cadena global `v2` coherente para validacion de `fromLL`, goals en `map` y comparativas entre `ideal`, `m8n` y `f9p_rtk`.

## Problema critico corregido: desalineacion global bajo movimiento con `gps_profile:=ideal`
Despues de corregir el runaway en reposo aparecio un segundo problema, distinto y mas sutil: al enviar un `NavigateToPose`, Gazebo mostraba una trayectoria razonable, pero en RViz la pose global se desalineaba violentamente y Nav2 terminaba abortando.

### Sintomas observados
- en Gazebo el robot avanzaba hacia el objetivo en forma fisicamente razonable;
- en RViz el robot y la trayectoria global mostraban saltos o giros violentos;
- `NavigateToPose` terminaba abortado con:
  - `Resulting plan has 0 poses in it`
  - `Pose Goes Off Grid`
- durante ese fallo:
  - `/odometry/local` seguia una trayectoria coherente;
  - `/odometry/gps` quedaba cerca de `/odometry/local`;
  - `/odometry/filtered` en `map` se alejaba decenas de metros;
  - `map -> odom` era la transform que corria.

### Evidencia medida
Durante la reproduccion se midio en paralelo:

- TF `map -> odom`
- TF `map -> base_footprint`
- `/odometry/local`
- `/odometry/gps`
- `/odometry/filtered`
- `/global_costmap/costmap`

El patron repetido fue este:

- al inicio:
  - `/odometry/local ≈ (0.699, 0.014)`
  - `/odometry/gps ≈ (0.724, 0.014)`
  - `/odometry/filtered ≈ (1.456, 0.014)`
  - `map -> odom ≈ (0.828, 0.005)`
- ya en movimiento:
  - `/odometry/local ≈ (6.626, 3.497, yaw 1.527)`
  - `/odometry/gps ≈ (6.822, 3.351)`
  - `/odometry/filtered ≈ (41.181, -40.001, yaw 1.499)`
  - `map -> odom ≈ (34.457, -43.307, yaw -0.029)`

El `global_costmap` no era el origen del fallo:

- el costmap global estaba presente;
- su frame era `map`;
- su ventana rodante seguia al robot;
- quien ya llegaba mal al costmap era la pose global producida por el EKF.

### Causa raiz
La investigacion termino mostrando que el problema no estaba en el planner ni en el costmap, sino en la base sintetica usada por el perfil `ideal`.

Antes de la correccion:

1. `sim_sensor_normalizer_v2` generaba `/gps/fix` de `ideal` a partir de `/odom_raw + datum`.
2. La cadena global `v2` navegaba y se orientaba contra `/odometry/local`.
3. Bajo maniobra, `/odom_raw` y `/odometry/local` no coincidían exactamente, aunque la diferencia fuera chica y estable.

Medicion concreta de esa discrepancia:

- `local_vs_raw_xy ≈ 0.26 m`
- `local_vs_raw_yaw ≈ 0.124 rad`

Esa diferencia parecia chica, pero el EKF global la amplificaba de manera acumulativa bajo movimiento. El resultado practico era:

- Gazebo seguia una trayectoria razonable;
- `/odometry/local` seguia coherente;
- `/odometry/gps` seguia razonablemente cerca;
- pero `map -> odom` y `/odometry/filtered` se iban desalineando cada vez mas.

### Ajustes descartados
Antes de encontrar la causa raiz se probaron varias alternativas que no cerraron el problema:

- `magnetic_declination_radians=0.0`
- `navsat_transform.delay=0.5`
- desacople del EKF global a `twist` local solamente
- `navsat_transform.use_odometry_yaw=true`
- reintroducir yaw absoluto local en el EKF global
- inyectar IMU directa en el EKF global

Los cuatro primeros cambios quedaron porque ayudan a mantener coherente la cadena `v2`, pero no resolvian por si solos el aborto en movimiento. Reintroducir yaw absoluto local y fusionar IMU directa en el EKF global empeoraron el comportamiento y quedaron descartados.

### Solucion aplicada
La correccion que mejoro claramente el caso para `gps_profile:=ideal` fue mover la base del GPS sintetico ideal desde `/odom_raw` a `/odometry/local`.

La implementacion efectiva hoy es:

- `gps_profile:=ideal`
  - usa `/odometry/local + datum` como base sintetica;
  - mantiene covarianza chica explicita;
  - sigue ignorando el ruido del plugin GPS del URDF;
  - deja de inyectar una discrepancia estructural contra la odometria local.
- `gps_profile:=m8n` y `gps_profile:=f9p_rtk`
  - siguen partiendo de `/odom_raw + datum`;
  - aplican su degradacion propia arriba de esa base;
  - permanecen como perfiles de realismo, no como baseline limpio de integracion.

### Resultado de la correccion
Con esa correccion, el comportamiento mejoro de forma medible:

- la separacion `/odometry/filtered` vs `/odometry/local` dejo de explotar a `100+ m` en los primeros segundos;
- la divergencia se mantuvo mas cerca del orden de `20 m` que del orden de `100 m`;
- el sistema dejo de desarmarse tan rapido como antes.

En una validacion intermedia, el goal de prueba `map -> (12.0, 8.0)` llego a devolver `SUCCEEDED`, pero esa observacion no se mantuvo bajo validacion mas estricta.

### Validacion posterior mas estricta
Al repetir las pruebas desde simulacion limpia, esperando tambien unos segundos despues del resultado del action para evitar contar exitos falsos, la navegacion global siguio fallando.

Resultados medidos:

- goal `(12.0, 8.0)`: `ABORTED`
  - `final_filtered ≈ (5.873, -25.214)`
  - `final_local ≈ (-64.791, 84.937)`
- goal `(15.0, 10.0)`: `ABORTED`
  - `final_filtered ≈ (-10.251, -1.539)`
  - `final_local ≈ (-38.601, 66.033)`
- goal `(18.0, 12.0)`: `ABORTED`
  - `final_filtered ≈ (7.452, -0.539)`
  - `final_local ≈ (-31.706, 121.135)`
- goal `(20.0, 0.0)`: una corrida rapida devolvio `SUCCEEDED`, pero al repetirla con espera post-resultado termino `ABORTED`
  - `final_filtered ≈ (-2.520, -8.142)`
  - `final_local ≈ (-107.996, 21.822)`

Conclusión de esta validación:

- la correccion de `ideal` mejora la estabilidad relativa de la cadena global;
- pero no alcanza todavia para considerar cerrada la navegacion global de mision;
- para goals medianos y largos, `/odometry/local` y `/odometry/filtered` siguen separandose de forma severa.

### Estado actual
La cadena global efectiva que se usa hoy para `gps_profile:=ideal` es:

- `/gps/fix` sintetico con `frame_id=base_footprint`
- base sintetica `ideal` derivada de `/odometry/local + datum`
- `magnetic_declination_radians=0.0`
- `navsat_transform.use_odometry_yaw=true`
- EKF global reutilizando solo `twist` local de `/odometry/local`
- arranque global retrasado para esperar TF local valido

Esto deja un estado materialmente mejor que el anterior, pero todavia incompleto:

- `map -> odom` deja de escaparse en reposo;
- Gazebo y RViz dejan de divergir tan violentamente en los primeros segundos;
- el baseline `ideal` queda mejor definido como herramienta de integracion;
- la navegacion prolongada sigue abierta.

Lo que queda pendiente ya no es este problema puntual, sino:

- seguir instrumentando por que `/odometry/local` y `/odometry/filtered` terminan en mundos distintos durante trayectorias medianas y largas;
- repetir la validacion con `m8n` y `f9p_rtk`;
- decidir si esos perfiles deben seguir apoyandose en `/odom_raw` o si tambien conviene alinearlos con `/odometry/local` para pruebas de integracion.

## Problema operativo corregido: `collision_monitor` quedaba `inactive`
Durante la investigacion del goal en `map` aparecio otro problema que no era de localizacion: Nav2 publicaba `/cmd_vel`, pero el robot no se movia.

### Sintomas observados
- `NavigateToPose` era aceptado;
- `/cmd_vel` tenia comandos validos;
- no aparecian `/cmd_vel_safe` ni `/cmd_vel_final`;
- `vehicle_controller_server` quedaba en `control_source=NONE`;
- el robot no se movia aunque la planificacion estuviera sana.

### Diagnostico
El problema no estaba en `nav_command_server` ni en el backend de simulacion. El nodo `collision_monitor` quedaba en estado lifecycle `inactive`, asi que nunca publicaba `/cmd_vel_safe`.

Evidencia:

- `ros2 lifecycle get /collision_monitor` devolvia `inactive [2]`;
- `ros2 node info /collision_monitor` mostraba el publisher de `/cmd_vel_safe`, pero no habia mensajes en runtime;
- al activar manualmente el nodo con `ros2 lifecycle set /collision_monitor activate`, la cadena completa volvia a funcionar:
  - `/cmd_vel_safe`
  - `/cmd_vel_final`
  - `/cmd_vel_gazebo`
  - `DriveTelemetry.drive_enabled=True`

### Solucion aplicada
En [nav_local_v2.launch.py](launch/nav_local_v2.launch.py) se retraso el arranque del lifecycle manager dedicado al `collision_monitor`:

- `COLLISION_MONITOR_START_DELAY_S = 2.0`

Con ese delay el nodo ya alcanza a quedar listo antes del `autostart`, y el estado final pasa a ser `active` desde el arranque, sin activacion manual.

### Resultado
Despues de este cambio:

- `collision_monitor` queda `active` por default;
- reaparece `/cmd_vel_safe`;
- `nav_command_server` vuelve a publicar `/cmd_vel_final`;
- `vehicle_controller_server` cambia a `control_source=PI`.

## Reproduccion historica del problema bajo movimiento
La reproduccion que permitio cerrar este caso mostro que el sintoma "RViz se vuelve loco mientras Gazebo sigue una trayectoria razonable" no era solo visual.

Durante una prueba de `NavigateToPose` con goal en `map`, antes de mover la base `ideal` a `/odometry/local`, se midio:

- `/odometry/local` coherente y monotona:
  - aprox. `(12.33, 15.41, yaw 1.15)` al final de la ventana
- `/odometry/gps` coherente con la local:
  - aprox. `(13.31, 14.85)`
- `/odometry/filtered` divergente:
  - aprox. `(-2.39, 15.34, yaw 0.03)` al final
- `map -> odom` con norma maxima de `36.315 m`
- separacion maxima `/odometry/filtered` vs `/odometry/local` de `21.596 m`
- el `global_costmap` siguio reportando al robot dentro de su ventana rodante en todas las muestras

Interpretacion:

- Gazebo sigue principalmente la cadena de control fisica;
- RViz refleja la pose global fusionada;
- cuando la fusion global se descompensa, la visualizacion explota aunque el robot real del simulador siga avanzando.

### Hipotesis descartadas
- No parece ser un offset fijo entre IMU y odometria local al arrancar.
  En reposo, inmediatamente despues del launch, se midio:
  - `imu_yaw = -0.0`
  - `local_yaw = 0.0`
  - diferencia `≈ 0.0 rad`
- Reintroducir yaw absoluto local en `ekf_filter_node_map.odom0_config` no cerro el problema.
  En las pruebas siguientes el sistema siguio abortando y la configuracion no mejoro la estabilidad.
- Inyectar IMU directa en el EKF global tampoco cerro el problema.
  En las variantes probadas, la divergencia subio a `100+ m`, asi que esa familia de cambios quedo descartada.

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

### Base sintetica de GPS
La implementacion actual ya no toma al `NavSatFix` bridged desde Gazebo como verdad base del perfil.

En `sim_global_v2`, `sim_sensor_normalizer_v2`:

- carga el datum fijo desde `config/global_localization_v2.yaml`;
- genera un `NavSatFix` sintetico base a partir de odometria planar + datum;
- publica `/gps/fix` a partir de esa base y del preset activo.

Base usada por preset:

- `ideal`: `/odometry/local + datum`
- `m8n`: `/odom_raw + datum`
- `f9p_rtk`: `/odom_raw + datum`

El topic `/gps/fix_raw` sigue existiendo como señal cruda del simulador, pero no define el comportamiento de `ideal`, `m8n` ni `f9p_rtk`.

## Localizacion global `v2`
La implementacion actual deja un unico perfil operativo para la cadena global. No quedan presets alternativos de fusion expuestos al usuario para evitar reintroducir el comportamiento defectuoso.

Configuracion efectiva hoy:

- `sim_global_v2.launch.py` fuerza `gps_frame_id=base_footprint` para que el GPS sintetico no herede el offset fisico de `gps_link`;
- `localization_global_v2.launch.py` retrasa el arranque de `navsat_transform` y `ekf_filter_node_map` para esperar TF local valido;
- `navsat_transform.use_odometry_yaw=true`;
- `navsat_transform.magnetic_declination_radians=0.0`;
- `navsat_transform.delay=0.5`;
- el EKF global reutiliza solo `twist` local de `/odometry/local` mientras fusiona posicion absoluta desde `/odometry/gps`.

### Preset `ideal`
Objetivo:

- validar wiring, TF y flujo global sin ruido del sensor `navsat` del URDF

Comportamiento:

- usa el GPS sintetico base derivado de `/odometry/local + datum`;
- no agrega ruido artificial ni bias walk;
- covarianza forzada a preset chico:
  - `sigma_xy = 0.02 m`
  - `sigma_z = 0.05 m`

Interpretacion:

- `ideal` ya no depende del ruido del plugin GPS de Gazebo;
- `ideal` tampoco arrastra la pequena discrepancia estructural entre `/odom_raw` y `/odometry/local`;
- no es covarianza cero;
- evita artefactos numericos;
- hace que RViz y el EKF global reflejen un baseline realmente ideal.

### Preset `m8n`
Objetivo:

- representar un GNSS comun sin RTK

Comportamiento:

- parte de una base sintetica derivada de `/odom_raw + datum`;
- ruido horizontal: `1.8 m`
- ruido vertical: `3.5 m`
- `publish_rate_hz = 5.0`
- jitter y bias walk activados
- covarianza alineada con ese mismo preset

### Preset `f9p_rtk`
Objetivo:

- aproximar un receptor RTK tipo F9P

Comportamiento:

- parte de una base sintetica derivada de `/odom_raw + datum`;
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
