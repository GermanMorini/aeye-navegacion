# Navegacion Global V2 Ackermann

## Estado actual
`sim_global_v2` ya existe y sirve para validar la integracion de:

- `map -> odom -> base_footprint`
- `navsat_transform`
- EKF global
- goals RViz y `fromLL` sobre `map`
- perfiles de GPS simulados `ideal`, `m8n`, `f9p_rtk`

Todavia no esta cerrada como navegacion global robusta para trayectos medianos o largos.

## Lo que ya esta descartado
Estas hipotesis ya no son el foco principal:

- `sim_local_v2` roto
- `collision_monitor` inactivo en el arranque
- runaway puro de `map -> odom` en reposo
- `global_costmap` como causa principal
- un bug puramente visual de RViz como unica explicacion

## Hechos medidos que hoy siguen vigentes

### 1. `sim_local_v2` esta sano
Prueba larga en `odom` con goal `(18, 12)`:

- `goal_status = SUCCEEDED`
- `max_base_vs_local_m ~= 0.0216`
- `max_base_vs_local_yaw_rad ~= 0.00024`

Conclusion:

- la odometria/localizacion local base funciona bien en `sim_local_v2`
- el problema aparece cuando se agrega la capa global

### 2. La mejor variante global actual mejora mucho los goals cortos, pero no cierra los largos
Perfil experimental actual mas sano:

- `strict_split_lag_compensated_odom_yaw_no_delay_soft_gps`

Equivale a:

- `navsat_transform.use_odometry_yaw=true`
- `navsat_transform.delay=0.0`
- GPS ideal con covarianza menos agresiva

En goals cortos se vio una mejora clara en RViz y en el forense.
En goals largos sigue fallando.

### 3. En goals largos, `global` sigue bastante de cerca a `local`
Goal largo reciente `(18, 12)` con el perfil anterior:

- `goal_status = TIMEOUT`
- `goal_min_base_distance_m ~= 0.46`
- `goal_min_local_distance_m ~= 3.01`
- `goal_min_global_distance_m ~= 3.03`
- `largest_tf_filtered_disagreement_m ~= 0.136`

Conclusion:

- `global` y `local` no se separan mucho entre si
- el error grande no parece ser solo `map -> odom`

### 4. En esa misma prueba, `local` y `gps_odom` se van juntos respecto de la base real
Nueva metrica en `odom`, sin contaminarla con `map -> odom`:

- `odom_max_base_vs_local_gap_m ~= 8.50`
- `odom_max_base_vs_gps_odom_gap_m ~= 8.65`
- `odom_max_gps_odom_vs_local_gap_m ~= 0.19`

Conclusion:

- `/odometry/local` se separa mucho de la base simulada en posicion
- `/odometry/gps` casi sigue a `/odometry/local`
- la capa global no corrige ese error porque hoy el GPS ideal no es independiente

### 5. La orientacion de `/odometry/gps` no sirve como referencia visual
En RViz:

- flecha roja: `/odometry/local`
- flecha azul: `/odometry/gps`

Hoy `/odometry/gps` sale con yaw no util para interpretar heading del robot.
Sirve para posicion, no para leer orientacion.

## Hipotesis actual mas fuerte
El problema principal actual en `sim_global_v2` no es solo el EKF global.

La hipotesis mas fuerte hoy es esta:

1. en goals largos y curvos, `/odometry/local` se va respecto de la base real dentro de `sim_global_v2`
2. `gps_profile:=ideal` usa `ideal_from_local_odom`
3. por eso `/odometry/gps` hereda casi la misma deriva de `local`
4. el EKF global recibe dos fuentes muy parecidas entre si
5. entonces `global` termina siguiendo a `local` en vez de corregirlo

Dicho simple:

- la referencia absoluta `ideal` es demasiado circular
- y por eso el stack global no logra corregir la deriva larga

## Cambios y hallazgos que si conviene recordar

### Cambios utiles que mejoraron el comportamiento
- `navsat_transform.use_odometry_yaw=true`
- `navsat_transform.delay=0.0`
- GPS ideal con covarianza mas suave
- fix de stamps viejos en `polygon_stamped_republisher.py`
- fix del forense para no mezclar corridas cuando reinicia `/clock`

### Hallazgos utiles
- la variante `strict_split_lag_compensated_odom_yaw_no_delay_soft_gps` es la mejor base experimental actual
- los goals cortos mejoraron bastante con esa base
- los goals largos siguen mostrando deriva de `local/gps_odom` respecto de la base

## Siguiente paso recomendado
El siguiente experimento correcto es hacer que el GPS ideal sea realmente independiente de `local`.

Prioridad:

1. probar `ideal_from_raw_odom` en la misma bateria larga
2. repetir el forense largo con la metrica nueva en `odom`
3. verificar si `/odometry/gps` deja de seguir tan de cerca a `/odometry/local`

Si eso mejora, la causa principal quedara bastante reducida:

- no seria “solo Nav2”
- no seria “solo EKF global”
- seria principalmente la circularidad del GPS ideal en la cadena global

## Herramientas utiles

### Benchmark corto
```bash
./tools/benchmark_sim_global_v2.sh --profile strict_split_lag_compensated_odom_yaw_no_delay_soft_gps
```

### Forense principal
```bash
./tools/check_sim_global_ekf_forensics.sh \
  --profile strict_split_lag_compensated_odom_yaw_no_delay_soft_gps \
  --goal 12,8 \
  --goal-timeout-s 35 \
  --post-goal-duration-s 25 \
  --sample-interval-s 0.05
```

### Goal largo actual
```bash
./tools/check_sim_global_ekf_forensics.sh \
  --profile strict_split_lag_compensated_odom_yaw_no_delay_soft_gps \
  --goal 18,12 \
  --goal-timeout-s 70 \
  --post-goal-duration-s 15 \
  --sample-interval-s 0.05
```
