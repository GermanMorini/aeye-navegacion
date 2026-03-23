# Pixhawk Odometry + GPS RTK Diagnostic (Sim v2)

## Scope
- This note documents the current simulation assumptions before wiring `/odometry/pixhawk` into the local EKF.
- No GPS parameters are changed in this iteration.

## 1) New simulated odometry source (`pixhawk_odometry`)
- New ROS 2 node: `pixhawk_odometry` (`navegacion_gps/pixhawk_odometry.py`).
- Input: `/odom_raw` (`nav_msgs/Odometry`).
- Output: `/odometry/pixhawk` (`nav_msgs/Odometry`).
- Default frame IDs: `odom -> base_footprint`.
- Model behavior:
  - publish rate default `50 Hz`,
  - configurable output latency queue (`latency_s`),
  - Gaussian noise in position, velocity and yaw,
  - random-walk bias drift in position, velocity and yaw rate,
  - configurable pose/twist covariance for future EKF fusion.

### Why 50 Hz
- MAVLink `LOCAL_POSITION_NED` is a local filtered state estimate commonly streamed at tens of Hz in real integrations.
- In this project we use `50 Hz` as a practical default to mimic a responsive FCU local-state feed.

Reference:
- MAVLink message semantics for `LOCAL_POSITION_NED` (filtered local position/velocity in NED): https://mavlink.io/en/messages/common.html

## 2) Pixhawk 6X characteristics used for fidelity assumptions
- `pixhawk_odometry` is not raw IMU emulation; it emulates a filtered FCU-like odometry stream.
- Hardware assumptions grounded on Pixhawk 6X architecture:
  - FMU: STM32H753,
  - multiple onboard IMUs,
  - onboard barometer/magnetometer.

Reference:
- Holybro Pixhawk 6X Technical Specification: https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification

## 3) Current simulated GPS noise in this repository
File: `models/cuatri_real.urdf`
- Sensor: `<sensor type="navsat" name="gps_sensor">`
- Position noise configured:
  - horizontal stddev: `0.0000005`
  - vertical stddev: `0.00000125`
- Velocity noise configured:
  - horizontal stddev: `0.00000025`

Interpretation of units:
- SDFormat specifies NAVSAT position noise in **meters** and velocity noise in **m/s**.

Reference:
- SDFormat sensor spec (`navsat`): https://sdformat.org/spec/1.7/sensor/

## 4) RTK comparison baseline (F9P)
Reference RTK accuracy for u-blox ZED-F9P:
- Horizontal RTK: `0.01 m + 1 ppm CEP`
- Vertical RTK: `0.01 m + 1 ppm CEP`

Reference:
- u-blox ZED-F9P-01B datasheet: https://content.u-blox.com/sites/default/files/documents/ZED-F9P-01B_DataSheet_UBX-17051259.pdf

## 5) Important pipeline effect in `sim_sensor_normalizer_v2`
File: `navegacion_gps/sim_sensor_normalizer_v2.py`
- If incoming `NavSatFix.position_covariance` is all zeros, the node overwrites covariance with:
  - horizontal variance: `2.5` (sigma ≈ 1.58 m)
  - vertical variance: `4.0` (sigma = 2.0 m)
- This is much worse than RTK-level covariance.

## 6) Conclusion: is current sim GPS RTK-like?
- **Measurement noise configured in URDF navsat:** much smaller than typical RTK (over-optimistic).
- **If covariance arrives as zeros and gets normalized:** covariance metadata becomes non-RTK-like (meter-level).
- Therefore, the current pipeline is **not consistently RTK-like end-to-end**:
  - value noise may look unrealistically precise,
  - reported covariance can be conservative meter-level depending on incoming message covariance.

## 7) Practical next step (next iteration)
- Keep this iteration as diagnostic-only (no GPS tuning change).
- In the next step, align both:
  - navsat measurement noise,
  - published covariance,
  to a single RTK target envelope.
