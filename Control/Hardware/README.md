# Hardware — Sensors, Actuation, and Sim-to-Real Evidence

This document explains the hardware layer so the Control Module can remain simple and robust. We focus on ideas, calibration processes, and experimental evidence; code is intentionally minimized.

<!-- IMG_SUGGEST: hardware_block.png | below title | "Sensing and actuation blocks feeding the controller" | alt="Block diagram of sensors and actuators connected to the controller" -->

## 0) What We Contribute
- Clean **driver abstraction**: one ROS 2 channel `/simulinkOut` (x: speed [m/s], y: steering [rad]) feeds the driver; the controller stays vendor-agnostic.
- **Field-calibrated mapping** that explicitly models **deadband** and **mechanical backlash** instead of hiding quirks in filters.
- A lightweight **Simscape plant** (bicycle lateral dynamics + ESC first-order delay) to justify safety limits and operating schedules.

## 1) Interface (hardware ⇄ control)
**Subscribe**: `/simulinkOut (geometry_msgs/Point)` → x = `v_cmd` [m/s], y = `steer` [rad], z = debug  
**Publish**: `/scope (geometry_msgs/Point)` — diagnostics

> PWM/ESC conversion happens here, not in the controller.

## 2) Sensor Suite and Roles
| Sensor | Role | Interface | Topics / Frames | Notes |
|---|---|---|---|---|
| Intel RealSense D435i | RGB-D + IMU | USB3 | `/camera/color/image_raw`, `/camera/depth/image_raw`, `camera_link` | Used upstream for lane/depth fusion |
| 2D LiDAR (Quanser/RPLIDAR **A2M12**) | SLAM & ranges | USB/UART | `/scan`, `base_scan` | Watch reflections/glass |
| IMU (onboard/external) | Attitude, yaw-rate | USB/I2C | `/imu`, `imu_link` | Bias/wrap management |
| 360° CSI cameras (4 × 160° FoV) | Curvature continuity | CSI | `/qcar/csi_*` | Multi-view geometry |

Frames: `map → odom → base_link → {base_scan, camera_link}`  
Power domains separated for compute, sensors, and actuation.  
Buses: USB (RGB-D/LiDAR), CSI (cameras), PWM/Serial/CAN (servo/ESC).

<!-- IMG_SUGGEST: frames_power.png | end of this section | "Frames, power, and bus layout" | alt="TF frames and power/IO bus layout" -->

## 3) Actuation Mapping (why the driver matters)
**Steering (rad → servo)**
- Model: `servo_cmd = steer_gain × (steer + center_offset) + backlash_bias`
- Saturation aligned with the controller: ±0.13 rad on straights, ±0.25 rad on curves

**Speed (m/s → ESC, open-loop)**
- Explicit deadband: `actual ≈ k_v × max(0, v_cmd − v_start)`
- We keep speed open-loop in this project; high-level scheduling proved sufficient.

**STOP and source selection**
- On `/stop = 1`, speed is forced to zero first (steering may hold).
- When avoidance runs, only one command source is applied at a time (simple source selection).

## 4) Calibration Protocols (what we actually did)
**Steering (no mechanical maximum-angle measurement)**
1. Straight-ahead zeroing: adjust `center_offset` so mean heading error on a straight ≈ 0.  
2. Curvature-based gain: on a mild constant-curvature section, fit `steer_gain` so demanded radians match the observed turning radius.  
3. Backlash observation (optional): apply small ±inputs to detect the threshold where motion starts; use as `backlash_bias` (or 0 if uncertain).

**Speed / ESC**
1. Apply steps `v_cmd = 0 → 0.10 → 0.20 …` m/s.  
2. Log actual speed to estimate `v_start` (deadband) and slope `k_v` with a first-order fit.

**Yaw consistency**
- Keep `/location.z` in radians with consistent sign and wrapping (±π). Mis-wraps create artificial kinks.

## 5) Plant Model (Simscape) and Evidence
Scope
- Longitudinal: vehicle mass and ESC first-order delay.  
- Lateral: kinematic bicycle with cornering stiffness; steering servo as a 2nd-order element.

Why it matters
- Quantifies safe yaw-rate and steering limits.  
- Guides conservative lookahead/speed pairs when tire slip increases.

Validation routine
- Step and ramp overlays (sim vs real) to check rise time, overshoot, and steady-state.  
- Parameters and scripts live in `hardware/simscape/` for reproducibility.

<!-- IMG_SUGGEST: simscape_validation.png | end of this section | "Sim vs real step response overlay" | alt="Overlay of simulated and real step responses" -->

## 6) Safety Design
- STOP path has priority: speed is forced to zero before anything else.  
- Exactly one command source is applied at a time.  
- Driver-side current/temperature and jerk protections.  
- Default logging: `/vehicle_state`, `/tf`, `/scan`, `/camera/*`, `/simulinkOut`, `/scope`.

## 7) Limitations and Next Steps
- Low-cost IMUs drift; better fusion would tighten yaw.  
- LiDAR reflections require intensity-aware filtering.  
- Open-loop speed mapping is surface-dependent; a drop-in closed-loop module is a natural extension if we keep this interface.

