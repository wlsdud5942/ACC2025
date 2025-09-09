# Control

This directory explains how we achieve stable low-speed steering and speed on a static track while keeping the controller portable and easy to review.

Subfolders
- `hardware/` — sensing and actuation, driver mapping, Simscape plant and evidence.
- `software/` — controllers (Pure Pursuit with mode scheduling), simulation studies, codegen strategy, and the AEB × avoidance decision layer (TTC + Stateflow).

<!-- IMG_SUGGEST: control_overview.png | below title | "Perception → Planning → Control overview" | alt="High-level architecture from perception to control" -->

## 0) Goals & Scope
- Stable control on a static indoor track.
- Loose coupling to the upstream planner via streamed waypoints and events (`/path_mode`, `/stop`).
- Respect real-car traits (vendor filtering/saturation, deadband, mechanical backlash) through calibration and safety at the driver layer.

## 1) ROS 2 Interface (contract)
**Subscribe**
- `/path_x` (`std_msgs/Float32`) — target waypoint X  
- `/path_y` (`std_msgs/Float32`) — target waypoint Y  
- `/location` (`geometry_msgs/Point`) — x, y, z = yaw [rad]  
- `/path_mode` (`std_msgs/Int32`) — 0 = straight, >0 = curve  
- `/stop` (`std_msgs/Int32`) — 0/1  
- `/sim_time` (`std_msgs/Float64`) — clock/trigger

**Publish**
- `/simulinkOut` (`geometry_msgs/Point`) — x = `v_cmd` [m/s], y = `steer` [rad], z = debug  
- `/scope` (`geometry_msgs/Point`) — diagnostics

> Final PWM/servo/ESC actuation is handled by a separate driver node that consumes `/simulinkOut`.

<!-- IMG_SUGGEST: io_contract.png | end of this section | "I/O contract and driver separation" | alt="ROS 2 topics and separation between controller and driver" -->

## 2) Control Profile (as operated)
Primary controller: **Pure Pursuit** with simple mode scheduling  
- Straight (`/path_mode = 0`): `v_cmd = 0.30 m/s`, lookahead `L_d = 3.0 m`, steering saturation ±0.13 rad  
- Curve   (`/path_mode > 0`): `v_cmd = 0.10 m/s`, lookahead `L_d = 0.10 m`, steering saturation ±0.25 rad  
- Internal yaw-rate limit: `0.5 rad/s`  
STOP behavior: `/stop = 1` sets speed to zero immediately and resumes after a short timeout (Stateflow).

## 3) Sim-to-Real Highlights
- Removed controller-side LPFs (vendor already filters/saturates); kept only minimal steering saturation.
- Mapping is handled in the driver: `rad → servo` and `m/s → ESC`. We calibrate `center_offset`, `steer_gain`, `backlash_bias`, and `v_start/k_v` from logs.
- When avoidance is active or STOP is asserted, only one command source is applied at a time; STOP has priority.

## 4) Failures → Fixes (timeline)
- V0 distance-based switching → timing jitter/desync → switched to **time-based streaming**.  
- V1 fixed lookahead → under/over-shoot on curves → **mode scheduling** added.  
- V2 steering guardrails (dual saturation) → removed divergence/oscillation.  
- V3 separate STOP chart → removed residual `/stop` effects.  
- V4 added LPF (failed) → duplicate with vendor filters → removed.  
- V5 real-car calibration → stable and predictable.

## 5) Parameters (for traceability)
| Parameter | Value | Notes |
|---|---:|---|
| Wheelbase `L` | 0.26 m | Bicycle model effective length |
| Yaw-rate max | 0.5 rad/s | Internal protection |
| `L_d` (straight) | 3.0 m | With `v_cmd = 0.30 m/s` |
| `L_d` (curve) | 0.10 m | With `v_cmd = 0.10 m/s` |
| Steering sat (straight) | ±0.13 rad | Guardrail |
| Steering sat (curve) | ±0.25 rad | Guardrail |

## 6) Traceability
- Actuation and calibration → `hardware/README.md`  
- Controllers, simulation, and Stateflow/TTC → `software/README.md`
