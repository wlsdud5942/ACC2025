# Software — Controllers, Simulation Studies, Codegen, and AEB × Avoidance

This document presents our ideas and process on the software side: why we operate Pure Pursuit with mode scheduling, how we validated in simulation, how we transferred to the real car, and how **TTC-driven Stateflow** combines AEB and avoidance. Code is secondary; design reasoning is primary.

<!-- IMG_SUGGEST: controller_roles.png | below title | "Controller roles and operating envelopes" | alt="Roles of PID, Pure Pursuit, Stanley, MPC with strengths and limits" -->

## 0) Problem Framing & Goals
- Static indoor track; waypoints are **streamed in time** (no distance-based arrival test).  
- Keep the controller **loosely coupled** to the upstream planner via `/path_x`, `/path_y`, `/path_mode`, and `/stop`.  
- On real hardware, respect vendor **built-in filtering/saturation** and **mechanical backlash**—do not duplicate filters.

Principle: one output (`/simulinkOut`) from the controller; the driver owns PWM/ESC. AEB and avoidance compose as event-driven logic on top.

## 1) Interface Snapshot (for completeness)
**Subscribe**: `/path_x (Float32)`, `/path_y (Float32)`, `/location (Point: x,y,z=yaw[rad])`, `/path_mode (Int32)`, `/stop (Int32)`, `/sim_time (Float64)`  
**Publish**: `/simulinkOut (Point: x=v_cmd[m/s], y=steer[rad], z=debug)`, `/scope (geometry_msgs/Point) — diagnostics`

## 2) Why Pure Pursuit (and why not Stanley/MPC here)
Tried three routes:
- **Stanley only**: with streamed waypoints and yaw noise, produced zig-zag/oscillation; stabilizing required extra filtering that clashed with vendor filters.  
- **Hybrid Stanley ↔ Pure Pursuit**: switch boundaries introduced spikes and tuning overhead.  
- **MPC**: added compute and modeling cost with limited benefit at our speeds/constraints.

**Conclusion**: For this track and streaming design, **Pure Pursuit + mode scheduling** is the most stable, auditable, and easy to tune.

**Operating profile (same as Control spec)**
- Straight: `v_cmd = 0.30 m/s`, `L_d = 3.0 m`, steering sat ±0.13 rad  
- Curve:   `v_cmd = 0.10 m/s`, `L_d = 0.10 m`, steering sat ±0.25 rad  
- Internal yaw-rate limit: `0.5 rad/s`

## 3) Decision Layer: TTC-Driven Stateflow (AEB × Avoidance)
Idea: “stop/slow” and “avoid” share the same skeleton—detect risk → reduce speed → lateral maneuver or stop → re-enter nominal tracking. We encode this in **Stateflow** using **TTC** as the risk metric.

TTC  
`TTC = d_rel / max(v_rel, ε)`  
- `d_rel`: forward-projected distance along the guiding line (fusion of lane, depth, and SLAM when available).  
- `v_rel`: relative speed (ego minus obstacle estimate).  
- Thresholds: `T_hard` (immediate brake), `T_soft` (prepare to avoid).

State sketch  
CRUISE → [TTC < T_soft] AVOID_PREP  
CRUISE → [TTC < T_hard] AEB_BRAKE  
AVOID_PREP → AVOID_EXEC → RECENTER → CRUISE

Control pixel  
When depth/SLAM are momentarily unreliable, we generate near-term steering around a **control pixel** on the lane mask to keep avoidance stable at low latency.

<!-- IMG_SUGGEST: stateflow_ttc.png | end of this section | "TTC-based Stateflow overview" | alt="Stateflow chart for TTC-based AEB and avoidance" -->

## 4) Simulation Campaigns (how we de-risked)
- **QLabs**: closed-loop integration with synthetic sensors to debug timing and topic wiring.  
- **Simulink**: co-simulation of Pure Pursuit, the Stateflow chart, and a Simscape plant for rise-time/overshoot/yaw-rate studies.  
- **xytron**: fast baseline where **PID alone** runs stably; used as a regression bar before enabling the full stack.

## 5) Sim-to-Real Transfer (decisions we can defend)
- **Remove controller-side LPFs**; keep minimal steering saturation.  
- **Keep the driver separate**: `/simulinkOut` carries intent; driver nodes own PWM/ESC.  
- **Calibrate first**: estimate `center_offset`, `steer_gain`, `backlash_bias`, and `v_start/k_v` from logs before any gain tuning.  
- **Priority**: STOP dominates; only one command source is applied at a time.

## 6) Failure Timeline (what failed, what stuck)
- V0 distance-based switching → jitter/desync → time-based streaming.  
- V1 fixed lookahead → under/over-shoot on curves → mode scheduling.  
- V2 steering guardrails (dual saturation) → removed divergence/oscillation.  
- V3 separate STOP chart → removed residual `/stop` effects.  
- V4 LPF attempt (failed) → duplicate with vendor filters → removed.  
- V5 real-car calibration → stable and predictable.

## 7) Parameters (for traceability)
| Parameter | Value | Notes |
|---|---:|---|
| Wheelbase `L` | 0.26 m | Bicycle model effective length |
| Yaw-rate limit | 0.5 rad/s | Internal protection |
| Lookahead `L_d` (straight) | 3.0 m | With `v_cmd = 0.30 m/s` |
| Lookahead `L_d` (curve) | 0.10 m | With `v_cmd = 0.10 m/s` |
| Steering sat (straight) | ±0.13 rad | Guardrail |
| Steering sat (curve) | ±0.25 rad | Guardrail |

## 8) Reproducibility & Artifacts
- Stateflow chart (AEB × Avoidance) and controller scripts live under `controllers/` and `stateflow/`.  
- Simscape parameter files and comparison scripts live under `hardware/simscape/`.  
- ROS 2 launch files keep interface names identical to the Control spec.

