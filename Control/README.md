# Control

This directory covers **vehicle control** on the real car. The README below keeps the original structure and incorporates today’s additions from the **software** and **hardware** work: explicit event/QoS fixes to stop message loss, the avoidance handshake, and concrete actuator-mapping details used on the car.

Subfolders
- `hardware/`: sensor/actuator specs, driver (servo/ESC) mapping, Simscape plant/verification.
- `software/`: controllers (Pure Pursuit primary), simulation (QLabs/Simulink/xytron), codegen to ROS 2, AEB × Obstacle Avoidance (Stateflow + TTC).

---

## Quick start
```bash
# (1) Hardware spec / driver mapping
sed -n '1,160p' hardware/README.md

# (2) Controller + Stateflow overview
sed -n '1,220p' software/README.md

# (3) Interface contract in this file
grep -n "^## 1) ROS 2 I/O" README.md
```

---

## 0) Goals & scope
- Stable steering/speed control on a **static track** with deterministic behavior.
- Keep **time-based waypoint streaming** separate from **events** (`/stop`, `/path_mode`, `/avoid_start`, `/avoid_done`) to avoid timing drift with the Simulink FSM.
- Before on-car runs, account for firmware filters/saturation/dead-band and mechanical backlash; include calibration and safety guards.

Reference background and experiments: `software/README.md` (“Scope & background”, “AEB × Obstacle Avoidance”).

---

## 1) ROS 2 I/O (interface contract)

### Subscriptions
- `/planned_path` (`nav_msgs/Path`) — sliding window of waypoints (`map` or `base_link`).
- `/path_x`, `/path_y` (`std_msgs/Float32`) — legacy axis streams.
- `/location` (`geometry_msgs/Pose2D`) — `(x, y, theta[rad])`.
- `/path_mode` (`std_msgs/Int32`) — `0: straight`, `1: curve`, `2: start/finish`.
- `/stop` (`std_msgs/Int32`) — `0/1`.
- `/avoid_start`, `/avoid_done` (`std_msgs/Bool`) — avoidance activity notifications from Obstacle_Avoidance.
- `/sim_time` (`std_msgs/Float64`) — optional timing hook.

### Publications
- `/simulinkOut` (`geometry_msgs/Point`)
  - `x = v_cmd` [m/s] speed command
  - `y = steer` [rad] steering command
  - `z = debug` (not used by the controller)
- `/scope` (`geometry_msgs/Point`) — diagnostics `(p0, p1, p2)` such as `cte`, `yaw_err`, `lookahead`.

Final PWM/servo/ESC actuation is performed by a **separate driver node** that consumes `/simulinkOut`.  
See `hardware/README.md` (“3) Actuation control”, “Actuator mapping”) for the exact mapping.

### QoS and timing fixes we actually applied
- **Paths/events/commands** → `reliable`, `keep_last(10)`.
- **Events** (`/stop`, `/avoid_*`, `/path_mode`) → `transient_local=true` so the last state survives node restarts.
- **/location** → `keep_last(5)` to prevent stamp inversion during bursts.
- **Executors** → single-threaded with **steady timers** for `/planned_path` consumption and control loops.
- **Clock discipline** → with bag replay, `use_sim_time=true` and launch a `/clock` provider first.

These changes removed the message “leaks” we saw at path transitions and during perception bursts.

---

## 2) Control profiles

### Pure Pursuit with mode scheduling
- Lookahead and speed
  - Straight (`path_mode=0`): `v_cmd = 0.30 m/s`, `L_d = 3.0 m`
  - Curve (`path_mode>0`): `v_cmd = 0.10 m/s`, `L_d = 0.10 m`
- Yaw-rate → steering via bicycle model + saturation  
  - Straight: `δ ∈ [-0.13, +0.13] rad`  
  - Curve:   `δ ∈ [-0.25, +0.25] rad`  
  - Yaw-rate limit: `0.5 rad/s`

### STOP chart
- On `/stop=1` → command speed to `0` immediately → resume after timeout.  
- Waypoints are time-streamed; the controller does not run distance-based reach checks.

### Tried and removed
- Stanley only → zig-zag under streaming + yaw noise.  
- Stanley ↔ Pure-Pursuit hybrid → switching spikes and extra stabilization.  
- Conclusion on this track: **Pure Pursuit + mode scheduling** is the most stable.

---

## 3)**software**

- **Path streaming is strictly index/time-based** (from `HelperPathSender`). The controller trusts the incoming window and never does reach gating.
- **Avoidance handshake**: when `/avoid_start` arrives, the controller switches to the avoidance path source; on `/avoid_done` it resumes the base stream. This keeps Simulink FSM timing deterministic.
- **Path mode map integration**: `PathSender` publishes `/path_mode` per segment; the controller applies the speed/lookahead schedule above without extra geometry checks.
- **Event debouncing upstream**: `/stop` and pickup/dropoff are already de-bounced in the sender; the controller remains stateless beyond STOP handling.

---

## 4)**hardware**

- **Steering mapping** in the driver node
  - `servo_cmd = steer_gain * (steer_rad - center_offset) + bias_backlash`
  - We tuned `center_offset` on the car by stopper sweeps; `bias_backlash` removes small-signal dead zone around center.
  - Typical values we used on QCar2:  
    `center_offset ≈ ±0.02 rad`, small **backlash bias** around zero, and linear `steer_gain` fitted from max-angle sweeps.
- **ESC mapping** in the driver node
  - Open-loop linearization with a **dead-band** threshold `v_start`.  
    Below `v_start` no motion is commanded; above it, `PWM` scales linearly with `v_cmd`.
- **Safety ordering**
  - On STOP or error, speed goes to zero first; steering may hold its last value so the car settles without oscillation.
- **Calibration workflow** now codified in bring-up (see §6).

All of this lives in the driver, keeping the Simulink controller itself simple and predictable.

---

## 5) Failures and fixes (timeline)

- V0 — Distance-based segment switching → timing jitter and FSM desync → moved to **timer/index-based streaming**.  
- V1 — Fixed lookahead only → over/under-steer on curves → added **mode scheduling**.  
- V2 — Steering guardrails → reduced excursions and chatter.  
- V3 — Separate STOP chart → removed residual effects of `/stop`.  
- V4 — Extra LPF in controller reduced authority because the vendor already filters → **removed** controller LPF.  
- V5 — On-car calibration of steering offset/gain and ESC dead-band → stable closed-loop tracking.

AEB × Obstacle Avoidance details are in `software/README.md` (“AEB × Obstacle Avoidance”).

---

## 6) Real-car bring-up checklist

- Steering calibration: stopper sweeps → measure max angle → compute `center_offset`, `steer_gain` → verify via `/scope`.  
- Speed calibration: step `v_cmd` (0→0.1→0.2 …) → log real speed → identify `v_start` dead-band → fit.  
- Yaw alignment: check `/location.theta` unit/sign/wrap and prevent ±π jumps.  
- Mode schedule: verify lower `L_d` and `v_cmd` in curves on the car.  
- STOP behavior: `/stop=1` must brake to standstill immediately and recover after timeout.  
- Avoidance activity: between `/avoid_start` and `/avoid_done`, follow only the avoidance path; resume base stream after.

---

## 7) Parameter guide

| Parameter | Default / Range | Notes |
|---|---:|---|
| Wheelbase `L` | 0.26 m | Retune on car |
| Max yaw-rate | 0.5 rad/s | Internal protection |
| Lookahead `L_d` straight | 3.0 m | Higher speed |
| Lookahead `L_d` curve | 0.10 m | Tight turns |
| `v_cmd` straight | 0.30 m/s | `path_mode=0` |
| `v_cmd` curve | 0.10 m/s | `path_mode>0` |
| Steering sat straight | ±0.13 rad | Guardrail |
| Steering sat curve | ±0.25 rad | Guardrail |

Small dead-band/backlash can delay tiny corrections; tune incrementally from logs.

---

## 8) Traceability
- Actuator/driver mapping → `hardware/README.md` §3, §5  
- Simscape plant/verification → `hardware/README.md` §4  
- Controller comparison/sim/codegen → `software/README.md` §1–§3  
- AEB × Obstacle Avoidance (Stateflow + TTC) → `software/README.md` §4  
- xytron observations/lessons → `software/README.md` §5

---

## 9) Conclusion
- Single channel `/simulinkOut` (`x: m/s`, `y: rad`) keeps sim→real consistent.
- On this hardware/track, **Pure Pursuit with mode scheduling** is the most robust.
- The practical fixes that mattered: reliable/transient event QoS, time-based streaming, removal of duplicate filtering, and actuator calibration in a dedicated driver.

---

## CHANGELOG
- Added the exact QoS profile and executor/timer choices that eliminated message loss.
- Documented the avoidance start/done handshake and **source selection** during avoidance.
- Inlined concrete actuator-mapping details used on the car (`center_offset`, `steer_gain`, backlash bias, `v_start`).
- Kept the original structure and expanded references to `software/` and `hardware/` for full context.
