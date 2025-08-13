# Control Module

## 0) Goals & Scope
- Achieve **stable steering and speed control** on a static track.  
- Keep the controller **loosely coupled** to higher-level planning (waypoint **streaming**) and events (`/stop`, `/path_mode`) for robust timing.  
- After transferring to the real car, reflect hardware traits (built-in filters/saturation/deadband, mechanical backlash) with appropriate **calibration and safety** measures.

---

## 1) ROS2 I/O (unchanged)

### Inputs (Subscribe)
- `/path_x` (`std_msgs/Float32`) — target waypoint X  
- `/path_y` (`std_msgs/Float32`) — target waypoint Y  
- `/location` (`geometry_msgs/Point`) — `(x, y, z = yaw [rad])`  
- `/path_mode` (`std_msgs/Int32`) — `0 = straight`, `1 = curve (or higher)`  
- `/stop` (`std_msgs/Int32`) — `0/1`  
- `/sim_time` (`std_msgs/Float64`) — clock/trigger

### Outputs (Publish)
- **`/simulinkOut` (`geometry_msgs/Point`)**  
  - `x = v_cmd [m/s]` (speed command)  
  - `y = steer [rad]` (steering command)  
  - `z = debug` (not used for control)
- (Optional) `/scope` (`geometry_msgs/Point`) — diagnostic values

> The final actuation (PWM/servo/ESC) is handled by a **separate driver node** that consumes `/simulinkOut`.

---

## 2) Control Algorithms (essentials)

### Pure Pursuit (final primary controller)
- **Lookahead scheduling**
  - Straight (`path_mode = 0`): `v_cmd = 0.30 m/s`, `L_d = 3.0 m`  
  - Curve (`path_mode > 0`): `v_cmd = 0.10 m/s`, `L_d = 0.10 m`
- **Yaw-rate → steering** (bicycle-model approximation) + **steering saturation**
  - Straight: `δ ∈ [-0.13, +0.13] rad`  
  - Curve: `δ ∈ [-0.25, +0.25] rad`  
  - Internal yaw-rate limit: `0.5 rad/s`
- **STOP state machine**
  - `/stop = 1` → **immediately set speed to 0** → resume after timeout (Stateflow chart in code)

> Waypoints are **streamed over time** by an upstream node with **no distance-based arrival test**. The controller simply tracks the incoming coordinates.

### Stanley / hybrid attempts (summary)
- **Stanley only**: with streaming + yaw noise, showed zigzag/oscillation → **shelved**.  
- **Stanley ↔ Pure Pursuit hybrid**: spikes at switching boundaries → stabilization cost ↑ → **shelved**.  
- **Conclusion**: For this track and streaming design, **Pure Pursuit + mode scheduling** is the most stable.

---

## 3) Sim-to-Real Transfer

### (A) Remove LPF · Minimize saturation
- The vehicle firmware already applies **built-in filters/saturation**. Adding controller-side LPFs/saturation **attenuated or nullified** commands.  
- **Action**: **Remove all LPFs** in our controller and keep only **minimal steering saturation** (limits above).

### (B) Actuator mapping (driver node)
- `/simulinkOut` → servo/ESC conversion happens in a **separate node**.
- **Steering**: `rad → servo command`
  - Parameters: `center_offset (rad)`, `steer_gain (cmd/rad)`, **backlash compensation** (small-signal bias)  
  - Calibration: sweep to left/right stoppers → measure max steering angles → **linear fit** → tune **center offset**
- **Speed (forward/stop)**: `v_cmd [m/s] → ESC`
  - **Deadband** present: motion only for `v_cmd > v_start`  
  - Initial calibration: build **first-order fit** of `v_cmd` vs measured speed (open loop)  
  - This project **does not use a closed-loop speed controller** (FSM/scheduling based)

### (C) Real-world parameter retuning
- **Effective wheelbase `L`**: adjust by a few mm to reflect the true rotation center  
- **Tire slip/friction**: on tight curves, prefer smaller `L_d` and `v_cmd`  
- **Yaw alignment**: keep `/location.z` radians/sign/wrap (±π) **consistent**

### (D) Safety · Gating
- On STOP/error, **gate speed to zero** first (steering may be held).  
- When the avoidance module uses `/avoid_start` / `/avoid_done`, **gate the base path stream** so only one source is consumed.

---

## 4) Failures & Fixes (sim → real timeline)
- **V0 – Distance-based segment switching**: timing jitter → event desync → ❌ → switch to **time-based streaming**  
- **V1 – Fixed lookahead**: under-/overshoot on curves → ❌ → add **mode scheduling** (`L_d`, `v_cmd`)  
- **V2 – Steering guardrails**: suppress divergence/oscillation (dual saturation) → ✅  
- **V3 – Separate STOP chart**: removed residual `/stop` effects → ✅  
- **V4 – Added LPF (failed)**: duplicated with vendor filters → weakened commands → ❌ → **remove LPF**, keep saturation  
- **V5 – Real-car calibration**: steering offset/gain & ESC deadband tuned → ✅

---

## 5) Real-Car Bring-Up Checklist
- **Steering calibration**  
  - Manually sweep to stoppers → measure max angles  
  - Compute `center_offset`, `steer_gain` → verify via `/scope`
- **Speed calibration**  
  - Step `v_cmd` (0→0.1→0.2 …) and log **measured speed**  
  - Identify **deadband (`v_start`)** and create a **first-order fit**
- **Yaw alignment**  
  - Verify `/location.z` radians/sign; enable **wrap handling** (avoid ±π jumps)
- **Mode-schedule verification**  
  - Confirm `L_d↓` and `v_cmd↓` actually apply on curves
- **STOP behavior**  
  - `/stop = 1` → immediate decel → stop; resume after timeout
- **Gating**  
  - When avoidance is active, confirm the base path stream is **paused/ignored**

---

## 6) Parameter Guide 

| Parameter | Default / Range | Notes |
|---|---:|---|
| Wheelbase `L` | 0.26 m | Tunable on the real car |
| Max yaw-rate | 0.5 rad/s | Internal protection |
| Lookahead `L_d` (straight) | 3.0 m | High-speed straight |
| Lookahead `L_d` (curve) | 0.10 m | Tight curves |
| `v_cmd` (straight) | 0.30 m/s | `path_mode = 0` |
| `v_cmd` (curve) | 0.10 m/s | `path_mode > 0` |
| Steering sat (straight) | ±0.13 rad | Safety |
| Steering sat (curve) | ±0.25 rad | Safety |

> Due to **deadband/backlash**, fine adjustments may not take effect on the real car. Tune parameters **incrementally based on logs**.

---

## 7) Conclusion
- The system keeps sim→real consistency via a **single `/simulinkOut` channel** (`x: m/s`, `y: rad`).  
- Key lessons from the real car: **avoid filter duplication (remove LPFs)** and **calibrate actuators first**.  
- Given the track and hardware, **Pure Pursuit + mode scheduling** was the most **stable**, while Stanley/hybrids required significant extra stabilization.
