# Obstacle_Avoidance — Process-Centric README

This module generates **temporary avoidance paths** when a forward obstacle blocks the nominal route (RRT waypoints / lane centerline). The design goal is to **stay within lane bounds**, apply the **minimum lateral offset**, and **rejoin** the original path smoothly, with timing that remains stable when coupled to a **Simulink FSM**.

---

## 0) Module Overview and Design Constraints

**Objective**
- When an obstacle lies on the planned path, create a lateral-offset curve that clears it with minimal deviation and return to the nominal path over a short distance.

**I/O Contract**
- **Inputs**
  - `/obstacle_info` (default: `[x, y, r]` in meters, `base_link` frame; r = effective radius/half-width)
  - `/location` (`geometry_msgs/Point` → `(x, y, yaw)` in `base_link`)
  - `dqn_paths.json` → `"total"` sequence of path IDs (global order)
- **Outputs**
  - `/path_x`, `/path_y` (streamed avoidance waypoints; time-based, no reach checks)
  - `/avoid_start`, `/avoid_done` (Boolean events announcing entry/exit of avoidance)

**Operational Constraints**
- Competition rule: **left (+) side** is the preferred avoidance direction.
- Due to synchronization issues with the Simulink FSM, **no distance-to-goal gating**; waypoints are **time-streamed**.
- Obstacles at **path-ID boundaries** must be handled: merge **current + next ID** into a **single full path** before computing offsets.

---

## 1) Problem Definition → Hypotheses

**Initial problems**
1) **Boundary fragility**: If avoidance operates only within the current path ID, obstacles near an **ID transition** cause late or failed avoidance.
2) **Timing instability**: Using **distance-based reach conditions** to switch paths desynchronizes with the **Simulink FSM** event timing.

**Hypotheses**
- **(G1)** Treat **current + next ID** as **one continuous curve** and compute offsets on that **full path** → removes boundary fragility.
- **(G2)** Remove reach gating; **stream waypoints on a timer** and signal **/avoid_start** and **/avoid_done** as the **only** synchronization hooks → reduces FSM timing mismatch.

---

## 2) Attempts → Failures → Improvements (Timeline)

**V0: ID-local avoidance (failed)**
- Active only inside the current ID; disabled at transitions.
- Symptom: late avoidance and discontinuous return near boundaries.
- Lesson: ID boundaries are artificial; planning must be continuous.

**V1: Fixed offset (constant d) (partial fail)**
- Apply the same lateral offset d near the obstacle.
- Symptom: over/under-clearance on curves; clearance not guaranteed.
- Lesson: Offset must depend on **local geometry**, point by point.

**V1.1: Normal-vector per-point offset (success with side effects)**
- For each path point `p_i`, compute normal `n_i` and set `p_i' = p_i + k·δ·n_i`.
- Increase `k = 1…max_k` until collision resolves → **minimum-motion** avoidance.
- Side effect: yaw discontinuities and sharp returns.

**V1.2: Segment obstacle model (stabilized)**
- Model the obstacle as a segment `[a, b]` of length `obs_length` oriented with local path tangent near the closest point.
- Collision if `dist(p_i, [a,b]) < r_obs + car_half (+ margin)`.
- Result: Robust to depth noise/shape variance; minimal `k` found faster.

**V2: Full-path merge (current + next ID) (key transition)**
- Build a single path across the boundary and compute offsets on it.
- Result: Boundary fragility gone; avoidance and return become **one curve**.

**V2.1: Yaw continuity + smoothed return**
- Compute `yaw_i = atan2(Δy, Δx)`; enforce continuity; apply short **spline/Bézier** smoothing on the last segment for `C1` continuity.
- Result: Suppresses steering jerks and zig-zag.

**V2.2: Timer-based streaming**
- Remove reach checks; publish `/path_x`, `/path_y` at a fixed timer period (e.g., 0.2 s).
- Announce transitions only via `/avoid_start`, `/avoid_done`.
- Result: Major reduction in FSM desync.

**V3: Hysteresis & debounce**
- Start avoidance only if `/obstacle_info` persists for **N consecutive frames**; ignore retriggers while avoiding.
- Result: Eliminates flicker due to depth bursts.

**V3.1: (Optional) Speed-dependent margin**
- `thresh = r_obs + car_half + α·v` (v = speed estimate).
- Start earlier at higher speeds.

**V3.2: (Optional) Multi-obstacle support**
- Accept repeated `[x, y, r]` triplets; select the candidate **closest to the path** (or worst clearance) first.

---

## 3) Current Algorithm (Detailed)

**3.1 Full path composition**
- Given `selected_id`, concatenate `cur_pts` with the next ID’s `next_pts[1:]` → `full_pts`.
- Start point: `start_pt = current_pose[0:2]`
- End point: `end_pt = last point of next_pts` (or `cur_pts[-1]` if no next ID)

**3.2 Obstacle segment model**
- Find `idx_obs = argmin_i || full_pts[i] − c_obs ||`.
- Local tangent: `tangent = full_pts[idx_obs+1] − full_pts[idx_obs−1]`, `dir_vec = normalize(tangent)`.
- Segment endpoints:  
  `a = c_obs − (obs_length/2) · dir_vec`, `b = c_obs + (obs_length/2) · dir_vec`.
- Collision threshold: `thresh = r_obs + car_half (+ safety margin)`.

**3.3 Distance to segment**
- `d(p, [a,b]) = || p − ( a + clip( dot(p−a, b−a) / ||b−a||^2 , 0, 1 ) · (b−a) ) ||`.

**3.4 Per-point offset (left-priority)**
- Normal at `p_i`: rotate `(p_{i+1} − p_{i−1})` by +90°, then normalize.
- For `k = 1 … max_k`:  
  `cand = p_i + k·δ·normal`.  
  Choose the **smallest k** such that `d(cand, [a,b]) ≥ thresh`.
- This implements **minimum lateral motion** to clear the obstacle, with **left-side preference**.

**3.5 Yaw and return smoothing**
- `yaw_i = atan2(y_{i+1} − y_i, x_{i+1} − x_i)` for continuity across the sequence.
- On the final segment, optionally smooth with a short **spline/Bézier** to ensure a gentle rejoin to the nominal path.

**Implementation note**  
All steps map directly to the code path in `compute_offset_points()` and the streaming logic in the timer callback.

---

## 4) Parameters and Tuning Log (Guide)

Parameter | Meaning | Tuning Guidance
--- | --- | ---
`car_half` | Half vehicle width | Core to clearance. Set conservatively (half of the widest footprint).
`obs_length` | Obstacle segment length | Represents box/sign width. Too small → miss; too large → over-avoid.
`delta` | Offset step (m) | Smaller = precise but slower. Curvy segments: 0.03–0.07 recommended.
`max_k` | Max step index | Ensures `delta * max_k` covers the largest expected lateral shift.
`reach_threshold` | End proximity tolerance | For `/avoid_done`. 0.15–0.30 m to account for control delay.

**Logging recommendation**  
Log timestamp, obstacle pose, chosen `k`, minimum clearance, generated path length, traveled distance/time from `/avoid_start` to `/avoid_done` → `logs/avoid_YYYYMMDD.csv`.

---

## 5) ROS2 Design (FSM Integration)

**Handshake**  
While avoidance is active, the **base path stream** (e.g., from `helper_path_sender`) should be paused or ignored.

**Integration options**
1) **Helper-pauses**: `helper_path_sender` subscribes to `/avoid_start` and halts base streaming; resumes on `/avoid_done`.
2) **Controller-prioritizes**: Controller always consumes an **“avoidance-priority queue”** first; base stream is secondary.

**QoS**
- Avoidance events: `reliable`, `transient_local` if late joiners must see the last state.
- Waypoint streams: use the same QoS as the base planner to keep behavior consistent.

---

## 6) Failure Cases and Mitigations

Case | Symptom | Mitigation
--- | --- | ---
Depth spikes | Flicker-avoidance triggers | Hysteresis (N-frame persistence); ignore retriggers during active avoidance.
Left-side margin insufficient | `max_k` exhausted without clearance | Fail-safe: command **stop FSM**, retry on next frames or allow right-side fallback if rules permit.
Boundary chatter | ID switching near nearest-point test | Use **arc-length nearest**; compute on **full path merge**.
Harsh return | Yaw discontinuity | Smooth last 5–10 points; spline/Bézier to enforce `C1` continuity.

---

## 7) Topics Summary (current setup)

Topic | Direction | Type | Notes
--- | --- | --- | ---
`/obstacle_info` | Sub | `std_msgs/Float32MultiArray` | `[x, y, r]` in meters, `base_link`; supports repeated triplets for multi-obstacle
`/location` | Sub | `geometry_msgs/Point` | `(x, y, yaw)` in `base_link`
`/path_x`, `/path_y` | Pub | `std_msgs/Float32` | Time-based streaming of avoidance waypoints
`/avoid_start`, `/avoid_done` | Pub | `std_msgs/Bool` | Event boundaries for FSM/Planner integration
---

