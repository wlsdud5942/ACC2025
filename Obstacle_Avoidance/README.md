# Obstacle_Avoidance

This module generates **temporary avoidance paths** when a forward obstacle blocks the nominal route (RRT waypoints or lane centerline). The goals are to **stay within lane bounds**, apply the **minimum lateral offset**, and **rejoin** the nominal path smoothly, with timing that remains stable when coupled to a **Simulink FSM**.

---

## 0) Module Overview and Design Constraints

**Objective**
- When an obstacle lies on the planned path, create a lateral-offset curve that clears it with minimal deviation and return to the nominal path over a short distance.

**Frames**
- Default computation and I/O are in **`base_link`**. This suits short-lived local avoidance that is streamed over time.
- Optional: set `output_frame_id = "map"` or `"odom"` to emit global-frame paths (transform is taken from TF at generation time).

**I/O Contract**
- **Inputs**
  - **`/obstacle_info`** (`std_msgs/Float32MultiArray`): one or more triplets `[x, y, r]` in meters, **`base_link`** frame by default; `r` = effective radius/half-width.
  - **`/location`** (`geometry_msgs/Pose2D`): `(x, y, theta)` in **`base_link`** (theta in radians; if a global frame is used, provide pose in that frame consistently).
  - **`dqn_paths.json`**: path repository and a global ID order. See **Appendix A** for schema.
- **Outputs** (choose via parameters; both can be enabled)
  - **Legacy**: `/path_x`, `/path_y` (`std_msgs/Float32`), time-based streaming of avoidance waypoints in the selected `output_frame_id`.
  - **Standard**: `/avoid_path` (`nav_msgs/Path`), emits a short horizon of `geometry_msgs/PoseStamped` points. `header.frame_id = output_frame_id`.
  - Events: `/avoid_start`, `/avoid_done` (`std_msgs/Bool`) announce entry/exit of avoidance.

**Operational Constraints**
- Competition rule: **left (+) side** is the preferred avoidance direction.
- To avoid Simulink FSM desync, **controller switching** does **not** rely on distance-to-goal gating; waypoints are **time-streamed**. Internally, the module still decides when to publish `/avoid_done` (see §5.2).
- Obstacles at **path-ID boundaries** must be handled: merge **current + next ID** into a **single working path** **before** computing offsets (configurable **merge horizon** allows merging beyond one ID if needed).

---

## 1) Problem Definition → Hypotheses

**Initial problems**
1) **Boundary fragility**: If avoidance operates only within the current path ID, obstacles near an **ID transition** cause late or failed avoidance.
2) **Timing instability**: Using **distance-based reach conditions** to switch paths desynchronizes with the **Simulink FSM** event timing.

**Hypotheses**
- **(G1)** Treat **current + next IDs** as **one continuous curve** and compute offsets on that **full path** → removes boundary fragility.
- **(G2)** Remove reach gating for **controller switching**; **stream waypoints on a timer**, and use **/avoid_start** and **/avoid_done** as the **only synchronization hooks** → reduces FSM timing mismatch.

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
- Remove reach checks for controller switching; publish waypoints at a fixed timer frequency.
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

### 3.1 Full path composition and merge horizon
- Given `selected_id`, obtain `cur_pts` and at least the **next** ID’s `next_pts`.
- Concatenate `cur_pts + next_pts[1:]` to form `full_pts`.
- Parameter `merge_horizon_ids ≥ 1` allows merging additional IDs forward until either (a) clearance is achieved, or (b) horizon cap reached.
- Start: `start_pt = current_pose[0:2]` (from `/location`).
- End: by default the last point of the merged sequence; may be truncated after `/avoid_done` is decided.

### 3.2 Arc-length nearest and local frame
- Compute cumulative arc-length `s_i` over `full_pts` and use it for nearest-point search to avoid boundary chatter.
- `idx_obs = argmin_i || full_pts[i] − c_obs ||` with arc-length tie-break.

### 3.3 Obstacle segment model
- Local tangent near `idx_obs`: `t = normalize(full_pts[idx+1] − full_pts[idx−1])`.
- Segment endpoints: `a = c_obs − 0.5·obs_length·t`, `b = c_obs + 0.5·obs_length·t`.
- Collision threshold: `thresh = r_obs + car_half + safety_margin` (optionally `+ α·v`).

**Point-to-segment distance**
```
d(p,[a,b]) = || p − ( a + clip( ((p−a)·(b−a)) / ||b−a||^2 , 0, 1 ) · (b−a) ) ||
```

### 3.4 Per-point offset with left priority
- Tangent `t_i = normalize(p_{i+1} − p_{i−1})`.
- Left normal (right-handed coordinates): `n_i = R(+90°)·t_i = (-t_i.y, t_i.x)`.
- For `k = 1 … max_k`: candidate `p'_i(k) = p_i + k·δ·n_i`.
- Choose the smallest `k` such that `d(p'_i(k), [a,b]) ≥ thresh`.
- If no `k` clears within `max_k`, trigger **fail-safe** (see §6) or, if rules permit, **right-side** fallback by using `n_i = R(−90°)·t_i`.

### 3.5 Yaw continuity and smooth rejoin
- Raw yaw: `yaw_i = atan2(y_{i+1} − y_i, x_{i+1} − x_i)`; unwrap to maintain continuity in `(-π, π]`.
- Final 5–10 points: smooth with a short segment using either
  - **Quadratic Bezier** (P0=pre-join, P1=control toward nominal, P2=first nominal point), or
  - **Cubic Hermite** with tangents from adjacent yaws to ensure **C1** continuity.

**Implementation note**
- These steps map to `compute_offset_points()` and the timer-based publisher. See §5 for streaming and events.

---

## 4) Parameters and Tuning Log

| Parameter | Type | Default | Meaning / Guidance |
|---|---|---:|---|
| `car_half` | float | 0.20 | Half vehicle width (m). Set conservatively for clearance. |
| `obs_length` | float | 0.50 | Effective obstacle length along tangent (m). Too small → miss; too large → over-avoid. |
| `delta` | float | 0.05 | Offset step (m). Curvy segments: 0.03–0.07 recommended. |
| `max_k` | int | 20 | Ensures `delta*max_k` covers worst-case lateral shift. |
| `safety_margin` | float | 0.05 | Static extra buffer (m). |
| `alpha_speed_margin` | float | 0.10 | Speed-dependent term `+ α·v` (m per m/s). Set 0 to disable. |
| `timer_hz` | float | 5.0 | Waypoint streaming frequency (Hz). |
| `publish_nav_path` | bool | false | Enable `/avoid_path` (`nav_msgs/Path`). |
| `publish_legacy_xy` | bool | true | Enable legacy `/path_x`, `/path_y`. |
| `output_frame_id` | string | `base_link` | Output frame for waypoints. |
| `merge_horizon_ids` | int | 1 | Merge current + N following IDs. |
| `health_persist_N` | int | 3 | Frames required before entering avoidance (hysteresis). |
| `qos_depth` | int | 10 | QoS history depth for streams. |

**Termination signal**
- `/avoid_done` is emitted when either (a) the rejoin point arc-length is reached (internal counter over the avoidance sequence) or (b) the generator has streamed all avoidance points. This does not force controller switching; it only signals availability to resume base path.

**Logging recommendation**
- Log timestamp, obstacle pose, chosen `k`, minimum clearance, generated path length, and elapsed time from `/avoid_start` to `/avoid_done` → `logs/avoid_YYYYMMDD.csv`.

---

## 5) ROS2 Design (FSM Integration)

### 5.1 Handshake
- While avoidance is active, the **base path stream** (e.g., from `helper_path_sender`) should pause or be ignored.

**Integration options**
1) **Helper-pauses**: `helper_path_sender` subscribes to `/avoid_start` and halts base streaming; resumes on `/avoid_done`.
2) **Controller-prioritizes**: Controller consumes an **“avoidance-priority queue”** first; base stream is secondary.

### 5.2 QoS and timing
- Events (`/avoid_start`, `/avoid_done`): `reliable`, `transient_local` (late joiners see the last state), `depth = 1`.
- Streams (`/path_x`, `/path_y` or `/avoid_path`): match the base planner QoS for consistent timing; typical: `reliable`, `keep_last`, `depth = qos_depth`.
- Timer-based streaming at `timer_hz` with fixed period. Controller switching is **not** distance-gated.

---

## 6) Failure Cases and Mitigations

| Case | Symptom | Mitigation |
|---|---|---|
| Depth spikes / flicker | Spurious triggers | Hysteresis (`health_persist_N`); ignore retriggers during active avoidance. |
| Left-side clearance insufficient | `max_k` exhausted without clearance | Fail-safe: command **stop FSM**; retry, or if rules permit, enable **right-side fallback**. |
| Boundary chatter | Nearest-point flips at ID joins | Use **arc-length nearest** on the **merged path**; increase `merge_horizon_ids` if needed. |
| Harsh return | Steering jerks on rejoin | Smooth last 5–10 points with Bezier/Hermite; enforce yaw unwrapping. |
| Multi-obstacle corridor | Two obstacles bracket the path | Select worst clearance first; if still blocked after rejoin, generate a second avoidance with updated `full_pts` (post-merge). |

---

## 7) Topics Summary

| Topic | Dir | Type | Frame | Notes |
|---|---|---|---|---|
| `/obstacle_info` | Sub | `std_msgs/Float32MultiArray` | `base_link` | One or more `[x, y, r]` triplets per message. |
| `/location` | Sub | `geometry_msgs/Pose2D` | `output_frame_id` | `(x, y, theta)`; theta in radians. |
| `/path_x`, `/path_y` | Pub | `std_msgs/Float32` | `output_frame_id` | Legacy per-axis streaming of waypoints at `timer_hz`. |
| `/avoid_path` | Pub | `nav_msgs/Path` | `output_frame_id` | Standard packed path for downstream planners/controllers. |
| `/avoid_start`, `/avoid_done` | Pub | `std_msgs/Bool` | — | Event boundaries for FSM/Planner integration. |

---

## 8) Testing Recipe (minimal)

1. Provide `dqn_paths.json` per Appendix A and set `selected_id`.
2. Publish a static obstacle near an ID boundary:
   - `/obstacle_info`: `[x, y, r]` in `base_link`.
3. Start the node with `publish_nav_path=false`, `publish_legacy_xy=true`, `timer_hz=5.0`.
4. Observe `/avoid_start` → `/path_x`/`/path_y` streams → `/avoid_done`.
5. Switch to `publish_nav_path=true` and visualize `/avoid_path` in RViz; confirm `frame_id = output_frame_id`.
6. Repeat with two obstacles; verify worst-clearance selection and rejoin smoothness.

---

## 9) Known Limitations
- With `output_frame_id = base_link`, the path is expressed in a moving frame; this is acceptable for **short** avoidance segments but less suitable for long horizons. Use `map/odom` if long-horizon controllers consume the path.
- The segment obstacle model assumes locally convex obstacles aligned with the tangent. Highly irregular obstacles may require polygonal approximations.
- The left-priority rule can produce suboptimal trajectories on tight left boundaries. Consider enabling conditional right fallback under explicit rule allowances.

---

## Appendix A — `dqn_paths.json` schema (example)

```json
{
  "paths": {
    "1": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.2], [3.0, 0.0]],
    "2": [[3.0, 0.0], [4.0, 0.0], [5.0, -0.1]],
    "3": [[5.0, -0.1], [6.0, -0.1], [7.0, 0.0]]
  },
  "total": [1, 2, 3]
}
```
- `paths[id]` is an array of `[x, y]` waypoints in meters.
- `total` lists the global order of path IDs. The module will merge current + `merge_horizon_ids` forward IDs to form `full_pts`.

---

## Appendix B — Pseudocode (high level)

```
load dqn_paths.json
set params (delta, max_k, obs_length, car_half, safety_margin, alpha_speed_margin, merge_horizon_ids, output_frame_id)

on obstacle_info and location update:
  full_pts = merge(current_id, horizon=merge_horizon_ids)
  idx_obs  = arc_length_nearest(full_pts, obstacle_center)
  [a,b]    = segment_from_tangent(obstacle_center, full_pts, idx_obs, obs_length)
  for each point p_i in full_pts:
    t_i = normalize(p_{i+1} - p_{i-1})
    n_i = left_normal(t_i)
    find smallest k in [1..max_k] s.t. d(p_i + k*delta*n_i, [a,b]) >= thresh
    p'_i = p_i + k*delta*n_i (or fallback)
  smooth tail with Bezier/Hermite; unwrap yaw
  publish /avoid_start once
  stream waypoints (legacy XY or nav_msgs/Path) at timer_hz
  when end-of-sequence reached -> publish /avoid_done
```
