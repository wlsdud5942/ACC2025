# Data Helper

We built **Path Sender** (with **data_helper**) so the Simulink-generated controller receives a **steady, C¹-smooth, equally-spaced** sequence—more stable in ROS 2 than pushing raw segments from MATLAB.

---

## 0) Roles

- **data_helper**
  - Validate and normalize `waypoints_*.json` (schema, frame).
  - **Re-sample by arc-length** (Δs), **yaw unwrap**, **C¹ smoothing** (short splines).
  - Boundary safety: pre-check **current + next segment** connectivity for clean joins.

- **Path Sender**
  - Load `dqn_paths.json` (`total` visit order) and per-segment `waypoints_*.json`.
  - Publish a **sliding window** at a **fixed timer rate** (no distance gating).
  - **Index-based transitions** with **preload**; emit events: `/path_mode`, `/pickup_dropoff`, `/stop`.

---

## 1) Inputs & Schemas

**Waypoints (normalized by data_helper)**
```json
{
  "frame_id": "map",
  "points": [[0.10, 0.20], [0.60, 0.30], [1.10, 0.40]],
  "spacing_m": 0.10
}
```

**DQN output (consumed by Path Sender)**
```json
{
  "paths": { "p1": [0,1,2,3], "p2": [3,4,5] },
  "total": ["p1","p2"],
  "stops": ["p1"],
  "path_mode_map": {"p1": 0, "p2": 1}
}
```

---

## 2) Node Behavior (stable streaming, no distance checks)

- **Timer** at `timer_hz` controls publishing cadence (guarded by `RateGuard`).
- **Window** of `W` points → `/planned_path` (standard). Optionally `/path_x`,`/path_y` (legacy).
- On **segment entry**:
  - Publish `/path_mode` from `path_mode_map`.
  - If in `stops`, emit `/stop=1` once (`EventOnce`), wait **non-blocking** `stop_wait_ticks`, then `/stop=0`.
  - Emit `/pickup_dropoff` if configured.
- **Preload** next segment when near the end; switch by **index** only.
- **Avoidance gate**: if `/avoid_start` arrives, switch gate to avoidance stream; revert on `/avoid_done`.

**Pseudocode**
```python
if rate_guard.tick():
    win = window(curr_seg, idx, W)
    pub_planned_path(win)
    if idx == 0:
        pub_path_mode(mode_map[curr_seg])
        if is_stop(curr_seg) and stop_once.ok(True):
            pub_stop(1); tick_wait(stop_wait_ticks); pub_stop(0)
        pub_pickup_dropoff_if_any(curr_seg)
    idx += step_points
    if near_end(curr_seg, idx): preload(next_seg)
    if end_of_segment(curr_seg, idx): curr_seg = next_seg; idx = 0
```

---

## 3) ROS 2 Topics (this package)

| Topic | Dir | Type | Frame | QoS |
|---|---|---|---|---|
| `/planned_path` | Pub | `nav_msgs/Path` | `output_frame_id` (default `map`) | **reliable, keep_last(10)** |
| `/path_x`, `/path_y` | Pub | `std_msgs/Float32` | `output_frame_id` | **reliable, keep_last(10)** |
| `/path_mode` | Pub | `std_msgs/Int32` | — | **reliable, keep_last(10), transient_local** |
| `/pickup_dropoff` | Pub | `std_msgs/Int32` | — | **reliable, keep_last(10), transient_local** |
| `/stop` | Pub | `std_msgs/Int32` | — | **reliable, keep_last(10), transient_local** |
| `/dqn_done` | Sub | `std_msgs/Bool` | — | **reliable, keep_last(1)** |
| `/location` | Sub | `geometry_msgs/Pose2D` | consumer-specific | reliable (diagnostics only) |
| `/avoid_start`, `/avoid_done` | Sub | `std_msgs/Bool` | — | reliable (gate switching) |

---

## 4) Parameters (what we actually ship)

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `output_frame_id` | string | `map` | Static track = global frame. |
| `timer_hz` | float | 2.0 | Streaming frequency. |
| `window_size` | int | 4 | Points per sliding window. |
| `step_points` | int | 1 | Index increment per tick. |
| `preload_ahead_pts` | int | 8 | Preload threshold. |
| `publish_nav_path` | bool | true | Enable `/planned_path`. |
| `publish_legacy_xy` | bool | true | Enable `/path_x`,`/path_y`. |
| `events_transient_local` | bool | true | Persistent events. |
| `cooldown_stop_frames` | int | 5 | Debounce for repeated segments. |
| `stop_wait_ticks` | int | 10 | Non-blocking wait ticks. |
| `paths_file` | string | `dqn_paths.json` | DQN result. |
| `waypoints_dir` | string | `planning/waypoints/` | Champion segments. |
| `stops_file` | string | `planning/config/stops.json` | Stops definition. |

---

## 5) Data Helper — Normalization Steps

- Enforce **uniform spacing** (Δs) along each segment.
- **Yaw unwrap** and short **spline smoothing** for C¹ continuity (especially last 3–5 points).
- Detect and fix **back-tracking** or sharp curvature spikes; ensure monotone arc-length.
- Verify **frame_id** and `spacing_m` consistency; repair if missing.
- Optionally pre-merge **current + next** at boundaries to ensure a soft join.

---

## 6) Testing Checklist

- `/planned_path` continuity: no heading jumps at joins.
- Exactly-once events at segment entry (no duplicates).
- QoS matches FSM (reliable, depth 10; events with `transient_local`).
- Leak monitor: **0% drops**, AoI and jitter inside control deadlines.

---

## 7) Why This Exists (project rationale)

Our controller expects a **well-ordered, evenly-spaced, continuous function sequence**. Rather than pushing raw MATLAB outputs, the **Path Sender + data_helper** on ROS 2 deliver a **deterministic stream** with explicit events and measurable timing, which proved more stable on the real vehicle.

---
```
