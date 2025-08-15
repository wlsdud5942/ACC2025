# HelperPathSender — DQN-Based Path Distribution & Stop Management (Revised, Process-Centric)

`HelperPathSender` publishes controller-friendly waypoints **on a timer** from a precomputed visit order (DQN). It performs **segment preloading**, **index-based** transitions (no distance gating), and emits **explicit events** for stops/pickup/dropoff and path mode. It supports both **standard** (`/planned_path` as `nav_msgs/Path`) and **legacy** (`/path_x`, `/path_y`) outputs.

---

## Overview
The node:
- Loads the **visit order** and per-segment waypoints.
- Streams a **sliding window** of waypoints on a fixed timer.
- **Preloads** the next segment before transition.
- Emits **/path_mode**, **/pickup_dropoff**, and **/stop** events according to `stops` and `path_mode_map`.
- Optionally publishes both **standard** and **legacy** path outputs.

---

## ROS 2 Topics

| Topic | Type | Dir | Frame | Notes |
|---|---|---|---|---|
| `/location` | `geometry_msgs/Pose2D` | Sub | consumer-specific | Used for diagnostics/ROI only (no distance gating). |
| `/dqn_done` | `std_msgs/Bool` | Sub | — | Start signal once DQN writes `dqn_paths.json`. |
| `/planned_path` | `nav_msgs/Path` | Pub | `output_frame_id` | Sliding window of waypoints (standard). |
| `/path_x`, `/path_y` | `std_msgs/Float32` | Pub | `output_frame_id` | Legacy per-axis streaming (windowed). |
| `/path_mode` | `std_msgs/Int32` | Pub | — | 0=straight, 1=curve, 2=start/finish. |
| `/pickup_dropoff` | `std_msgs/Int32` | Pub | — | 1=pickup, 2=dropoff. |
| `/stop` | `std_msgs/Int32` | Pub | — | 1=request stop, 0=clear. (Optional: subscribe to `/stop_ack` for FSM acks.) |

**QoS**: Paths/events → `reliable`, `keep_last(10)`. Use `transient_local` for events if late-join recovery is needed. Timers should be **steady** for determinism.

---

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `output_frame_id` | string | `map` | Frame for outputs (`map` recommended for static tracks). |
| `timer_hz` | float | 2.0 | Streaming frequency. |
| `window_size` | int | 4 | Number of points per window. |
| `step_points` | int | 1 | Index increment per tick (advance rate). |
| `preload_ahead_pts` | int | 8 | Preload next segment when current index within this many points of the end. |
| `publish_nav_path` | bool | true | Enable `/planned_path`. |
| `publish_legacy_xy` | bool | true | Enable `/path_x`, `/path_y`. |
| `events_transient_local` | bool | true | Make events survive restarts. |
| `cooldown_stop_frames` | int | 5 | Debounce window to suppress duplicate `/stop`. |
| `stop_wait_ticks` | int | 10 | Non-blocking wait ticks after publishing `/stop=1`. |
| `paths_file` | string | `dqn_paths.json` | DQN output file. |
| `waypoints_dir` | string | `planning/waypoints/` | Directory for `waypoints_*.json`. |
| `stops_file` | string | `planning/config/stops.json` | List of stop segment IDs or node IDs. |

---

## Node Logic

1. **Wait** for `/dqn_done=True` (or file presence) and **validate** `dqn_paths.json` and waypoints.
2. **Initialize** indices: `seg_idx=0`, `pt_idx=0`; push first window.
3. **Publish** sliding window on a fixed timer (no distance gating).
4. **Preload** next segment when `pt_idx` near end (`preload_ahead_pts`).
5. On **segment entry**:
   - Emit `/path_mode` from `path_mode_map`.
   - If segment is in `stops`, emit `/pickup_dropoff` (1=pickup first, 2=dropoff last) and `/stop=1`.
   - Start **tick-based wait** (`stop_wait_ticks`) → then `/stop=0` and resume streaming.
6. **Advance** `pt_idx += step_points`; when end reached or timeout, **switch** to next segment (index-based).

---

## Practical Issues & Fixes

| Issue | Root Cause | Fix |
|---|---|---|
| First waypoints skipped at transition | Publish order vs index update race | **Preload buffer** + publish after index reset. |
| `sleep()` froze node at stops | Blocking timer callback | **Tick-based wait**; keep executor spinning. |
| `stops.json` nested list | Unclean config merges | **Auto-flatten** on load + schema check. |
| Duplicate IDs repeated segments | Bad `dqn_paths.json` content | Validate uniqueness per leg; warn on repeats. |
| `/stop` not received | QoS mismatch | Use **reliable**, optional `transient_local`; add cooldown. |

---

