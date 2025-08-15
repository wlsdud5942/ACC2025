# Planning Module — Process-Centric README (Revised)

This module stabilizes end-to-end driving on a **static track** by combining **offline preprocessing** (path generation) with **runtime streaming**. We prebuild high-quality local paths per segment using **RRT**, optimize the **visit order** of stops/waypoints using a lightweight **DQN** planner, and then stream waypoints at fixed timing via a helper node **without distance-based reach checks**. The FSM (Simulink) handles stop/start independently through events.

---

## 0) Purpose and Design Constraints

**Goals**
- Make full-lap driving robust on a static track using offline paths plus online, timing-based streaming.
- Minimize synchronization errors between ROS2 and the Simulink FSM by separating “trajectory streaming” from “behavior events”.

**Key ideas**
1. Run RRT many times per segment to build a **bank of smooth, collision‑free waypoints**.
2. Use a NumPy DQN to compute the **optimal visiting order** of stops/intermediate nodes and save it as `dqn_paths.json`.
3. At runtime, **stream** waypoints on a timer; the FSM **does not** rely on geometric reach checks.

**Real‑world constraints**
- Simulink FSM vs ROS2 timing can drift; distance‑based switching caused jitter → we **removed reach conditions** from the streaming node.
- Obstacles or transitions at **path‑ID boundaries** can cause discontinuities; avoidance merges **current + next ID** before computing offsets.

**Frames & coordinates**
- Waypoints are expressed in `output_frame_id` (default `map` for static tracks; `base_link` allowed for short local segments).
- If `map` is used, controllers that operate in `base_link` should transform at consume time using TF (`map → odom → base_link`).

---

## 1) Problem → Hypothesis → Validation

1) **Unstable distance‑based switching**
- **Problem**: Pose delay/noise caused segment switches to oscillate around thresholds, confusing the FSM.
- **Hypothesis**: Stream `/path_x`, `/path_y` on a timer only; handle FSM events via separate topics.
- **Validation**: Implemented a sliding‑window stream; switching became repeatable across runs.

2) **Collisions/discontinuities at boundaries**
- **Problem**: Obstacles and path transitions co‑located at an ID boundary caused sharp turns.
- **Hypothesis**: During avoidance, merge **current + next ID** into a single curve and compute the lateral‑offset path there.
- **Validation**: Obstacle_Avoidance full‑path merge + offset yields a single continuous curve from avoidance to rejoin.

3) **Visit‑order optimization**
- **Problem**: Fixed stop order produced inefficient tours.
- **Hypothesis**: A simple DQN with loop/branch penalties can learn a practical order quickly.
- **Validation**: Custom NumPy DQN with BFS fallback robustly generates `dqn_paths.json`.

---

## 2) Component Summary

| Component | Description |
|---|---|
| `rrt/` | Offline generation of per‑segment waypoint sets on a static map; repeated runs select smooth, short, collision‑free paths. |
| `dqn/` | `DQNPathPlanner` that optimizes the visit order of stops/intermediate nodes. |
| `waypoints_*.json` | Per‑segment arrays of `[x, y]` in meters; consistent sampling interval and heading direction. |
| `dqn_paths.json` | DQN result: `{ "p1": [...], "p2": [...], "total": [...] }` where `total` is the global node sequence. |
| `helper_path_sender` | Streams waypoints **without reach checks**; emits `/path_mode`, `/stop`, `/pickup_dropoff` as separate events. |

**Data contracts**
- `waypoints_*.json`: list of `[x, y]` points in map/track plane coordinates (meters).
- `dqn_paths.json`: segment node sequences `p1..pk` and the overall visiting order `total`.

---

## 3) Core Node: `helper_path_sender` (Why streaming?)

**What it does**
- Loads `dqn_paths.json` and sequentially loads each `waypoints_*.json` in the `total` order.
- Publishes waypoints at a **fixed timer frequency** using a **sliding window of W points** (default `W=4`).
- Exposes both **legacy** per‑axis streams (`/path_x`, `/path_y`) and **standard** `/planned_path` (`nav_msgs/Path`).

**Why remove reach checks**
- Geometric reach checks depend on pose latency/noise and frequently desynchronize with the FSM.
- Streaming is purely **index/time‑based**; **events** remain independent.

**Termination & transitions**
- Segment transition occurs when the **index** reaches the end of the segment (or when an **end‑of‑segment** timer expires). No distance thresholds.
- `/planned_path` publishes the current window with `header.frame_id = output_frame_id`.

**Events (published by this node)**

| Event | Meaning | When it’s published |
|---|---|---|
| `/stop` | Vehicle stop | Upon entering a segment listed in `stops.json` (with cooldown). |
| `/pickup_dropoff` | Pickup or dropoff | According to the scenario definition (e.g., first/last stop). |
| `/path_mode` | 0: straight, 1: curve, 2: start/finish | Derived from `path_mode_map` for each segment. |

---

## 4) ROS2 Topics

| Topic | Dir | Type | Frame | Description |
|---|---|---|---|---|
| `/location` | Sub | `geometry_msgs/Pose2D` | consumer‑specific | Current pose (x, y, theta). |
| `/dqn_done` | Sub | `std_msgs/Bool` | — | Trigger to (re)start streaming after DQN planning. |
| `/path_x`, `/path_y` | Pub | `std_msgs/Float32` | `output_frame_id` | Legacy per‑axis streaming of W points (time‑based). |
| `/planned_path` | Pub | `nav_msgs/Path` | `output_frame_id` | Standard sliding‑window path for controllers. |
| `/path_mode` | Pub | `std_msgs/Int32` | — | Segment type: 0=straight, 1=curve, 2=start/finish. |
| `/stop` | Pub | `std_msgs/Int32` | — | FSM stop command. |
| `/pickup_dropoff` | Pub | `std_msgs/Int32` | — | FSM pickup/dropoff command. |

**QoS recommendations**
- Paths & events: `reliable`, `keep_last(10)`; use `transient_local` for events if late‑join recovery is needed.
- Timer period must be stable; prefer steady timers and single‑threaded executor for determinism.

---

## 5) Parameters

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `output_frame_id` | string | `map` | Frame for waypoint publishing. Use `base_link` only for short local segments. |
| `timer_hz` | float | 2.0 | Streaming frequency (Hz). |
| `window_size` | int | 4 | Sliding window W. |
| `publish_nav_path` | bool | true | Publish `/planned_path` (`nav_msgs/Path`). |
| `publish_legacy_xy` | bool | true | Publish `/path_x`, `/path_y`. |
| `use_index_end_trigger` | bool | true | End segment by index (not distance). |
| `segment_timeout_s` | float | 30.0 | Optional max time per segment (backup termination). |
| `cooldown_stop_frames` | int | 5 | Prevent duplicate `/stop` events at segment entry. |
| `path_mode_map` | dict | — | Segment→mode mapping. |

---

## 6) Tuning Notes
- **Streaming frequency (`timer_hz`)**: balance controller bandwidth and CPU/network load.
- **Window size (`W`)**: `4` works well for pure‑pursuit style controllers; increase for MPC with longer preview.
- **Interpolation**: if your controller requires `C1` continuity, spline/Bézier post‑smoothing can be applied offline.
- **Stops**: keep `stops.json` minimal and unambiguous; avoid back‑to‑back stop segments unless intentional.

---

## 7) Testing & RViz Recipe
1. Load `dqn_paths.json` and the referenced `waypoints_*.json` files.
2. Run `helper_path_sender` with `publish_nav_path=true` and visualize `/planned_path` in RViz (Path display). Confirm `frame_id`.
3. Verify event topics (`/stop`, `/pickup_dropoff`, `/path_mode`) fire at the correct segment entries.
4. Record a rosbag2 and replay; check that the same timing/sequence reproduces.

---

## 8) Failure Cases & Fixes

| Issue | Symptom | Fix |
|---|---|---|
| RRT discontinuities | Sudden curvature change | Unify sampling interval; linear interpolation; heading correction for continuity. |
| Collisions at boundaries | Sharp turns / miss at joins | During avoidance, always **merge two IDs** before offset computation. |
| Duplicate stops | Multiple `/stop` per entry | Apply `cooldown_stop_frames`; de‑bounce per segment index. |
| FSM vs ROS2 desync | Timing mismatch | Keep streaming and events separated; **no distance gating**. |

---

