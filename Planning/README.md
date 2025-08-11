# Planning Module — Process-Centric README

This module stabilizes end-to-end driving on a **static track** by combining **offline preprocessing** (path generation) with **runtime streaming**. We prebuild high-quality local paths per segment using **RRT**, optimize the **visit order** of stops/waypoints using a lightweight **DQN** planner, and then stream waypoints at fixed timing via `helper_path_sender` without distance-based reach checks. The FSM (Simulink) handles stop/start independently through events.

---

## 0) Purpose and Design Constraints

Goals
- Make full-lap driving robust on a static track using offline paths plus online, timing-based streaming.
- Minimize synchronization errors between ROS2 and the Simulink FSM by separating “trajectory streaming” from “behavior events”.

Key ideas
1. Run RRT thousands of times per segment to build a **bank of smooth, collision-free waypoints**.
2. Use a NumPy DQN to compute the **optimal visiting order** of stops/intermediate nodes and save it as `dqn_paths.json`.
3. At runtime, **stream** waypoints on a timer; the FSM **does not** rely on geometric reach checks.

Real-world constraints
- Simulink FSM vs. ROS2 timing can drift; distance-based switching caused jitter. We therefore **removed reach conditions** from the streaming node.
- Obstacles or transitions at **path-ID boundaries** can cause discontinuities; avoidance is designed to **merge current + next ID** before computing offsets.

---

## 1) Problem → Hypothesis → Validation

1) Unstable distance-based switching  
- Problem: Pose delay/noise caused segment switches to oscillate around thresholds, confusing the FSM.  
- Hypothesis: Stream `/path_x`, `/path_y` on a timer only; handle FSM events via separate topics.  
- Validation: Implemented a 4-point sliding window stream in `helper_path_sender`; switching became repeatable.

2) Collisions/discontinuities at boundaries  
- Problem: Obstacles and path transitions co-located at an ID boundary caused sharp turns.  
- Hypothesis: During avoidance, merge **current + next ID** into a single curve and compute the lateral-offset path there.  
- Validation: In the Obstacle_Avoidance module, full-path merge + offset yields a single continuous curve from avoidance to rejoin.

3) Visit-order optimization  
- Problem: Fixed stop order produced inefficient tours.  
- Hypothesis: A simple DQN with loop/branch penalties can learn a practical order quickly.  
- Validation: Custom NumPy DQN with BFS fallback robustly generates `dqn_paths.json`.

---

## 2) Component Summary

Component | Description
--- | ---
`rrt/` | Offline generation of per-segment waypoint sets on a static map; repeated runs select smooth, short, collision-free paths.
`dqn/` | ROS2 node `DQNPathPlanner` that optimizes the visit order of stops/intermediate nodes.
`waypoints_*.json` | Per-segment arrays of `[x, y]` in meters; consistent sampling interval and heading direction.
`dqn_paths.json` | DQN result: `{"p1":[...], "p2":[...], "total":[...]}` where `total` is the global node sequence.
`helper_path_sender` | Streams waypoints **without reach checks**; emits `/path_mode`, `/stop`, `/pickup_dropoff` as separate events.

Data contracts
- `waypoints_*.json`: list of `[x, y]` points in map/track plane coordinates (meters).
- `dqn_paths.json`: segment node sequences `p1..pk` and the overall visiting order `total`.

---

## 3) Core Node: `helper_path_sender` (Why streaming?)

What it does
- Loads `dqn_paths.json` and, in that order, loads each `waypoints_*.json`.
- Publishes `/path_x`, `/path_y` at a **fixed timer period** (e.g., 0.55 s) in a **sliding window of 4 points**.

Why remove reach checks
- Geometric reach checks depend on pose latency/noise and frequently desynchronize with the FSM.  
- By making streaming purely **time-based**, and emitting **events** separately, FSM transitions are deterministic.

Code notes
- Some versions still include a distance-based `end_trigger()`. For production, either switch to **index-based** transitions or keep `end_trigger()` as a backup only.

Event separation

Event | Meaning | When it’s published
--- | --- | ---
`/stop` | Vehicle stop | Upon entering a segment listed in `stops.json` (first = pickup, last = dropoff, as defined)
`/pickup_dropoff` | Pickup or dropoff | According to the scenario definition (e.g., first/last stop)
`/path_mode` | 0: straight, 1: curve, 2: start/finish | Derived from `path_mode_map` for each segment

---

## 4) ROS2 Topics

Topic | Dir | Type | Description
--- | --- | --- | ---
`/location` | Sub | `geometry_msgs/Point` | Current pose; some implementations store yaw in `z`
`/dqn_done` | Sub | `std_msgs/Bool` | Trigger to (re)start streaming after DQN planning
`/path_x`, `/path_y` | Pub | `std_msgs/Float32` | Waypoint streaming (window of 4 points), time-based
`/path_mode` | Pub | `std_msgs/Int32` | Segment type: 0 straight, 1 curve, 2 start/finish
`/stop` | Pub | `std_msgs/Int32` | FSM stop command
`/pickup_dropoff` | Pub | `std_msgs/Int32` | FSM pickup/dropoff command

QoS recommendation
- Paths and events: `reliable`, `keep_last(N)`; use `transient_local` if you need late-join recovery after node restarts.

---

## 5) Technical Issues & Fixes

Issue | Fix
--- | ---
RRT discontinuities | Unify sampling interval; linear interpolation between samples; heading correction to ensure continuity.
Collisions at boundaries | During avoidance, always **merge two IDs** before offset computation.
Duplicate stops | Publish stop events once per segment index and apply a short cooldown.
FSM vs ROS2 desync | Keep **streaming** and **events** fully separated; do not gate streaming on distance.

---

## 6) Data & File Layout (reference)

- `planning/rrt/` — scripts to generate per-segment waypoints and select best candidates.  
- `planning/dqn/` — `DQNPathPlanner` node and utilities; outputs `dqn_paths.json`.  
- `planning/waypoints/` — files named like `waypoints_<segment>.json` with `[x, y]` arrays in meters.  
- `planning/config/` — `path_mode_map`, `stops.json`, any thresholds.  
- `planning/nodes/helper_path_sender.py` — streaming node used at runtime.

`dqn_paths.json` schema (verbal)
- Keys `p1..pk`: each is a list of node indices for a subpath.  
- Key `total`: concatenated unique node sequence covering the full tour.


---

## 7) Tuning Notes

- Streaming period: choose to balance controller bandwidth and network load (e.g., 0.55 s in current setup).  
- Window size: 4 points worked well with our controller; adjust with your lookahead logic.  
- Interpolation: consider spline/Bézier post-smoothing if your controller requires higher curvature continuity.  
- `stops.json`: keep minimal and unambiguous; avoid consecutive segments that both request a stop unless intentional.

---



