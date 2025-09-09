# Planning Module

We stabilize full-lap driving on a **static track** with **offline preprocessing** (RRT path bank + DQN visiting order) and a **timer-based runtime stream** (documented elsewhere). This README focuses on *what* is planned (RRT/DQN/Local Avoidance) and *why*, not on the data-sender implementation.

---

## 0) Purpose & Constraints

**Goals**
- Robust lap completion using offline segments plus timing-based streaming.
- Minimize ROS 2 ↔ Simulink FSM desync by **separating trajectory streaming** from **behavior events**.

**Key ideas**
1. Run RRT many times per segment to build a **bank of smooth, collision-free waypoints**.
2. Use a **NumPy-only DQN** to compute a **fast, low-memory visiting order**; serialize as `dqn_paths.json`.
3. Keep runtime switching **time/index-based**, not distance-based (details in the streaming doc).

**Real-world constraints**
- Pose latency/noise made distance-based switching jittery → **reach conditions removed**.
- Obstacles at **path-ID boundaries** → Local Avoidance **merges current+next ID** before offsetting to keep continuity.

**Frames**
- Offline artifacts are in `map` (meters). For short local avoidance we may operate in `base_link`; consumers transform via TF (`map → odom → base_link`).

---

## 1) Problem → Hypothesis → Validation

1) **Unstable distance-based switching**  
   *Hypothesis*: time-based streaming + separate events → *Validated*: jitter removed, runs repeatable.

2) **Boundary collisions/discontinuities**  
   *Hypothesis*: during avoidance, compute offsets on a **merged (current+next) path** → *Validated*: single continuous curve to rejoin.

3) **Inefficient visit order**  
   *Hypothesis*: lightweight DQN with BFS fallback → *Validated*: immediate, low-memory plan (`dqn_paths.json`).

---

## 2) Components (planning only)

| Component | Description |
|---|---|
| `rrt/` | Offline generation of per-segment candidates (dozens–thousands of trials), scoring by length/curvature/continuity/clearance, smoothing/resampling; keep champions as `waypoints_*.json`. |
| `dqn/` | **DQNPathPlanner (NumPy-only)** computes the visit sequence of stops/intermediate nodes with explicit loop/revisit penalties; outputs `dqn_paths.json`. |
| `local_planners/obstacle_avoidance/` | Local Avoidance (summary below): generates temporary offset paths and events to bypass forward obstacles and rejoin smoothly. |

**Data contracts (files)**
- `waypoints_*.json`: `[x, y]` list in meters, `frame_id=map`, uniform spacing `spacing_m`.
- `dqn_paths.json`: per-leg node sequences and global `total` order (optional `stops`, `path_mode_map`).

---

## 3) Local Avoidance (summary)

**Objective**  
When an obstacle lies on the nominal route, generate a **minimum-offset** lateral path that **stays within lane bounds** and **rejoins** smoothly, with timing stable under an FSM.

**Frames & I/O (summary)**  
- Default compute/output in **`base_link`** for short-horizon behavior; optional `output_frame_id = map|odom`.  
- Inputs (typical): `/obstacle_info` with one or more `[x, y, r]` triplets (m, base_link), current pose; nominal path IDs and points.  
- Outputs: a short-horizon avoidance path (`nav_msgs/Path` **or** legacy `/path_x`, `/path_y`) and events `/avoid_start`, `/avoid_done`.

**Core algorithm (concise)**  
1) **Full-path merge**: concatenate **current + next** ID into a single working curve.  
2) **Left-priority offset**: per-point normal `n_i`, increase by δ (e.g., 0.05 m) until the point-to-segment distance to the obstacle exceeds the threshold (`r_obs + car_half + margin`).  
3) **C1 rejoin**: smooth the last 5–10 points (Bezier/Hermite) and unwrap yaw for continuity.  
4) **Timer-based emission**: waypoints are streamed on a fixed period; controller switching is **not** distance-gated.  
5) **Hysteresis**: require persistence (N frames) before entering avoidance; ignore retriggers during active avoidance.

**Typical params**  
`delta=0.05 m`, `max_k=20`, `safety_margin=0.05 m`, `merge_horizon_ids=1`, `timer_hz=5.0 Hz`.

---

## 4) DQN (why this tiny model)

- Chosen for **simplicity, speed, and immediate application** on a static loop.  
- Rewards combine **goal bonus (+100)**, **loop penalty (−1000)** via SCC, **step cost (−1)**, **branch bonus (+5)**, and **revisit penalty (−10×visits)** to favor short, decisive, loop-free tours.  
- Model: 2-layer MLP (hidden=64, ReLU; linear output), **MSE** loss with target \(y=r+\gamma \max Q(s',\cdot)\); ε-greedy (0.5→0.0).  
- Determinism guarantee via **BFS fallback** when exploration stalls.

**Example `dqn_paths.json`**

~~~json
{
  "paths": {
    "p1": [0, 1, 2, 3, 4],
    "p2": [4, 5, 6, 7],
    "p3": [7, 8, 9, 39, 40, 41]
  },
  "total": ["p1", "p2", "p3"],
  "stops": ["p1", "p3"],
  "path_mode_map": { "p1": 0, "p2": 1, "p3": 2 }
}
~~~

---

## 5) RRT (how we judge candidates)

- **Run many trials** per pair (50–2000+); keep top **k %** by a **normalized weighted score**: length, mean/peak curvature, continuity penalties, clearance, and a jerk proxy.  
- Typical limits: \( \kappa_{\max} \le 0.30\,\mathrm{m}^{-1} \), `clearance_min ≥ 0.30 m`.  
- Uniform resampling (e.g., Δs=0.10 m), yaw unwrap/low-pass, curvature cleanup (SG/spline); validate loop connectivity.

---

## 6) Evaluation (planning-wide)

- **RRT quality**: success rate, mean/95th length, \( \kappa_{\max} \) & clearance distributions, generation time.  
- **DQN plan**: total length, #revisits, #loop entries, fork indecision rate, **compute latency** (ms), **memory** (MB).  
- **End-to-end**: bag replay for **timing reproducibility**; on-car **lap time**, **off-track rate**, **event timeliness**.
