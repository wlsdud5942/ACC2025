# `rrt/` — Offline RRT Path Set Generation (Process-Centric README)

This module builds a **high-quality set of offline paths** for a **static track**. Instead of computing paths online, we repeatedly run **RRT** (dozens to thousands of trials per node pair X→Y), evaluate each candidate by **length / curvature / continuity**, smooth and resample it, and persist only the **best** as `waypoints_XtoY.json`. These curated segments are later stitched (via `dqn_paths.json`) and streamed at runtime by the planning stack.

---

## 0) Purpose

On a static course, **precomputing** robust segments is safer than relying on on-the-fly search. The objective is to:
- Generate multiple candidate paths for each directed pair **X→Y** between meaningful track nodes (splits, corners, stops, pickup/dropoff).
- Score and filter candidates using geometric and dynamic criteria.
- **Normalize** spacing and heading, and smooth curvature.
- Save a **single champion** (or a small Pareto set) per pair to `waypoints_XtoY.json`.

---

## 1) Process Timeline (Experiment Log Style)

V0 — Single-shot RRT (baseline; did not meet quality)
- Attempt: one run per pair.
- Observations: high variance; occasional discontinuities and excessive curvature; micro-vibrations on the vehicle due to heading jitter.
- Lesson: RRT is stochastic; single samples are unreliable for production.

V1 — Multi-run + filtering (large improvement)
- Method: run RRT **N times** per X→Y (N = 50–2000 depending on pair difficulty).
- Metrics: total length L, average curvature κ̄, curvature peak κ_max, continuity penalties (heading jumps).
- Selection: keep the top **k%** (e.g., 10–20%) by a composite score; discard the rest.

V2 — Smoothing + heading normalization (stability)
- Resampling: uniform spacing (e.g., 0.10 m) to normalize controller lookahead behavior.
- Heading continuity: unwrap yaw to remove 2π discontinuities; low-pass filter yaw to reduce micro-jitter.
- Curvature cleanup: remove spike outliers (e.g., Savitzky–Golay on coordinates or curvature), then re-resample.

V3 — Connectivity & loop validation (track completeness)
- On split-join regions, verify that chosen pairs allow **closed-loop traversal** without kinematic dead-ends.
- Introduce a “return-to-nominal” test: successful traverse from X→Y and back through the designed network without violating curvature limits.

Result: A repeatable pipeline that yields smooth, controller-friendly, and mutually compatible segments.

---

## 2) How to Use (Operator Workflow)

1) Define **semantic nodes** on the map (split, corner entry/exit, stop, pickup/dropoff, etc.).  
   Store node indices and (x, y) anchors in a small config file.

2) For each **directed pair** X→Y:
   - Run RRT **N** times; collect successful paths (polyline sequences).
   - Evaluate, filter, and smooth; enforce uniform spacing (e.g., Δs = 0.10 m).
   - Persist champion to `waypoints_XtoY.json`.

3) Naming convention  
   `waypoints_13to21.json` → path from node 13 to node 21.

4) Storage format (JSON; meters in the map plane)  
   Example content (shown as plain text):
     [[0.10, 0.20], [0.60, 0.30], [1.10, 0.40], ...]

5) Recommended RRT parameters (typical starting points)
- step_size: 0.20 m
- goal_sample_rate: 0.10–0.20
- max_iter: ≥ 1000 (increase for narrow passages)
- path_resolution: 0.10 m (post-smoothing resample target)
- collision_check_fn: map-aware clearance function (see §7)

---

## 3) Algorithm Details (What We Actually Do)

Node Pair Expansion (per X→Y)
- Seeding: set RNG seeds (multiple seeds per pair for coverage) and bounds for sampling.
- RRT Loop:
  - Sample a point; with probability goal_sample_rate, sample around Y to bias progress.
  - Extend the nearest tree node toward the sample by step_size while collision-checking.
  - If within goal tolerance of Y, backtrack the tree to form a path.
- Path Formatting:
  - Densify polyline to target spacing (linear interpolation at Δs).
  - Optional shortcutting: attempt to replace small subsections with straight segments when collision-free.
  - Smoothing: either cubic-spline through anchor subsets, Bézier fragment stitching, or coordinate-wise Savitzky–Golay (window 9–21, poly 2–3).
- Curvature Computation:
  - For discrete points P_i = (x_i, y_i) sampled at equal arc length s, estimate curvature κ_i by finite differences of yaw or by discrete Frenet:
      κ_i ≈ |(x' y'' − y' x'')| / ( (x'^2 + y'^2)^(3/2) ), using centered differences.
  - Compute metrics: average κ̄, peak κ_max, and curvature jerk proxy Δκ/Δs.
- Heading Continuity:
  - Compute yaw_i = atan2(Δy, Δx), then unwrap; low-pass filter to remove flicker.
- Scoring (see §4) and selection.

---

## 4) Quality Criteria and Scoring

We score each candidate by a weighted composite:

S = w_L · norm(L) + w_k̄ · norm(κ̄) + w_kmax · norm(κ_max) + w_cont · P_cont + w_clear · P_clearance + w_smooth · P_smooth

Where:
- L: total path length (shorter is generally better, but do not penalize gentle detours that reduce κ_max).
- κ̄: average curvature; κ_max: worst-case curvature (bounded by vehicle limits).
- P_cont: continuity penalty (heading jumps, position discontinuities).
- P_clearance: penalty for near-obstacle segments (smaller clearance → larger penalty).
- P_smooth: penalty proportional to curvature jerk Δκ/Δs.

Normalization:
- Normalize each metric per pair X→Y using min-max across the candidate set to obtain comparable scales.

Thresholds (example defaults; adapt to your platform):
- κ_max ≤ 0.25–0.35 1/m (depends on min turning radius).
- Δκ/Δs capped at a comfortable steering rate for your controller.
- Clearance ≥ 0.20–0.30 m from static obstacles.

Selection:
- Keep top-k% (e.g., 10–20%) for manual review; typically select the **single best** for production and keep **2 backups**.

---

## 5) Output Normalization (Why Controllers Love It)

Uniform spacing
- Resample to a fixed Δs so that controllers with fixed lookahead rely on predictable geometry.

Monotone progress
- Ensure cumulative arc length s is strictly increasing; no backtracking segments.

Heading and yaw rate
- Enforce yaw continuity; limit |Δyaw| between consecutive samples to suppress micro-oscillations.

Endpoint contracts
- Start tangent should roughly align with the incoming node’s expected heading; we optionally pin the first 2–3 samples to ensure a clean hand-off across segments.

---

## 6) Parameter Tuning Guide

Core RRT knobs
- step_size: too small → slow convergence, jaggy; too large → poor clearance. Start at 0.20 m; sweep 0.10–0.30.
- goal_sample_rate: higher accelerates convergence but can cause “bee-lining” through tight spaces; 0.10–0.20 works well.
- max_iter: scale with corridor width; narrow passages may require 3000–5000.

Smoothing knobs
- Resample Δs: 0.05–0.15 m depending on controller frequency and speed.
- Savitzky–Golay window (odd): 9–21; poly order 2–3.
- Spline tension: small to avoid overshoot; keep curvature bounded.

Scoring weights (example starting set)
- w_L = 1.0, w_k̄ = 2.0, w_kmax = 4.0, w_cont = 2.0, w_clear = 3.0, w_smooth = 2.0.

Grid search
- Run small sweeps of (step_size, goal_sample_rate, weights) per “difficult” pair and log the Pareto front.

---

## 7) Collision Environment and Clearance

Map model
- Use a 2D occupancy / polygon map aligned to the same coordinate frame as waypoints.
- Inflate static obstacles by the vehicle half-width + a safety margin.

Collision check function
- For a segment (p_i → p_{i+1}), discretize at ≤ 0.5·step_size and test against inflated obstacles.
- For speed, precompute a distance field over the map; reject points with distance < clearance_min.

Clearance scoring
- Along each candidate, accumulate minimum distance to obstacles; penalize segments that graze boundaries.

---

## 8) Reproducibility, Determinism, and Caching

Randomness control
- Record RNG seed per candidate; store in metadata for exact replay.

Version pinning
- Log library versions (NumPy, your geometry libs, Python) and map revision.

Cache artifacts
- Keep intermediate candidate polylines, not only champions, for later audits.

Metadata sidecar (YAML or JSON)
- Save: X, Y indices; seed; L, κ̄, κ_max; min clearance; timestamps; chosen parameters; score components.

---

## 9) Directory Layout (Suggested)

rrt/  
├─ README.md  
├─ configs/  
│  ├─ nodes.yaml                (node indices, anchor coordinates)  
│  └─ rrt_params.yaml           (defaults per track or per pair)  
├─ maps/                        (track map, inflated obstacles, distance field)  
├─ scripts/                     (generators, evaluators, visualizers)  
├─ candidates/                  (optional cache of raw candidates)  
├─ outputs/  
│  ├─ waypoints_13to21.json     (champion per pair)  
│  └─ ...  
└─ rrt_logs/  
   ├─ rrt_eval_YYYYMMDD.csv     (metrics & reasons for selection)  
   └─ rrt_debug/                (per-candidate diagnostics)

---

## 10) Logging and Evaluation (CSV Schema)

CSV columns (example)
- pair_id, X, Y  
- seed, step_size, goal_sample_rate, max_iter  
- length_m, curv_avg, curv_max, curv_jerk_avg  
- clearance_min, clearance_avg  
- heading_jump_penalty, smooth_penalty, continuity_penalty  
- score_total, rank, selected_flag  
- notes (free-text; e.g., “tight chicane; spline t=0.3”)

Why log this much?
- Post-mortems are faster; when a path feels “nervous,” you can correlate with curvature jerk or heading penalties.

---

## 11) Visualization and QA

Sanity plots
- Plot x-y polylines over the map with obstacle inflation shown.
- Plot yaw vs. arc length; curvature vs. arc length; mark κ_max.
- Compare champion vs. runner-ups for a pair (X→Y).

Controller-in-the-loop checks
- Feed the normalized polylines into your low-level controller in simulation; inspect steering rate and cross-track error envelopes.

Continuity at junctions
- Visualize hand-offs node-to-node (…→ X→Y → …) to ensure tangent continuity and no lateral jumps.

---

## 12) Integration with Planning

Contracts
- Each `waypoints_XtoY.json` must use the **same frame** as the planner (`map` or track plane) and **meters** as units.
- Sampling density should match runtime lookahead (e.g., Δs ≈ v / f_ctrl).

Streaming
- `helper_path_sender` reads `dqn_paths.json` (global node sequence), loads the associated `waypoints_*` for each hop, and **streams** (no reach gating) to `/path_x`, `/path_y`.

Boundaries and avoidance
- At ID boundaries, the Obstacle_Avoidance module **merges current+next** to compute offsets, ensuring continuous rejoin.

---

## 13) Failure Modes and Remedies

Failure mode: jaggy micro-turns on straights  
- Cause: uneven spacing or spline overshoot.  
- Fix: re-resample; lower spline tension; apply SG filter before resampling.

Failure mode: κ_max exceeds vehicle limits in tight corners  
- Cause: insufficient sampling or aggressive smoothing.  
- Fix: enforce curvature cap in scoring; allow slightly longer path that lowers κ_max.

Failure mode: poor clearance near inner apex  
- Cause: RRT goal bias cutting too close.  
- Fix: inflate obstacles more; add mid-way anchor(s); penalize small clearance in scoring.

Failure mode: heading jump at the start/end  
- Cause: segment endpoints not aligned with adjacent segments.  
- Fix: pin first/last tangents; perform end-segment smoothing over last 3–5 points.



