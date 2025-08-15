# RRT

This module builds a **high-quality set of offline paths** for a **static track**. Instead of computing paths online, we repeatedly run **RRT** (dozens to thousands of trials per node pair X→Y), evaluate each candidate by **length / curvature / continuity / clearance**, smooth and resample it, and persist only the **best** as `waypoints_XtoY.json`. These curated segments are later stitched (via `dqn_paths.json`) and streamed at runtime by the planning stack.


---

## 0) Purpose

On a static course, **precomputing** robust segments is safer than relying on on-the-fly search. The objective is to:
- Generate multiple candidate paths for each directed pair **X→Y** (splits, corners, stops, pickup/dropoff).
- Score and filter candidates using geometric and dynamic criteria.
- **Normalize** spacing and heading, and smooth curvature.
- Save a **single champion** (or a small Pareto set) per pair to `waypoints_XtoY.json` (+ metadata).

**Frames & Units**
- All waypoints and node anchors are in the **`map`** (track-plane) frame, **meters**.
- If a different frame is required, provide a TF transform at **consume** time; do **not** mix frames in offline artifacts.

---

## 1) Process Timeline (Experiment Log Style)

**V0 — Single-shot RRT (baseline; did not meet quality)**
- One run per pair → high variance; occasional discontinuities and excessive curvature; micro-vibrations due to heading jitter.

**V1 — Multi-run + filtering (large improvement)**
- Run RRT **N times** per X→Y (N=50–2000). Keep top **k%** by composite score; discard the rest.

**V2 — Smoothing + heading normalization (stability)**
- Resample to uniform spacing (e.g., Δs=0.10 m); unwrap yaw; low-pass yaw; SG or spline for curvature cleanup.

**V3 — Connectivity & loop validation (track completeness)**
- Verify closed-loop traversal and no kinematic dead-ends; add “return-to-nominal” test (X→Y and return) under curvature limits.

Result: A repeatable pipeline that yields smooth, controller-friendly, and mutually compatible segments.

---

## 2) Operator Workflow (How to Use)

1) Define **semantic nodes** on the map (split, corner entry/exit, stop, pickup/dropoff). Store node indices and `(x, y)` anchors.

2) For each **directed pair** X→Y:
   - Run RRT **N** times; collect successful candidate polylines.
   - Evaluate, filter, and smooth; enforce uniform spacing (e.g., Δs=0.10 m).
   - Persist the **champion** to `waypoints_XtoY.json` and save a `waypoints_XtoY.meta.json` sidecar.

3) Naming convention: `waypoints_13to21.json` → path from node 13 to node 21.

4) Storage format: JSON list of `[x, y]` points (meters) plus optional `frame_id` (defaults to `map`). See §6.

5) Typical RRT parameters:
- `step_size`: 0.20 m
- `goal_sample_rate`: 0.10–0.20
- `max_iter`: ≥ 1000 (increase for narrow passages)
- `path_resolution`: 0.10 m (post-smoothing resample target)
- `collision_check_fn`: map-aware clearance (see §7)

---

## 3) Algorithm (What We Actually Do)

**Node Pair Expansion (per X→Y)**
- **Seeding**: set RNG seeds (multiple per pair) and sampling bounds.
- **RRT loop**:
  1. Sample a point; with prob. `goal_sample_rate`, sample in a neighborhood of Y.
  2. Extend nearest node toward the sample by `step_size` with collision checks.
  3. On goal reach, backtrack and export a raw polyline.

**Path formatting**
- Densify to target spacing (linear interpolation at Δs).
- **Shortcutting**: try straight-line replacements for small segments when collision-free.
- **Smoothing**: cubic spline or Bézier fragments; or coordinate-wise **Savitzky–Golay** (window 9–21, poly 2–3).

**Curvature & heading**
- Equal-arc sampling ensures stable estimates. For \( P_i=(x_i,y_i) \):
  - \( \kappa_i \approx |x' y'' - y' x''| / (x'^2 + y'^2)^{3/2} \) via centered differences.
  - `yaw_i = atan2(Δy, Δx)` → unwrap → (optional) low-pass.

**Metrics**
- Length **L**, mean curvature **κ̄**, peak curvature **κ_max**, jerk proxy **Δκ/Δs**, min/avg clearance, continuity penalties.

**Scoring**
- See §4 for the composite score and normalization.

---

## 4) Quality Criteria & Scoring

Composite score for candidate ranking:
```
S = w_L·norm(L) + w_kbar·norm(κ̄) + w_kmax·norm(κ_max)
  + w_cont·P_cont + w_clear·P_clear + w_smooth·P_smooth
```
**Terms**
- `L`: total length (short is good, but allow gentle detours that reduce κ_max).
- `κ̄`, `κ_max`: average and worst curvature (bounded by vehicle limits).
- `P_cont`: continuity penalty (heading jumps/discontinuities).
- `P_clear`: clearance penalty (distance to inflated obstacles).
- `P_smooth`: jerk penalty ∝ mean |Δκ/Δs|.

**Normalization**
- Per pair X→Y, min-max normalize each metric across candidates to [0,1].

**Thresholds (defaults)**
- `κ_max ≤ 0.25–0.35 1/m` (vehicle-dependent).
- |Δκ/Δs| below controller’s steering-rate comfort.
- `clearance_min ≥ 0.20–0.30 m` from inflated obstacles.

**Selection**
- Keep top `k%` (10–20%) for review; select **one champion** and keep **2 backups**.

---

## 5) Output Normalization (Why Controllers Love It)

- **Uniform spacing**: resample to fixed Δs so fixed-lookahead controllers see predictable geometry.
- **Monotone progress**: strictly increasing arc length; no backtracking segments.
- **Heading continuity**: unwrap yaw; limit |Δyaw| per sample; bound curvature.
- **Endpoint contracts**: align start tangent with upstream node heading; optionally pin first/last 2–3 samples.

---

## 6) JSON Schemas & Examples


### 6.1 `waypoints_XtoY.json`
```json
{
  "frame_id": "map",
  "points": [[0.10, 0.20], [0.60, 0.30], [1.10, 0.40]],
  "spacing_m": 0.10
}
```

---

## 7) Collision Environment & Clearance

**Map model**
- 2D occupancy or polygonal map aligned to `map`. Inflate obstacles by vehicle half-width + safety margin.

**Collision check**
- For segment `(p_i → p_{i+1})`, discretize at ≤ `0.5·step_size`; reject points with `distance < clearance_min`.
- Precompute a **distance field** for speed; cache it on disk keyed by map revision.

**Clearance scoring**
- Accumulate minimum/average distance along the candidate; penalize boundary grazes.

---

## 8) Parameters

| Group | Parameter | Type | Default | Notes |
|---|---|---|---:|---|
| rrt | `step_size` | float | 0.20 | Extension step (m).
| rrt | `goal_sample_rate` | float | 0.15 | Goal bias probability.
| rrt | `max_iter` | int | 2000 | Increase for narrow passages.
| format | `resample_ds` | float | 0.10 | Uniform spacing target (m).
| smooth | `sg_window` | int | 15 | Odd, 9–21 typical.
| smooth | `sg_poly` | int | 3 | 2–3 typical.
| limits | `kappa_max` | float | 0.30 | 1/m; cap in smoothing/selection.
| limits | `clearance_min` | float | 0.30 | Meters; inflated obstacles.
| scoring | `w_L` | float | 1.0 | Length weight.
| scoring | `w_kbar` | float | 2.0 | Mean curvature weight.
| scoring | `w_kmax` | float | 4.0 | Peak curvature weight.
| scoring | `w_cont` | float | 2.0 | Continuity penalty weight.
| scoring | `w_clear` | float | 3.0 | Clearance penalty weight.
| scoring | `w_smooth` | float | 2.0 | Jerk penalty weight.


---

## 9) Reproducibility, Determinism, & Caching

- **Randomness control**: record RNG seed per candidate; store in sidecar.
- **Version pinning**: log NumPy/Python versions and **map revision**.
- **Caching**: keep raw candidate polylines for audits; store per-pair CSV logs (see §10).

---

## 10) Integration with Planning

**Contracts**
- Each `waypoints_XtoY.json` uses the **same frame** as the planner (`map`) and **meters** as units.
- Sampling density should match runtime lookahead (`Δs ≈ v / f_ctrl`).

**Streaming**
- `helper_path_sender` reads `dqn_paths.json`, loads `waypoints_*` for each hop, and **streams** (no reach gating) to `/planned_path` (and legacy `/path_x`, `/path_y`).

**Boundaries & avoidance**
- At ID boundaries, Obstacle_Avoidance **merges current+next** before computing offsets for a continuous rejoin.

---

## 11) Failure Modes & Remedies

| Failure mode | Cause | Fix |
|---|---|---|
| Jaggy micro-turns on straights | Uneven spacing or spline overshoot | Re-resample; lower spline tension; SG before resampling. |
| κ_max exceeds vehicle limits | Aggressive smoothing / insufficient samples | Enforce curvature cap in scoring; allow slightly longer path lowering κ_max. |
| Poor clearance near apex | Goal bias cuts too close | Inflate obstacles more; add mid-way anchors; penalize small clearance. |
| Heading jump at start/end | Endpoint misalignment | Pin first/last tangents; end-segment smoothing over last 3–5 points. |

---

