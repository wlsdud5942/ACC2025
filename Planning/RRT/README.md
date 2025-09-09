# RRT — Offline Path Bank for a Static Track

On a static course, it is safer and more controllable to **precompute** a bank of segments offline by running **RRT tens to thousands of times** per node pair, rank the candidates with clear metrics, smooth/resample them, and keep only the **champions** (`waypoints_XtoY.json`).

---

## 0) Purpose
- For each directed pair **X→Y**, generate many candidates, evaluate them by geometry & safety, normalize spacing/heading/curvature, and keep the **top 1–3** with metadata.

---

## 1) Pipeline (concise)
- **V1**: multi-run (N=50–2000) → keep **top k%** by composite score.  
- **V2**: uniform resampling (e.g., Δs = 0.10 m), yaw unwrap/low-pass, curvature cleanup (SG or spline).  
- **V3**: connectivity & loop validation (round-trip feasible within curvature limits).

---

## 2) Quality metrics & score (why these)

**Metrics**
- Length \(L\) — shorter is better unless curvature spikes.  
- Mean/peak curvature \(\bar{\kappa}, \kappa_{\max}\) — respect vehicle limits.  
- Continuity \(P_{\text{cont}}\) — penalize heading jumps/discontinuities.  
- Clearance \(P_{\text{clear}}\) — distance to inflated obstacles.  
- Smoothness \(P_{\text{smooth}}\) — jerk proxy \( \lvert \Delta \kappa / \Delta s \rvert \) mean.

**Normalized weighted score (default)**
\[
S = w_L\tilde L + w_{\bar\kappa}\widetilde{\bar\kappa} + w_{\kappa_{\max}}\widetilde{\kappa_{\max}}
  + w_{\text{cont}}P_{\text{cont}} + w_{\text{clear}}P_{\text{clear}} + w_{\text{smooth}}P_{\text{smooth}}
\]
- Simple & interpretable; per-pair **min–max normalization** removes scale issues.  
- Typical limits: \( \kappa_{\max} \le 0.30\,\mathrm{m}^{-1} \), `clearance_min ≥ 0.30 m`.

**Alternative (for auditing)**: keep the **Pareto front** of non-dominated candidates to study tuning sensitivity.

---

## 3) Performance reporting (for large trials)
- **Success rate** (reaches goal without collision), **mean/95th length**, **\(\kappa_{\max}\) distribution**, **clearance distribution**, **generation time (ms)**.  
- **A/B replay**: compare previous champion vs. new candidate under the same controller (fixed lookahead/speed) and log tracking error, saturation rate, yaw-rate peaks.

---

## 4) Output format (example)

~~~json
{
  "frame_id": "map",
  "points": [[0.10, 0.20], [0.60, 0.30], [1.10, 0.40]],
  "spacing_m": 0.10
}
~~~

All coordinates are in `map` (meters). **Do not mix frames** in offline artifacts.

---

## 5) Parameters (excerpt)

| Group | Param | Default | Note |
|---|---|---:|---|
| rrt | `step_size` | 0.20 | Extension step (m) |
| rrt | `goal_sample_rate` | 0.15 | Goal bias probability |
| rrt | `max_iter` | 2000 | Increase for narrow passages |
| format | `resample_ds` | 0.10 | Target uniform spacing (m) |
| smooth | `sg_window` | 15 | Savitzky–Golay window (odd) |
| smooth | `sg_poly` | 3 | SG polynomial order |
| limits | `kappa_max` | 0.30 | Peak curvature bound (1/m) |
| limits | `clearance_min` | 0.30 | Min clearance (m) |

---

## 6) Integration
- `helper_path_sender` reads `dqn_paths.json`, loads `waypoints_*` per leg, and **streams by time** to `/planned_path` (and legacy `/path_x`, `/path_y`).  
- At ID boundaries, the avoidance layer **merges current + next** before offsetting, so rejoin is continuous.
