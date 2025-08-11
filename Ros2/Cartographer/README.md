# Cartographer (SLAM) for ROS 2 — README

This package documents how we run **Cartographer 2D** on ROS 2 Humble to provide a stable pose for the autonomous stack in a **static track** environment. It focuses on engineering details that made the system robust on the real vehicle (QCar2), especially under mixed **Docker/host** deployments and heterogeneous sensor clocks.

---

## 0) Goals & Constraints

Goals
- Provide **real-time 2D SLAM pose** (`map → base_link`) on a static track.
- Keep the SLAM pose consistent with the rest of the pipeline (Perception / Planning / Control).

Constraints
- Mixed runtime: Docker containers + host processes.
- Possible mismatches in frames and timestamps across **LiDAR / IMU / camera**.
- The rest of the pipeline assumes stable TF tree and reproducible timing.

---

## 1) Problem Statement

- Running Cartographer in Docker frequently produced **/tf frame mismatches** and **timestamp drift** (sensor vs container clocks).
- Over longer runs, **drift accumulates**, which can move the planning/control reference frame and destabilize behavior.

---

## 2) Attempts → Failures → Improvements

V0 — Basic port (Foxy → Humble)
- Symptoms: inconsistent frame names and TF tree; time bases not aligned; sporadic initialization failures.

V1 — Custom TF broadcaster (fix frames)
- Introduced a small `lidar_tf_broadcaster.py` that **publishes `base_link → laser`** at a fixed rate.
- Standardized frame names and placed **`map / odom / base_link`** into `/tf_static` where appropriate.

V1.1 — Time synchronization (fix clocks)
- Enabled `use_sim_time=true` for nodes that respect `/clock`.
- Normalized all consumers to a single **clock source**; used **receipt-time interpolation** for IMU/LiDAR if needed.
- Result: initialization success rate up; reproducibility up.

V2 — Vision-assisted drift monitoring
- Compared **SCNN 3D centerline** lateral position to the **SLAM pose** to estimate lateral error `e_y`.
- When the error persists above a threshold, we flag **drift segments** and trigger recovery (alerts / re-initialize map).

---

## 3) Nodes, Topics, TF

Main nodes
- `cartographer_node` — SLAM backend; consumes sensors and publishes pose/TF.
- `cartographer_occupancy_grid_node` — publishes 2D occupancy map for visualization/debugging.
- `lidar_tf_broadcaster` (custom) — continuously publishes the rigid transform `base_link → laser` to avoid TF gaps.

Sensor inputs
- `/qcar/scan` (LiDAR)
- `/qcar/imu`  (IMU)
- `/clock`     (optional, in Docker/sim-time setups)

Pose/TF outputs (typical)
- `/tf` — dynamic transforms (e.g., `map → base_link` when using Cartographer to provide localization).
- `/tf_static` — static frames (e.g., `map ↔ odom` if fixed; `base_link ↔ laser` if true rigid and not expected to move).
- `/map` — occupancy grid (from occupancy node).

Recommended TF tree (target)
- map → odom → base_link → laser
- base_link ↔ camera and other sensors are static extrinsics published once at startup (prefer `/tf_static`).
- If your platform requires an odometry frame from Cartographer, set `provide_odom_frame=true` and align downstream consumers.

---

## 4) Configuration (Humble)

Keep the following parameters consistent across the launch and Lua config. Do not rely on defaults; pin them explicitly.

Core frames (yaml style shown inline; place in your package’s param file)
- map_frame: map  
- tracking_frame: base_link  
- published_frame: base_link  
- odom_frame: odom  
- provide_odom_frame: true  
- use_odometry: false  
- use_nav_sat: false  
- use_landmarks: false  
- publish_period_sec: 0.5  

Trajectory builder (2D; in your `.lua`)
- TRAJECTORY_BUILDER_2D.use_imu_data = true  
- TRAJECTORY_BUILDER_2D.min_range = 0.2  
- TRAJECTORY_BUILDER_2D.max_range = 6.0  
- POSE_GRAPH.optimization_problem.huber_scale = 1e1  
- POSE_GRAPH.optimize_every_n_nodes = 90  

Practical notes
- If your LiDAR is close to the ground, keep `min_range` conservative to avoid ground returns dominating scans.
- Start with `optimize_every_n_nodes ≈ 90` and adjust by map size and CPU budget.
- On QCar-like footprints, using IMU improves heading stability significantly; ensure IMU orientation is correct in TF.

---

## 5) Vision-Assisted Validation and Mitigation

- Compute **lateral error** `e_y` between the **SCNN centerline-derived position** and the **SLAM pose** at the same arc length.
- Log `time, e_y, pose_source, vehicle_speed` to `logs/slam_drift.csv`.
- If `e_y` persists above a threshold (e.g., 0.30 m for N seconds), trigger:
  - A re-localization attempt (reseed pose to a nearby consistent section),
  - Or a soft map reset during a safe stop window,
  - And raise an operator alert.

This cross-check provides an independent signal for drift segments and complements Cartographer’s internal loop-closure.

---

## 6) Real-Vehicle Issues & Remedies

Issue: TF mismatch (wrong or missing frames)
- Remedy: enforce `base_link → laser` via the dedicated **TF broadcaster**; standardize names; put rigid extrinsics into `/tf_static`.

Issue: IMU clock mismatch or jumps
- Remedy: unify `/clock` across nodes; when not possible, **interpolate on receipt time** and document offsets; prefer hardware timestamps.

Issue: Initial pose “jumps” or delayed convergence
- Remedy: delay consumers until SLAM becomes stable; compare against SCNN centerline and **retry initialization** if the early error is large.

Issue: Mixed Docker/host time bases
- Remedy: pick a single source of truth for time; export it to containers; verify `/use_sim_time` is **consistently true/false** everywhere.


---

## 7) Health Metrics to Log

- Cartographer status: loop-closure counts, constraint residual stats, pose graph optimization time.
- TF continuity: rate and latency of `/tf` broadcasts; missing-frame alerts.
- Drift monitor: `e_y` statistics and flagged segments; correlate with vehicle speed and turns.
- Map freshness: time since last optimization; number of nodes since last loop closure.

---


## 8) Summary

- Robust SLAM in our stack came from **three pillars**: correct TF, unified time, and an **independent vision-based drift monitor**.  
- Keep frame names consistent, publish rigid extrinsics in `/tf_static`, and prefer a **single clock** across Docker/host.  
- Use SCNN centerline as a **sanity check** to keep Cartographer honest and the vehicle predictable.

