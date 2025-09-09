# Cartographer

This package documents how we run **Cartographer 2D** on ROS 2 Humble to provide a stable pose for the autonomous stack in a **static track** environment. It focuses on engineering details that made the system robust on the real vehicle (QCar2), especially under mixed **Docker/host** deployments and heterogeneous sensor clocks.

---

## 0) Goals & Constraints

**Goals**
- Provide **real-time 2D SLAM pose** (`map → base_link`) on a static track.
- Keep the SLAM pose consistent with Perception / Planning / Control.

**Constraints**
- Mixed runtime: Docker containers + host processes.
- Possible mismatches in frames/timestamps across **LiDAR / IMU / camera**.
- The rest of the pipeline assumes a stable TF tree and reproducible timing.

---

## 1) Problem Statement (Observed on QCar2)

- Running Cartographer in Docker produced **/tf frame mismatches** and **timestamp drift** (sensor vs container clocks).
- Over longer runs, **drift accumulates**, moving the planning/control reference frame and destabilizing behavior.
- In **open-loop laps** (sparse loop closures), **noisy scans**, or on a **large map**, we occasionally observed mapping “breaks” (submap mis-alignment). We treat this as multi-factor: **loop-closure sparsity + sensor visibility + compute budget**.

---

## 2) Attempts → Failures → Improvements

**V0 — Basic port (Foxy → Humble)**  
- Symptoms: inconsistent frame names and TF tree; time bases not aligned; sporadic initialization failures.

**V1 — Custom TF broadcaster (fix frames)**  
- Added `lidar_tf_broadcaster.py` that **publishes `base_link → laser`** at a fixed rate.  
- Standardized frame names and pushed rigid extrinsics to **`/tf_static`**.

**V1.1 — Time synchronization (fix clocks)**  
- Enabled `use_sim_time=true` when `/clock` present.  
- Normalized consumers to a **single clock source**; used **receipt-time interpolation** for IMU/LiDAR if needed.  
- Result: higher init success; better reproducibility.

**V2 — Vision-assisted drift monitoring**  
- Compared **SCNN 3D centerline** lateral position to the **SLAM pose** to estimate cross-track error `e_⊥`.  
- When persistent above threshold, flag **drift segments** and trigger recovery (alerts / re-init).

**V3 — Open-loop & big-map mitigation (added)**  
- Encourage **intentional re-visits** in the lap to densify loop closures.  
- Pin Cartographer to dedicated CPU cores; avoid optimization spikes during heavy traffic.  
- Inputs use **SensorData QoS**; outputs **reliable**; `/tf_static` is **reliable + transient_local**.  
- If `e_⊥` exceeds threshold for `T_window`, perform a **soft reset** in a safe stop window.

---

## 3) Nodes, Topics, TF

**Main nodes**
- `cartographer_node` — SLAM backend; consumes sensors, publishes pose/TF.  
- `cartographer_occupancy_grid_node` — publishes 2D occupancy map for visualization/debug.  
- `lidar_tf_broadcaster` (custom) — continuously publishes `base_link → laser` to avoid TF gaps.

**Sensor inputs**
- `/qcar/scan`  — LiDAR (`sensor_msgs/LaserScan`)  
- `/qcar/imu`   — IMU   (`sensor_msgs/Imu`)  
- `/clock`      — optional (bag replay / sim-time)

**Pose/TF outputs (typical)**
- `/tf` — dynamic transforms (e.g., `map → base_link`).  
- `/tf_static` — static frames (e.g., `base_link ↔ laser`, `camera` extrinsics).  
- `/map` — occupancy grid (`nav_msgs/OccupancyGrid`).

**Recommended TF tree**
```
map → odom → base_link → laser
                 ↘ camera_*_optical_frame (via /tf_static)
```
- If you require an odometry frame from Cartographer, set `provide_odom_frame=true` and align downstream consumers.

**QoS guidance (project-specific)**  
- Inputs (LiDAR/IMU): **SensorData** (best_effort, low latency).  
- Outputs (`/tf`, `/map`): **reliable, keep_last(10)**.  
- `/tf_static`: **reliable + transient_local** so restarts recover geometry.

**ROS 2 Topics (summary)**
| Topic | Dir | Type | QoS | Notes |
|---|---|---|---|---|
| `/qcar/scan` | Sub | `sensor_msgs/LaserScan` | SensorData | 10–15 Hz |
| `/qcar/imu` | Sub | `sensor_msgs/Imu` | SensorData | 100–200 Hz |
| `/tf` | Pub | TF | reliable, keep_last(10) | dynamic transforms |
| `/tf_static` | Pub | TF | **reliable + transient_local** | rigid extrinsics |
| `/map` | Pub | `nav_msgs/OccupancyGrid` | reliable, keep_last(10) | visualization/debug |

---

## 4) Configuration (Humble)

Pin critical frames and timings explicitly; avoid relying on defaults.

**Core frames (YAML)**
```yaml
cartographer_ros:
  ros__parameters:
    map_frame: map
    tracking_frame: base_link
    published_frame: base_link
    odom_frame: odom
    provide_odom_frame: true
    use_odometry: false
    use_nav_sat: false
    use_landmarks: false
    publish_period_sec: 0.5
```

**Practical notes**
- Low LiDAR height → keep `min_range` conservative to avoid ground returns dominating scans.  
- Start `optimize_every_n_nodes ≈ 90`; tune by map size and CPU budget.  
- Ensure IMU orientation is correct in TF; using IMU greatly improves heading stability.

---

## 5) Vision-Assisted Validation & Mitigation

- Compute **cross-track error** `e_⊥` between the **SCNN centerline** (in `map`) and the **SLAM pose** at a fixed lookahead.  
- Log `time, e_⊥, pose_source, vehicle_speed` to `logs/slam_drift.csv`.  
- If `mean(|e_⊥|) > e_thresh` for `T_window` seconds (e.g., 0.30 m for 3 s):
  - Attempt re-localization (reseed near a consistent section), or  
  - Perform a **soft reset** during a safe stop window, and  
  - Raise an operator alert.

This cross-check complements Cartographer’s loop closure and catches clock/TF issues early.

---

## 6) Real-Vehicle Issues & Remedies

| Issue | Likely Cause | Remedy |
|---|---|---|
| TF mismatch (wrong/missing frames) | Incomplete extrinsics | Enforce `base_link → laser` via **TF broadcaster**; push rigid frames to `/tf_static`. |
| IMU clock mismatch / jumps | Multiple clocks | Single `/clock`; interpolate on receipt time; prefer HW timestamps. |
| Initial pose jumps / slow convergence | Early bad data / wrong orientation | Delay consumers until stable; validate against SCNN centerline; retry init if early error is large. |
| Mixed Docker/host time bases | Competing clocks | Pick a single source of truth; export to containers; keep `use_sim_time` consistent. |

---

## 7) Health Metrics to Log

- **Cartographer**: loop-closure count, constraint residuals, optimization latency.  
- **TF**: `/tf` broadcast rate/latency; missing-frame alerts.  
- **Drift**: `e_⊥` stats and flagged segments; correlate with speed/turns.  
- **Map freshness**: time since last optimization; nodes since last loop closure.

---

## 8) Version Matrix

| Component | Tested Version |
|---|---|
| ROS 2 | Humble |
| Cartographer | `cartographer_ros` (Humble-compatible) |
| LiDAR | 10–15 Hz (LaserScan) |
| IMU | 100–200 Hz |

---

## 9) Summary

Robust SLAM in our stack came from **three pillars**: correct TF, a **single clock**, and an **independent vision-based drift monitor**. Keep frame names consistent, publish rigid extrinsics in `/tf_static`, and standardize timing across Docker/host. Use the SCNN centerline as a **sanity check** to keep Cartographer honest and the vehicle predictable.
