# Cartographer

This package documents how we run **Cartographer 2D** on ROS 2 Humble to provide a stable pose for the autonomous stack in a **static track** environment. It focuses on engineering details that made the system robust on the real vehicle (QCar2), especially under mixed **Docker/host** deployments and heterogeneous sensor clocks.

---

## 0) Goals & Constraints

**Goals**
- Provide **real‑time 2D SLAM pose** (`map → base_link`) on a static track.
- Keep the SLAM pose consistent with Perception / Planning / Control.

**Constraints**
- Mixed runtime: Docker containers + host processes.
- Possible mismatches in frames/timestamps across **LiDAR / IMU / camera**.
- The rest of the pipeline assumes a stable TF tree and reproducible timing.

---

## 1) Problem Statement

- Running Cartographer in Docker produced **/tf frame mismatches** and **timestamp drift** (sensor vs container clocks).
- Over longer runs, **drift accumulates**, moving the planning/control reference frame and destabilizing behavior.

---

## 2) Attempts → Failures → Improvements

**V0 — Basic port (Foxy → Humble)**
- Symptoms: inconsistent frame names and TF tree; time bases not aligned; sporadic initialization failures.

**V1 — Custom TF broadcaster (fix frames)**
- Added `lidar_tf_broadcaster.py` that **publishes `base_link → laser`** at a fixed rate.
- Standardized frame names and pushed rigid extrinsics to **`/tf_static`**.

**V1.1 — Time synchronization (fix clocks)**
- Enabled `use_sim_time=true` when `/clock` present.
- Normalized consumers to a **single clock source**; used **receipt‑time interpolation** for IMU/LiDAR if needed.
- Result: higher init success; better reproducibility.

**V2 — Vision‑assisted drift monitoring**
- Compared **SCNN 3D centerline** lateral position to the **SLAM pose** to estimate cross‑track error `e_⊥`.
- When persistent above threshold, flag **drift segments** and trigger recovery (alerts / re‑init).

---

## 3) Nodes, Topics, TF

**Main nodes**
- `cartographer_node` — SLAM backend; consumes sensors, publishes pose/TF.
- `cartographer_occupancy_grid_node` — publishes 2D occupancy map for visualization/debug.
- `lidar_tf_broadcaster` (custom) — continuously publishes `base_link → laser` to avoid TF gaps.

**Sensor inputs**
- `/qcar/scan`  — LiDAR (`sensor_msgs/LaserScan`)
- `/qcar/imu`   — IMU   (`sensor_msgs/Imu`)
- `/clock`      — optional (bag replay / sim‑time)

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

**QoS guidance**
- `/tf_static`: **reliable + transient_local** so restarts recover geometry.
- Sensors: SensorData QoS (best_effort, low latency). SLAM outputs: reliable, keep_last(10).

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

**Trajectory builder (Lua)**
```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_period_sec = 0.5,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 6.0
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 90

return options
```

**Practical notes**
- Low LiDAR height → keep `min_range` conservative to avoid ground returns dominating scans.
- Start `optimize_every_n_nodes ≈ 90`; tune by map size and CPU budget.
- Ensure IMU orientation is correct in TF; using IMU greatly improves heading stability.

---

## 5) Vision‑Assisted Validation & Mitigation

- Compute **cross‑track error** `e_⊥` between the **SCNN centerline** (in `map`) and the **SLAM pose** at a fixed lookahead.
- Log `time, e_⊥, pose_source, vehicle_speed` to `logs/slam_drift.csv`.
- If `mean(|e_⊥|) > e_thresh` for `T_window` seconds (e.g., 0.30 m for 3 s):
  - Attempt re‑localization (reseed near a consistent section), or
  - Perform a **soft reset** during a safe stop window, and
  - Raise an operator alert.

This cross‑check complements Cartographer’s loop closure and catches clock/TF issues early.

---

## 6) Bring‑Up & Launch

**A) Cold start (on vehicle)**
```bash
# 1) Static TF + (optional) SLAM
ros2 launch slam cartographer.launch.py use_sim_time:=false
# 2) Start lidar_tf_broadcaster (if your laser extrinsic is not static)
ros2 run slam lidar_tf_broadcaster
```
- Start perception/planning after SLAM is publishing; controllers should wait for a valid `map → base_link`.

**B) Bag replay (diagnostics)**
```bash
# Provide /clock first
ros2 bag play your.bag --clock
# Launch SLAM + broadcasters with use_sim_time:=true
ros2 launch slam cartographer.launch.py use_sim_time:=true
```
- Publish `/tf_static` with **transient_local** before consumers start.

**C) Partial stack**
- Run SLAM + TF only; verify TF tree in RViz and `/tf` continuity.

---

## 7) Real‑Vehicle Issues & Remedies

| Issue | Likely Cause | Remedy |
|---|---|---|
| TF mismatch (wrong/missing frames) | Incomplete extrinsics | Enforce `base_link → laser` via **TF broadcaster**; push rigid frames to `/tf_static`. |
| IMU clock mismatch / jumps | Multiple clocks | Single `/clock`; interpolate on receipt time; prefer HW timestamps. |
| Initial pose jumps / slow convergence | Early bad data / wrong orientation | Delay consumers until stable; validate against SCNN centerline; retry init if early error is large. |
| Mixed Docker/host time bases | Competing clocks | Pick a single source of truth; export to containers; keep `use_sim_time` consistent. |

---

## 8) Health Metrics to Log

- **Cartographer**: loop‑closure count, constraint residuals, optimization latency.
- **TF**: `/tf` broadcast rate/latency; missing‑frame alerts.
- **Drift**: `e_⊥` stats and flagged segments; correlate with speed/turns.
- **Map freshness**: time since last optimization; nodes since last loop closure.

---

## 9) Minimal TF Broadcaster

```python
# lidar_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class LidarTFBroadcaster(Node):
    def __init__(self):
        super().__init__('lidar_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        # TODO: load from params
        self.parent = 'base_link'
        self.child = 'laser'
        self.xyz = (0.20, 0.0, 0.10)
        self.q = (0.0, 0.0, 0.0, 1.0)
    def tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = self.q
        self.br.sendTransform(t)

if __name__ == '__main__':
    rclpy.init()
    rclpy.spin(LidarTFBroadcaster())
```

---

## 10) Version Matrix

| Component | Tested Version |
|---|---|
| ROS 2 | Humble |
| Cartographer | `cartographer_ros` (Humble‑compatible) |
| LiDAR | 10–15 Hz (LaserScan) |
| IMU | 100–200 Hz |

---

## 11) Summary

Robust SLAM in our stack came from **three pillars**: correct TF, a **single clock**, and an **independent vision‑based drift monitor**. Keep frame names consistent, publish rigid extrinsics in `/tf_static`, and standardize timing across Docker/host. Use the SCNN centerline as a **sanity check** to keep Cartographer honest and the vehicle predictable.
