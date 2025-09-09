# ROS 2 

End-to-end ROS 2 (Humble) stack that connects **Perception (SCNN / YOLOv7)**, **Planning (RRT / DQN / Path Sender)**, **Control (FSM; Simulink codegen)**, **Obstacle Avoidance**, and **SLAM (Cartographer)** on the real vehicle (QCar2).  
The design emphasizes **timing robustness**, **deterministic handshakes**, and **simple, well-documented topic contracts**.

---

## 0) Goals & Design Constraints

**Goals**
- Build a **reproducible** on-vehicle pipeline from perception → planning → control → avoidance/SLAM.
- Keep synchronization between Python ROS 2 nodes and **Simulink-generated C++ FSM** **stable and observable**.
- Stream waypoints **without distance-based reach checks**; use **explicit events** for stop/pickup and other transitions.

**Constraints**
- Minimize event/timing mismatch with the Simulink FSM.
- Avoid localization-dependent reach triggers (noisy & laggy).
- Run reliably in **mixed Docker/host** deployments; keep TF frames & time discipline consistent.

---

## 1) Problem → Hypotheses → Outcomes

**Observed problems**
1. **FSM desync**: Distance-based “reach triggers” caused ordering/latency mismatches with the Simulink FSM.  
2. **Boundary fragility**: At path-ID boundaries, perception/SLAM/planning events did not align → failed hand-offs.  
3. **SLAM drift**: Cartographer accumulates small drift over long runs.

**Hypotheses & Outcomes**
- **(G1)** Stream waypoints strictly on a **timer** and drive FSM transitions with **explicit event topics** → fewer desyncs. **(Observed: success)**  
- **(G2)** Treat **transitions** as **continuous curves** (avoid artificial ID boundaries) in perception/avoidance/planning → smoother hand-offs. **(Observed: success)**  
- **(G3)** Compare **SCNN 3D centerline** to **SLAM pose** to monitor and mitigate long-term drift → early detection/reset hooks. **(Observed: success)**

---

## 2) Architecture Overview

- **Perception**
  - **SCNN**: RGB → lane mask → **centerline (3D)**.
  - **YOLOv7**: RGB(+Depth) → objects/stop-line → `/obstacle_info`, `/yolo_stop`.

- **Planning**
  - **Offline**: `rrt/` → `waypoints_*.json`; `dqn/` → `dqn_paths.json`.  
  - **Online**: `HelperPathSender` streams waypoints from the **DQN total sequence** on a fixed timer (no distance gating).

- **Obstacle Avoidance**
  - Lateral **offset-curve** over **merged (current+next) path**; **gates** base stream while active.  
  - Emits `/avoid_start`, `/avoid_done`; streams temporary avoidance waypoints.

- **Control (FSM)**
  - Simulink **code-generated C++** node; consumes `/stop`, `/pickup_dropoff`, `/path_mode`; follows streamed waypoints.

- **SLAM**
  - **Cartographer 2D** (LiDAR + IMU); TF: `map → odom → base_link → sensors`.

---

## 3) Interface Matrix (Topics & QoS)

| Module | Subscriptions | Publications | QoS (suggested) | Notes |
|---|---|---|---|---|
| **SCNN** | `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` | `/lane_mask`, `/centerline_path` (std), `/centerline_3d` (legacy) | Sensors: SensorData (best_effort); Outputs: reliable, keep_last(10) | `frame_id` from color optical; outputs default `base_link` |
| **YOLOv7** | `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` | `/objects_3d` (legacy), `/detections_3d` (std, optional), `/yolo_stop` | Sensors: SensorData; Outputs/Events: reliable, keep_last(10) | Depth windowed median; publish meters |
| **HelperPathSender** | `/dqn_done`, `/location` | `/planned_path` (std), `/path_x`, `/path_y`, `/path_mode`, `/stop`, `/pickup_dropoff` | reliable, keep_last(10); events optionally `transient_local` | **Time-based streaming** (no distance gating) |
| **ObstacleAvoidance** | `/obstacle_info`, `/location` | `/avoid_start`, `/avoid_done`, avoidance `/planned_path` or `/path_x/y` | Events: reliable + transient_local; paths: reliable | **Gate** base stream when active |
| **FSM (Simulink)** | `/planned_path` or `/path_x/y`, `/stop`, `/pickup_dropoff`, `/path_mode` | `/cmd` | reliable, keep_last(10) | Event-driven only |
| **SLAM (Cartographer)** | sensor topics, `/clock` (if bag) | `/tf`, `/tf_static`, `/map` | `/tf_static`: reliable + transient_local | Maintain single clock domain |

### 3.1 Publish→Subscribe→Purpose (condensed, project-specific)

| Producer → Consumer | Topic | Purpose |
|---|---|---|
| SCNN → Planning/Control | `/centerline_path` | 3D lane centerline; also used for drift check vs SLAM. |
| YOLO → FSM | `/yolo_stop` | Debounced stop event (0/1). |
| YOLO(+Depth) → Avoidance | `/obstacle_info` | Seed local offset-curve around obstacles. |
| Path Sender → FSM/Controller | `/planned_path`, `/path_x`, `/path_y` | **Timer/index-based** sliding window; no distance checks. |
| Path Sender → FSM | `/stop`, `/pickup_dropoff`, `/path_mode` | Behavior events; deterministic handshakes. |
| Cartographer → All | `/tf`, `/tf_static` | Global pose/frames for consistent transforms. |

---

## 4) Frames & Time Discipline

- **Frames**: `map` (global), `odom` (continuous local), `base_link` (vehicle), `camera_*_optical_frame`.  
- **TF Tree**: `map → odom → base_link → {camera, lidar, imu}`.  
- **/tf_static**: publish with **reliable + transient_local** so restarts recover geometry.  
- **Clock**:  
  - **Live**: `use_sim_time=false`; prefer hardware device stamps.  
  - **Bag**: `use_sim_time=true`; launch `/clock` **before** consumers.  
- **Docker/host**: single `/clock`, unified NTP/chrony, one `ROS_DOMAIN_ID`.

---

## 5) Bring-Up Runbooks

### A) Cold start (on vehicle)
```bash
ros2 launch slam cartographer.launch.py use_sim_time:=false
# start SCNN, YOLOv7
# ensure planning artifacts exist: rrt/outputs/*, dqn/dqn_paths.json
ros2 run planning helper_path_sender  # publish_nav_path:=true
# start Simulink FSM (subscribes: /planned_path or /path_x/y, /stop, /pickup_dropoff, /path_mode)
```

### B) Bag replay (diagnostics)
```bash
ros2 bag play your.bag --clock          # 1) provide /clock
# 2) publish /tf_static (transient_local)
# 3) launch stack with use_sim_time:=true
```

### C) Partial stack
- Module + TF + (optional) clock; use RViz overlays; record outputs for tuning.

---

## 6) Events, Gating, and Switching

- **Streaming vs Events**: Paths are **timer/index-based**; **never** gated by distance. FSM transitions are **events** only.  
- **Avoidance gating**: `/avoid_start` → controller prioritizes avoidance stream; resume base stream on `/avoid_done`.  
- **Segment switching**: Path Sender transitions by **index** (preloaded); optional ROI hints are allowed but must not rely on noisy distance thresholds.

---

## 7) Global Parameters

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `output_frame_id` | string | `map` | Global path frame (static track). |
| `timer_hz` | float | 2.0 | HelperPathSender streaming rate. |
| `window_size` | int | 4 | Sliding window size. |
| `events_transient_local` | bool | true | Events survive restarts. |
| `use_sim_time` | bool | false | `true` for bag replay. |
| `drift_e_thresh_m` | float | 0.25 | Centerline vs SLAM cross-track error threshold. |

---

## 8) Leak/Loss Budget & QoS Discipline (added)

**What we measured**
- **Drop rate** (pub→sub), **Age-of-Information (AoI)** = `now − header.stamp`, **jitter** (period std), queue overruns.

**Targets (control-critical)**
- Drops **0%**, AoI P95 ≤ **~40 ms**, jitter P95 ≤ **~10 ms**.

**QoS profiles we pinned (that fixed leaks)**
- **Images** (RGB / aligned depth): **SensorData** (`best_effort, keep_last=5`).
- **Paths** (`/planned_path`, `/path_x`, `/path_y`): **reliable, keep_last=10**.
- **Events** (`/stop`, `/pickup_dropoff`, `/path_mode`): **reliable, keep_last=10, transient_local**.
- **`/tf_static`**: **reliable + transient_local**.
- **`/dqn_done`**: **reliable, keep_last=1**.

**Quick checks**
```bash
ros2 topic hz /planned_path
ros2 topic bw /planned_path
# AoI: log (now - msg.header.stamp) in a tiny helper node
```

**Tiny guard helpers we use**
```python
# Steady timer tick
class RateGuard:
    def __init__(self, hz): self.dt=1.0/hz; self.next=0.0
    def tick(self, now): 
        if now>=self.next: self.next=now+self.dt; return True
        return False

# Exactly-once event with cooldown
class EventOnce:
    def __init__(self, cd_s=0.3): self.t=0.0; self.last=None; self.cd=cd_s
    def ok(self, v, now): 
        if v!=self.last or (now-self.t)>self.cd: self.t=now; self.last=v; return True
        return False
```

---

## 9) Docker Execution Notes (added)

- Same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` across containers.  
- Share `/etc/localtime`; pin CPUs for determinism; one **/clock** source (bag container first).  
- Document time offsets in `logs/run_meta.yaml`.

```yaml
# docker-compose excerpt
services:
  perception: { environment: [ROS_DOMAIN_ID=42, RMW_IMPLEMENTATION=rmw_fastrtps_cpp] }
  planning:   { environment: [ROS_DOMAIN_ID=42, RMW_IMPLEMENTATION=rmw_fastrtps_cpp] }
  control:    { environment: [ROS_DOMAIN_ID=42] }
```

---

## 10) Logging & Monitoring

- **Run manifest**: `logs/run_meta.yaml` — clock source, QoS, frames, versions.  
- **Drift**: `logs/slam_drift.csv` — `time, e_perp, pose_source, vehicle_speed`.  
- **Streaming**: segment index, timer periods, queue depths.  
- **Events**: `/stop`, `/pickup_dropoff`, `/avoid_start`, `/avoid_done` with timestamps.

---

## 11) Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| FSM/event jitter | Legacy distance gating | Remove distance checks; events only; `transient_local` on events. |
| Boundary hand-off glitch | ID-local assumption | Merge current+next path pre-transition; keep index-based switch. |
| `/planned_path` drops | QoS/CPU | `reliable, depth=10`; steady timer; avoid heavy callbacks. |
| Drift accumulates | Time/TF inconsistency | Single clock; monitor centerline vs pose; add reset hooks. |
| Missed events after restart | Volatile QoS | Use `transient_local` on event topics. |
| Mixed Docker/host time | Multiple `/clock` providers | Keep exactly one `/clock`; record offsets. |

---

## 12) Version Matrix

| Component | Tested Version |
|---|---|
| ROS 2 | Humble |
| RealSense | librealsense 2.x (aligned depth enabled) |
| PyTorch (YOLOv7 optional) | 1.12.x (Jetson FP16) |
| Cartographer | cartographer_ros (Humble) |
| Simulink ROS 2 toolbox | slros2 (matching Humble msgs) |
---
