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
2. **Boundary fragility**: At path‑ID boundaries, perception/SLAM/planning events did not align → failed hand‑offs.  
3. **SLAM drift**: Cartographer accumulates small drift over long runs.

**Hypotheses & Outcomes**
- **(G1)** Stream waypoints strictly on a **timer** and drive FSM transitions with **explicit event topics** → fewer desyncs. **(Observed: success)**  
- **(G2)** Treat **transitions** as **continuous curves** (avoid artificial ID boundaries) in perception/avoidance/planning → smoother hand‑offs. **(Observed: success)**  
- **(G3)** Compare **SCNN 3D centerline** to **SLAM pose** to monitor and mitigate long‑term drift → early detection/reset hooks. **(Observed: success)**

---

## 2) Architecture Overview

- **Perception**
  - **SCNN**: RGB → lane mask → **centerline (3D)**.
  - **YOLOv7**: RGB(+Depth) → objects/stop‑line → `/obstacle_info`, `/yolo_stop`.

- **Planning**
  - **Offline**: `rrt/` → `waypoints_*.json`; `dqn/` → `dqn_paths.json`.  
  - **Online**:  
    - `HelperPathSender`: time‑based waypoint streaming from **DQN total sequence**.  
    - `PathSenderNode` (optional): SCNN‑ROI–assisted segment switching.

- **Obstacle Avoidance**
  - Lateral **offset‑curve** over **merged (current+next) path**; **gates** base stream while active.  
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
| **HelperPathSender** | `/dqn_done`, `/location` | `/planned_path` (std), `/path_x`, `/path_y`, `/path_mode`, `/stop`, `/pickup_dropoff` | reliable, keep_last(10); events optionally `transient_local` | **Time‑based streaming** (no distance gating) |
| **PathSenderNode** | `/location`, `/lane_mask` or centerline ROI | `/planned_path` (std), `/path_x`, `/path_y`, `/path_mode` | reliable, keep_last(10) | ROI‑assisted segment switching |
| **ObstacleAvoidance** | `/obstacle_info`, `/location` | `/avoid_start`, `/avoid_done`, avoidance `/planned_path` or `/path_x/y` | Events: reliable + transient_local; paths: reliable | **Gate** base stream when active |
| **FSM (Simulink)** | `/planned_path` or `/path_x/y`, `/stop`, `/pickup_dropoff`, `/path_mode` | `/cmd` | reliable, keep_last(10) | Event‑driven only |
| **SLAM (Cartographer)** | sensor topics, `/clock` (if bag) | `/tf`, `/tf_static`, `/map` | `/tf_static`: reliable + transient_local | Maintain single clock domain |

---

## 4) Frames & Time Discipline

- **Frames**: `map` (global), `odom` (continuous local), `base_link` (vehicle), `camera_*_optical_frame`.  
- **TF Tree**: `map → odom → base_link → {camera, lidar, imu}`.  
- **/tf_static**: publish with **reliable + transient_local** so restarts recover geometry.  
- **Clock**:  
  - **Live**: `use_sim_time=false`; prefer hardware time from devices.  
  - **Bag**: `use_sim_time=true`; launch a `/clock` provider before anything else.  
- **Docker/host mixes**: pin NTP/chrony; avoid multiple `/clock` sources; document offsets in the run manifest.

---

## 5) Bring‑Up Runbooks

### A) Cold start (on vehicle)
1. Launch TF static + SLAM (or TF static only if no SLAM):  
   ```bash
   ros2 launch slam cartographer.launch.py use_sim_time:=false
   ```
2. Perception (SCNN, YOLOv7) in one composition or separate terminals.  
3. Planning offline artifacts present: `rrt/outputs/*`, `dqn/dqn_paths.json`.  
4. Start `helper_path_sender` with `publish_nav_path:=true` (or `path_sender_node` if using ROI switching).  
5. Start **FSM** (Simulink codegen): ensure it listens to `/stop`, `/pickup_dropoff`, `/path_mode` and consumes `/planned_path` or `/path_x/y`.

### B) Bag replay (diagnostics)
1. `use_sim_time:=true`; start a `/clock` provider (`ros2 bag play --clock` or a minimal clock node).  
2. Publish `/tf_static` first (transient_local).  
3. Launch stack components; verify stamps increase with `/clock`.  
4. Reproduce events and path streaming deterministically.

### C) Partial stack (perception‑only or planning‑only)
- Launch module + TF + a fake clock if needed. Use RViz overlays and record outputs for tuning.

---

## 6) Events, Gating, and Switching

- **Streaming vs Events**: Path streaming is **timer/index‑based**; **never gated by distance**. FSM transitions are driven by **events** only.  
- **Avoidance gating**: When `/avoid_start` fires, controllers should prioritize the avoidance path queue; resume base stream on `/avoid_done`.  
- **Segment switching**: Prefer **index‑based** transitions in sender; optional ROI heuristics may assist but must not depend on noisy distance thresholds.

---

## 7) Global Parameters

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `output_frame_id` | string | `map` | Global path frame (static track). Use `base_link` only for short local segments. |
| `timer_hz` | float | 2.0 | HelperPathSender streaming rate. |
| `window_size` | int | 4 | Sliding window for path streaming. |
| `events_transient_local` | bool | true | Make events survive restarts. |
| `use_sim_time` | bool | false | Set `true` for bag replay. |
| `drift_e_thresh_m` | float | 0.25 | Centerline vs SLAM cross‑track error threshold. |



---

## 8) Logging & Monitoring

- **Run manifest** (`logs/run_meta.yaml`): record clock source, QoS choices, frames, package versions.  
- **Drift** (`logs/slam_drift.csv`): timestamp, e_perp, decision, pose source.  
- **Streaming**: segment index, published count, timer periods.  
- **Events**: `/stop`, `/pickup_dropoff`, `/avoid_start`, `/avoid_done` with timestamps.

---

## 9) Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| FSM/event jitter | Legacy reach‑trigger logic still in use | Remove distance gating; events only. |
| Boundary hand‑off glitch | ID‑local assumptions | Merge current+next path for transitions; use ROI assist. |
| Drift accumulates | Time/TF inconsistency | Enforce single clock; monitor centerline vs pose; provide reset hooks. |
| Missed events after restarts | Volatile QoS | Use `transient_local` for critical events. |
| Mixed Docker/host time | Multiple `/clock` providers | Keep a single `/clock`; document offsets in run manifest. |

---

## 10) Version Matrix

| Component | Tested Version |
|---|---|
| ROS 2 | Humble |
| RealSense | librealsense 2.x (aligned depth enabled) |
| PyTorch (YOLOv7 optional) | 1.12.x (Jetson FP16) |
| Cartographer | cartographer_ros latest for Humble |
| Simulink ROS 2 toolbox | slros2 (matching Humble msgs) |


---

