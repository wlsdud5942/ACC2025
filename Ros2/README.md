# ROS2 README

End-to-end ROS 2 (Humble) stack that connects **Perception (SCNN / YOLOv7)**, **Planning (RRT / DQN / Path Sender)**, **Control (FSM; Simulink codegen)**, **Obstacle Avoidance**, and **SLAM (Cartographer)** on the real vehicle (QCar2).  
The design emphasizes **timing robustness**, **deterministic handshakes**, and **simple, well-documented topic contracts**.

---

## 0) Goals & Design Constraints

**Goals**
- Build a **reproducible** on-vehicle pipeline across perception → planning → control → avoidance/SLAM.
- Keep synchronization between Python ROS 2 nodes and **Simulink-generated C++ FSM** **stable and observable**.
- Stream waypoints **without distance-based reach checks**; use **explicit events** for stop/pickup and other transitions.

**Constraints**
- Minimize event/timing mismatch with the Simulink FSM.
- Avoid localization-dependent reach triggers (noisy & laggy).
- Run reliably in **mixed Docker/host** deployments; keep TF frames & time discipline consistent.

---

## 1) Problem Definition → Hypotheses

**Observed problems**
1. **FSM desync**: Distance-based “reach triggers” caused ordering/latency mismatches with the Simulink FSM.  
2. **Boundary fragility**: At path-ID boundaries, perception/SLAM/planning events did not align → failed hand-offs.  
3. **SLAM drift**: Cartographer accumulates small drift over long runs.

**Hypotheses**
- **(G1)** Stream waypoints strictly on a **timer** and drive FSM transitions with **explicit event topics** → fewer desyncs.  
- **(G2)** Treat **transitions** as **continuous curves** (avoid artificial ID boundaries) in perception/avoidance/planning.  
- **(G3)** Compare **SCNN 3D centerline** to **SLAM pose** to monitor and mitigate long-term drift.

---

## 2) Attempts → Failures → Improvements

- **V0 — Distance-based “reach trigger” (failed)**  
  Switch at segment endpoints using pose distance.  
  → Result: FSM event ordering jitter; misbehavior under latency.

- **V1 — Time-based streaming (success)**  
  `HelperPathSender` streams `/path_x`, `/path_y` on a fixed timer; FSM changes state using explicit events.  
  → Result: Synchronization & repeatability improved markedly.

- **V1.1 — SCNN-ROI assisted switching (optional)**  
  `PathSenderNode` can use SCNN centerline ROI for next-segment switching (Cartographer ROI as backup).  
  → Result: More stable transitions at boundaries.

- **V2 — Cartographer integration & drift monitoring**  
  Unified TF/time across Docker and host. Compare SCNN centerline vs SLAM pose; add alert/reset hooks.  
  → Result: Early drift detection and simpler recovery.

---

## 3) Current Architecture (Overview)

- **Perception**
  - **SCNN**: lane mask → **centerline (3D)**.
  - **YOLOv7**: objects/stop-line → `/obstacle_info`, `/yolo_stop`.

- **Planning**
  - **Offline**: `rrt/` → `waypoints_*.json`; `dqn/` → `dqn_paths.json`.  
  - **Online**:  
    - `HelperPathSender`: time-based waypoint streaming from **DQN total sequence**.  
    - `PathSenderNode`: SCNN-ROI–assisted segment switching (alternative).

- **Obstacle Avoidance**
  - Lateral **offset-curve** over **merged (current+next) path**; gates base stream while active.  
  - Emits `/avoid_start`, `/avoid_done`; streams temporary avoidance waypoints.

- **Control (FSM)**
  - Simulink **code-generated C++** node; consumes `/stop`, `/pickup_dropoff`, `/path_mode` and follows streamed waypoints.

- **SLAM**
  - **Cartographer 2D** (LiDAR + IMU), `use_sim_time`, custom TF broadcaster for `base_link ↔ laser`.

> Optional, include this in the repo as `mermaid`:

```mermaid
flowchart LR
  subgraph Perception
    A[SCNN] -->|/centerline_3d| P1
    B[YOLOv7] -->|/obstacle_info, /yolo_stop| OA
  end
  subgraph Planning
    RRT[RRT → waypoints_*.json]
    DQN[DQN → dqn_paths.json]
    P1[HelperPathSender / PathSenderNode]
    RRT --> P1
    DQN --> P1
  end
  subgraph Avoidance
    OA[Obstacle Avoidance] -->|/avoid_start, /avoid_done| P1
    P1 -->|/path_x, /path_y| CTRL
    OA -->|/path_x, /path_y| CTRL
  end
  subgraph Control
    CTRL[Simulink FSM (C++)] -->|/cmd| Vehicle
    P1 -->|/path_mode, /stop, /pickup_dropoff| CTRL
  end
  subgraph SLAM
    SL[Cartographer] -->|/tf, /pose| P1
  end
```

---

## 4) Interface Contracts (Summary)

| Module                | Inputs                                     | Outputs                                                                | Notes |
|-----------------------|---------------------------------------------|------------------------------------------------------------------------|-------|
| **Perception**        | `/camera/*`                                 | `/lane_mask`, `/centerline_3d`, `/obstacle_info`, `/yolo_stop`         | Keep `frame_id` / `stamp` consistent |
| **HelperPathSender**  | `/dqn_done`, `/location`                    | `/path_x`, `/path_y`, `/path_mode`, `/stop`, `/pickup_dropoff`         | **No reach checks** (time-based streaming) |
| **PathSenderNode**    | `/location`, `/lane_detection/*`            | `/path_x`, `/path_y`, `/path_mode`                                     | Uses SCNN ROI–assisted switching |
| **ObstacleAvoidance** | `/obstacle_info`, `/location`               | `/path_x`, `/path_y`, `/avoid_start`, `/avoid_done`                    | **Gate** base stream while avoidance is active |
| **Control (FSM)**     | `/path_x`, `/path_y`, `/stop`, `/pickup_dropoff` | `/cmd`                                                              | Simulink C++ codegen node |

**QoS guidance**
- **Sensors/images**: `SensorDataQoS` (best effort, low latency).  
- **Control/events**: `reliable` + `keep_last(N)`; consider **`transient_local`** for events that must survive restarts.  
- **`/tf_static`**: `reliable` + `transient_local`.

---

## 5) Simulink → ROS 2 Codegen (Strategy)

**Why codegen?** Speed of tuning + reproducibility with a deterministic controller.

**How**
1. Build the controller FSM in Simulink with **ROS 2 Pub/Sub** blocks.  
2. Generate a C++ node via **`slros2`** toolbox.  
3. Add to the ROS 2 workspace; `colcon build`.

**Key lesson**  
Remove distance-based reach logic from Simulink. Make transitions **event-driven** only (`/stop`, `/pickup_dropoff`, `/path_mode`). Let ROS 2 nodes **stream** waypoints on a timer.

---

## 6) Time Sync & TF Discipline

- Use **`use_sim_time=true`** consistently when `/clock` is present.  
- For **Docker ↔ host** mixes, run a **TF broadcaster** to pin frames & timestamps (e.g., publish `base_link ↔ laser`).  
- Publish **`/tf_static`** with **`transient_local + reliable`** so restarts recover geometry.  
- Prefer **hardware timestamps**; otherwise normalize to a single clock and document offsets.

---

## 7) Launch & Operations

**Bringup (skeleton)**
```bash
# Perception
ros2 launch perception realsense_scnn_yolov7.launch.py

# SLAM
ros2 launch cartographer_ros cartographer.launch.py use_sim_time:=true

# Planning
ros2 run planning helper_path_sender     # or: ros2 run planning path_sender_node

# Obstacle Avoidance
ros2 run obstacle_avoidance waypoint_generator

# Control (Simulink codegen)
ros2 run control fsm_node
```

**Operational checklist**
- [ ] `dqn_paths.json` present; `HelperPathSender` streams **without** reach gating.  
- [ ] `/path_mode`, `/stop`, `/pickup_dropoff` are **events**, not distance-derived in control.  
- [ ] Avoidance merges **current+next** path and **gates** base stream while active.  
- [ ] `/tf` tree stable; `/tf_static` persists across restarts.  
- [ ] SCNN centerline vs SLAM pose **drift monitor** active (thresholds & alerts logged).

---

## 8) Logging & Monitoring

- **Drift**: lateral error between **centerline** and **SLAM pose** → `logs/slam_drift.csv`.  
- **Streaming**: segment indices, timestamps, count of published points.  
- **Events**: audit trail for `/stop`, `/pickup_dropoff`, `/avoid_start`, `/avoid_done`.  
- **Run manifest**: capture QoS profiles & clock source at startup → `logs/run_meta.yaml`.

---

## 9) Directory Layout (suggested)

```
ros2/
├─ README.md
├─ perception/
├─ planning/
│  ├─ rrt/                      # offline segment generation
│  ├─ dqn/                      # visit-order planner
│  └─ nodes/
│     ├─ helper_path_sender.py
│     └─ path_sender_node.py
├─ obstacle_avoidance/
│  └─ waypoint_generator.py
├─ control/                     # Simulink codegen package
└─ slam/
   ├─ cartographer_configs/
   └─ tf_broadcaster/
```

---

## 10) Known Issues & Mitigations

| Issue                           | Mitigation |
|---------------------------------|------------|
| FSM/event jitter (legacy reach-trigger) | Use **time-based streaming**; drive FSM via **explicit events** only. |
| Boundary hand-off glitches      | Operate on **merged (current+next) paths**; avoid ID-local assumptions. |
| Cartographer drift              | Monitor **centerline vs pose**; provide alert/reset hooks; maintain time/TF discipline. |
| Missed events after restarts    | Use **`transient_local`** QoS for critical events; small `keep_last(N)` buffers. |
| Mixed Docker/host timestamps    | Single source of truth for `/clock`; normalize stamps at bridges; verify `frame_id` consistency. |


---
