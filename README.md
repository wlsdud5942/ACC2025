# ACC2025

<img src="images/kdasmain.gif" alt="KDAS" width="800"/>

---

This repo tracks our autonomous driving pipeline that started in the **Quanser QCar simulator** and transferred to **QCar2**. Every module includes **failures and iteration logs**, not just final results.

> Scope: All contracts match the **ACC / Quanser** setup we actually ran.

---

## Operating Principles

- **Reproducibility** – Document code, configs, and data flows for repeatable runs.  
- **Layered Integration** – *Perception → Planning → Avoidance → Control → ROS 2*, with explicit topic contracts.  
- **Real-Vehicle Validity** – After simulation passes, repeat on QCar2; prioritize SLAM drift, time sync, and actuator deadband.

---

## Repository Layout

| Directory | Role | Key Sub-Docs |
|---|---|---|
| `perception/` | SCNN lanes + YOLOv7 objects (RGB-D fusion) | `perception/README.md`, `perception/scnn/README.md`, `perception/yolov7/README.md` |
| `planning/` | Offline RRT waypoints + DQN visit/order | `planning/README.md`, `planning/rrt/README.md`, `planning/dqn/README.md` |
| `Obstacle_Avoidance/` | Offset-curve avoidance over merged segments | `Obstacle_Avoidance/README.md` |
| `ros2_system/` | ROS 2 wiring, time/TF sync, Simulink codegen | `ros2_system/README.md`, `ros2_system/data_helper/README.md`, `ros2_system/cartographer/README.md` |
| `control/` | **Pure Pursuit + FSM** (no PID, LPF removed) | `control/README.md` |
| `hardware_setup/` | Jetson, RealSense, LiDAR, ESC wiring | `hardware_setup/README.md` |
| `simulator_to_real/` | Transfer issues & fixes | `simulator_to_real/README.md` |
| `common_utils/` | CSV/JSON/logging/visualization tools | `common_utils/README.md` |
| `docs/` | Reports / posters / design notes | `docs/README.md` |

---

## System Overview (Summary)

### Sensors & Platform
- **RealSense D435i** (RGB + Depth) for perception  
- **RPLIDAR A2M8** for Cartographer SLAM  
- **Jetson AGX Orin** onboard compute  
- **ESC/Servo**: MKSESC 75100 V2, Traxxas battery

### Perception
- **SCNN** (640×480) → binary mask → **DBSCAN + 3rd-order polyfit** → lane centerline  
- Depth → **3D centerline** (meters)  
- **YOLOv7** (stop line / crosswalk / signs) – chosen over v8 for Jetson stability  
- Cross-check **3D centerline vs SLAM pose** to monitor drift

### Planning
- **RRT**: multi-run per segment; select by length/curvature/continuity  
- **DQN**: optimize visit order → **`dqn_paths.json`**  
- **Execution**: `helper_path_sender` **streams** `/path_x`, `/path_y` on a timer (**no arrival checks**); FSM uses separate events

### Obstacle Avoidance
- Merge **current + next** segment; build **offset candidates**  
- Choose minimal-clearance path via **point-to-segment** distance; **smooth** and rejoin base path

### Control
- **Steering**: **Pure Pursuit** with **mode-scheduled** lookahead/speed from `/path_mode`  
- **Speed**: Event-centric **Stateflow/FSM** (stop/start/pickup-dropoff).  
- **Controller output**: **`/simulinkOut`** (`geometry_msgs/Point`) – **x = v_cmd [m/s]**, **y = steer [rad]**, **z = debug**  
  (A separate *driver* node maps this to servo/ESC commands.)

### ROS 2 Integration
- **`data_helper`**: Perception → FSM (`/stop`, `/pickup_dropoff`, `/path_mode`)  
- **Cartographer**: 2D LiDAR+IMU; TF/time-sync fixes; drift check vs SCNN centerline  
- **Simulink Codegen**: Control compiled to **ROS 2 C++** with the contracts above

---

## Interfaces (Canonical Contracts)

**Planning → Control**
- `/path_x`, `/path_y` — `std_msgs/Float32` (streamed waypoints)  
- `/path_mode` — `std_msgs/Int32` (0: straight, 1: curve, 2: start/finish)  
- `/stop` — `std_msgs/Int32` (0/1)  
- `/location` — `geometry_msgs/Point` (**x, y, z = yaw [rad]**)  
- `/sim_time` — `std_msgs/Float64` (optional)

**Control → Driver**
- `/simulinkOut` — `geometry_msgs/Point` (**x = v_cmd**, **y = steer**, **z = debug**)

**Perception / Avoidance**
- `/lane_mask`, `/centerline_3d`, `/obstacle_info`, `/yolo_stop`  
- `/avoid_start`, `/avoid_done` (when avoidance overrides base path)

---

## Quick Start

### 1) Build
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2) Generate the DQN route
```bash
# Example: publish stops
ros2 topic pub /dqn_path_input std_msgs/Int32MultiArray "{data: [4, 7, 21, 33]}"
```
This creates `dqn_paths.json` and publishes `/dqn_done: True`.

### 3) Stream waypoints
```bash
# After /dqn_done == True
ros2 run path_planning helper_path_sender
```
`helper_path_sender` streams `/path_x`, `/path_y` from `waypoints_*.json` in order.  
The controller publishes **`/simulinkOut`** continuously.

> Launch/QoS/TF details: see `ros2_system/README.md` and each module’s README.

---

## Major Issues & Solutions

- **YOLOv8 → YOLOv7 (Jetson compat):** v8 deps/runtime unstable → moved to **v7**; tuned class-wise conf/NMS.  
- **FSM/event sync:** Distance “arrival” caused skew → **time-based streaming** + explicit FSM events.  
- **SLAM drift:** Compare **SCNN 3D centerline vs SLAM pose**; log and relocalize/reset on threshold.  
- **Avoidance at boundaries:** Merge segments and optimize offset over the **full span**; smooth rejoin.  
- **Filter duplication (real car):** Embedded filters + controller LPFs attenuated commands → **removed controller LPFs**; keep steering saturation & STOP chart.  
- **Actuator deadband & calibration:** Measured servo/ESC deadbands; calibrated **steer offset/gain** in the driver node.

---

## Recommended Reading Order

1. `perception/README.md` – RGB-D fusion (SCNN + YOLOv7) with failure logs  
2. `planning/README.md` → `planning/rrt/`, `planning/dqn/` – Offline waypoints + order optimization; streaming `helper_path_sender` design  
3. `Obstacle_Avoidance/README.md` – Offset curves, clearance checks, smoothing  
4. `ros2_system/README.md` – `data_helper`, Cartographer, codegen, QoS/TF/time-sync  
5. `control/README.md` – Why **PP + FSM**, scheduling, and real-vehicle fixes

---

## Tech Stack

- **ROS 2 Humble**, Python 3.10  
- **RealSense SDK**, **RPLIDAR SDK**  
- **SCNN** (PyTorch training + custom inference), **YOLOv7**  
- **MATLAB/Simulink R2024b** codegen → ROS 2 **C++** node  
- **Docker** (optional isolation for SLAM/Perception)

---

## Achievements

- **American Control Conference Student Challenge 2025 — Finalist**  
- **Kookmin University Self-Driving Competition 2025 — Finalist**  
- Delivered a real-vehicle pipeline: **RGB-D perception**, **SLAM cross-checks**, and **streaming planning→control integration**

---

## License & Citation

- Code intended for academic research in ACC 2025; see per-folder licenses if present.  
- If you use this repo, please cite it and the underlying SCNN/YOLO/Cartographer works.
