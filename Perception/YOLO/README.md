# YOLOv7-based Object Detection
This module performs **real-time road-object detection**—stop lines, crosswalks, traffic signage—using **YOLOv7** and fuses results with **Depth** to recover **3D positions**. We adopted **v7** instead of v8 due to Jetson compatibility issues (custom C++ ops). Outputs feed directly into the FSM for stop/avoid decisions and into Planning for obstacle-aware behavior.

---

## 1) Objectives
- Emit **FSM events** (e.g., `/yolo_stop`) when specific objects are detected with high confidence.
- Provide **3D positions** (meters) for detections so that avoidance and stopping logic can operate in physical space.

---

## 2) Model Choice & Runtime Environment
- **Why YOLOv7 (not v8)**: v8 required ops that conflicted with our Jetson stack; v7 ran reliably after dependency cleanup.
- **Repo customization**: trimmed visualization and heavy dataloaders; added a **ROS2 node** interface.
- **Typical Jetson setup (example)**: Python 3.8, CUDA 11.4, cuDNN compatible, PyTorch 1.12.x.
- **Inference knobs**: image size 640, FP16 (half precision) where stable, NMS tuned per class.

---

## 3) Custom Training & Classes
- **Data**: in-house captures + selected open datasets to reduce domain gap (lighting, paint wear, camera height).
- **Classes** (example): `stop`, `crosswalk`, `red_light`.
- **Training example**: epochs = 100, img size = 640, batch = 16, model = `yolov7-tiny`.
- **Tuning philosophy**: favor **precision** over recall for critical classes (minimize false stop-line triggers).
- **Recommended augmentations**: brightness/contrast jitter, motion blur (mild), random crop/scale, hue shift (small).
- **Artifacts to store**: `weights/best.pt`, class map, normalization settings, and a metrics summary (mAP / precision by class).

---

## 4) ROS2 Integration

**Subscriptions**
- `/camera/color/image_raw` (`sensor_msgs/Image`, BGR8). Use the camera frame’s `stamp`/`frame_id`.

**Publications**
- `/obstacle_info` (`std_msgs/Float32MultiArray`): per detection → `[x, y, z, cls_id, conf]` in meters (define and keep a consistent frame, e.g., `base_link`).
- `/yolo_stop` (`std_msgs/Int32`): `0/1` event for the FSM when a stop line is confidently detected.

**QoS**
- Images: `SensorDataQoS` or `best_effort`.
- Events/3D outputs: `reliable`, `keep_last(10)`.

**Per-class thresholds (example)**
- stop: `conf ≥ 0.6`, `NMS IoU ≤ 0.45`
- crosswalk: `conf ≥ 0.5`
- red_light: `conf ≥ 0.5`

---

## 5) 3D Localization with Depth

**Depth source**
- `/camera/depth/image_raw` (`16UC1` or `32FC1`, meters). Align with the color stream.

**Robust depth estimate**
- Take a **k×k windowed median** around the BBox center (k = 3–7) to reduce NaNs/spikes.
- Clamp valid range (e.g., 0.3–6.0 m). If median invalid, expand window or fall back to nearest valid pixel within radius r.

**Projection**
- From pixel `(u, v)` and depth `Z`, compute camera-frame point:
  - `X = (u − cx) / fx × Z`, `Y = (v − cy) / fy × Z`.
- Transform `(X, Y, Z)` to **`base_link`** using the calibrated extrinsics (`T_base_link^camera`) before publishing.

**Contract**
- `/obstacle_info` can concatenate multiple detections:
  - `[x1,y1,z1,cls1,conf1, x2,y2,z2,cls2,conf2, …]`
  - Always document which frame you use (`camera` or `base_link`) and keep it consistent.

---

## 6) Processing Flow (Runtime)

1. Receive an RGB frame.
2. Run YOLOv7 inference → BBoxes, classes, confidences.
3. For each detection, pull a windowed median depth around the BBox center.
4. Project to 3D and transform to `base_link`.
5. Publish `/obstacle_info`. If a **stop** is confidently detected and within the relevant ROI/range, publish `/yolo_stop = 1`.
6. Control FSM transitions based on these events.

---

## 7) Troubleshooting & Tuning

Issue | Mitigation
--- | ---
YOLOv8 not runnable on Jetson | Use **YOLOv7**; pin CUDA/cuDNN/PyTorch versions that match Jetson image.
Depth at center is NaN | Windowed median, expand window adaptively; fallback to nearest valid depth within radius.
Stop vs lane confusion | Add diverse training samples; post-process with lane mask geometry checks (e.g., alignment angle).
Over-detections (signage) | Raise class-specific `conf` and tighten NMS; restrict to **ROI** where objects are plausible.
Jittery 3D | Temporal median of depth; reject outliers by speed/acceleration bounds in `base_link`.

---

## 8) Message & File Layout
- Topics
  - `/obstacle_info`: Float32MultiArray, concatenated detections in meters (document frame).
  - `/yolo_stop`: Int32, 0/1 event.

- Directory
  - `yolov7/`
    - `README.md`
    - `weights/best.pt`
    - `config/thresholds.yaml` (per-class `conf`/NMS/ROI)
    - `nodes/yolov7_node.py` (ROS2 node)
    - `utils/` (preprocess, depth utils)

---
