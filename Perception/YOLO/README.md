# YOLOv7-based Object Detection — Process-Centric README (Revised)

This module performs **real-time road-object detection**—stop lines, crosswalks, traffic signage—using **YOLOv7** and fuses results with **Depth** to recover **3D positions**. We adopted **YOLOv7** instead of v8 due to Jetson compatibility issues (custom C++ ops). Outputs feed the **FSM** (e.g., stop events) and **Planning** (obstacle-aware behavior).

---

## 1) Objectives
- Emit **FSM events** (e.g., `/yolo_stop`) when specific objects are detected with high confidence and within a valid range/ROI.
- Provide **3D positions** (meters) for detections so that avoidance and stopping logic can operate in physical space.

---

## 2) Model Choice & Runtime Environment

**Why YOLOv7 (not v8)**
- YOLOv8 required ops that conflicted with our Jetson stack; YOLOv7 ran reliably after dependency cleanup and FP16.

**Repo customization**
- Trimmed visualization/dataloaders; added a **ROS2 node** interface and depth fusion utilities.

**Typical Jetson setup (example)**
- Python 3.8, CUDA 11.4, cuDNN compatible, PyTorch 1.12.x.
- TensorRT is optional (not required for the baseline), FP16 inference recommended.

**Inference knobs**
- Image size: 640 (square letterbox).
- Precision: FP16 where stable.
- NMS: per-class IoU/score thresholds.

---

## 3) ROS2 Integration

**Frames**
- Camera optical frame: `camera_color_optical_frame` (RealSense convention x→right, y→down, z→forward).
- Vehicle: `base_link` (x→forward, y→left, z→up).
- Outputs default to **`base_link`** unless `output_frame_id` is overridden.

**Subscriptions**
- `/camera/color/image_raw` — `sensor_msgs/Image` (`rgb8`/`bgr8`), header stamp/frame kept.
- `/camera/aligned_depth_to_color/image_raw` (preferred) or `/camera/depth/image_raw` — `sensor_msgs/Image` (`16UC1` in mm or `32FC1` in m).

**Publications**
- **Legacy** `/objects_3d` — `std_msgs/Float32MultiArray` in meters, concatenated as `[x,y,z,cls_id,conf] × N`, `header.frame_id = output_frame_id` (documented below).
- **Standard (optional)** `/detections_3d` — `vision_msgs/Detection3DArray` (each `Detection3D` holds pose/size/class/score).
- `/yolo_stop` — `std_msgs/Int32` (0/1) stop event after debounce.

**QoS**
- Images: **SensorData QoS** (best_effort, keep_last(5)).
- Control-relevant streams (3D objects, events): **Reliable**, keep_last(10).

**Per-class thresholds (example)**
- `stop`: conf ≥ **0.60**, NMS IoU ≤ **0.45**.
- `crosswalk`: conf ≥ **0.50**.
- `red_light`: conf ≥ **0.50**.

---

## 4) Depth Fusion & 3D Localization

**Depth source**
- Use **aligned depth** to color when available so RGB pixel `(u, v)` and depth share the same frame.

**Robust depth estimate**
- Compute a **k×k windowed median** at the BBox **center** (k∈{3,5,7,9}).
- Reject samples outside `[z_min, z_max]` (default 0.3–6.0 m) before median.
- If invalid, expand the window or fall back to nearest-valid depth within radius `r`.

**Projection (camera → base_link)**
Given intrinsics `(fx, fy, cx, cy)` and depth `Z` (m):
```
Xc = (u - cx)/fx * Z
Yc = (v - cy)/fy * Z
Zc = Z
```
Transform to output frame using TF:
```
p_out = T_output_camera ⋅ [Xc, Yc, Zc, 1]^T
```
Publish in meters; keep `header.frame_id = output_frame_id`.

**Contract (legacy)**
- `/objects_3d` concatenates detections: `[x1,y1,z1,cls1,conf1, x2,y2,z2,cls2,conf2, …]`.
- If using `vision_msgs/Detection3DArray`, include covariance/size when meaningful.

---

## 5) /yolo_stop Event Logic

- **Class**: `stop` (or `stop_line` depending on dataset mapping).
- **Confidence debounce**: require `N_stop` consecutive frames with `conf ≥ conf_stop`.
- **Range gating**: only trigger if `z ∈ [z_stop_min, z_stop_max]` (e.g., 0.5–8.0 m).
- **ROI gating**: ensure the BBox center lies within a configurable lower image band (e.g., bottom 40%).
- **Cooldown**: after triggering `1`, enforce `cooldown_frames` before the next trigger.

This avoids false positives from distant textures or paint artifacts.

---

## 6) Processing Flow 

1. Receive synchronized RGB + (aligned) depth.
2. Run YOLOv7 → BBoxes/classes/conf.
3. For each detection, median-filter depth at BBox center and project to 3D.
4. Transform to `output_frame_id` (default `base_link`).
5. Publish `/objects_3d` (or `/detections_3d`).
6. Apply event logic; if valid, publish `/yolo_stop = 1` (else `0`).

---

## 7) Parameters

| Group | Parameter | Type | Default | Notes |
|---|---|---|---:|---|
| general | `output_frame_id` | string | `base_link` | Frame for 3D outputs/events. |
| sync | `sync_slop_ms` | int | 40 | ApproximateTime tolerance. |
| depth | `use_aligned_depth` | bool | true | Prefer aligned depth image. |
| depth | `z_min` | float | 0.3 | Min valid depth (m). |
| depth | `z_max` | float | 6.0 | Max valid depth (m). |
| depth | `median_k` | int | 9 | Window size for depth median (odd). |
| model | `img_size` | int | 640 | Letterbox input size. |
| model | `conf_thresh` | float | 0.25 | Base confidence threshold. |
| model | `nms_iou` | float | 0.45 | NMS IoU. |
| model | `use_fp16` | bool | true | Enable half precision on Jetson. |
| stop | `conf_stop` | float | 0.60 | Stop-line confidence threshold. |
| stop | `N_stop` | int | 3 | Required consecutive frames. |
| stop | `z_stop_min` | float | 0.5 | Min stop-line range (m). |
| stop | `z_stop_max` | float | 8.0 | Max stop-line range (m). |
| stop | `roi_band_bottom` | float | 0.4 | Fraction of image height used for ROI (bottom band). |
| stop | `cooldown_frames` | int | 10 | Frames to ignore after trigger. |


---

## 8) Troubleshooting & Tuning

| Issue | Mitigation |
|---|---|
| YOLOv8 not runnable on Jetson | Pin CUDA/cuDNN/PyTorch; use YOLOv7; avoid custom ops. |
| Depth at center is NaN | Windowed median; expand window; nearest-valid fallback within radius. |
| Stop vs lane confusion | Add diverse samples; overlay lane mask constraint; restrict ROI. |
| Over-detections (signage) | Raise class-specific `conf`; tighten NMS; limit plausible ROIs. |
| Jittery 3D | Temporal median; clamp velocity/acceleration of detected points in `base_link`. |
| Latency spikes | Co-locate nodes (composition); enable intra-process comms; set Jetson to max perf. |

---

## 9) Testing & RViz Recipe

1. Record RGB + aligned depth, TF, `/objects_3d`, `/yolo_stop`.
2. RViz: add **Image** (RGB), **Image** (depth), **TF**, **Marker/MarkerArray** for 3D points, and text for classes/conf.
3. Validate: frame alignment (`frame_id` consistency), depth units (meters), event timing vs ground truth.
4. Bag replay: outputs should be deterministic from the recorded inputs.

---


## 10) Message Contracts

### `/objects_3d` (legacy — `std_msgs/Float32MultiArray`)
- `data = [x1, y1, z1, cls1, conf1, x2, y2, z2, cls2, conf2, ...]` in **meters**.
- `header.frame_id = output_frame_id` (default `base_link`).

### `/detections_3d` (optional — `vision_msgs/Detection3DArray`)
- Each `Detection3D` includes `results[0].hypothesis.class_id`, `results[0].hypothesis.score`, and `bbox.center`/`size`.

### `/yolo_stop` (`std_msgs/Int32`)
- `data ∈ {0,1}` after debounce + range/ROI gating.

---

## 11) Notes
- Keep **frame_id** consistent across outputs derived from the same image/depth pair.
- Depth units: convert `16UC1` (mm) to meters before projection.
- If you later change RealSense resolution, re-measure intrinsics `(fx, fy, cx, cy)`.
- Document exact **class map** and any ROI rules in `config/thresholds.yaml` to avoid training/production drift.
