# Perception Module

This directory documents the **full perception stack** for our autonomous vehicle. The stack consists of **SCNN-based lane detection** and **YOLOv7-based object detection**, driven by **Intel RealSense D435i RGB + Depth**. All outputs are expressed in **metric 3D coordinates** and consumed by **Planning / Control / ROS2** for path generation, FSM transitions, and avoidance decisions.

---


## 1) Goals & Scope
- Detect a **3D lane centerline** and **road objects** in real time and convert them to **actionable metric units (meters)**.
- Provide lane, stop line, crosswalk, and sign detections under **consistent QoS & timestamping**.
- Compare the **SCNN-derived 3D centerline** against **Cartographer SLAM** poses to monitor and mitigate **long-term drift**.

---

## 2) Frames & Calibration

**Frames**
- **Camera (RealSense optical)**: x → right, y → down, z → forward (RealSense SDK convention).
- **Vehicle (base_link)**: x → forward, y → left, z → up.
- **Extrinsics**: fixed transform \(T^{\text{camera}}_{\text{base\_link}}\) broadcast via TF (`static_transform_publisher`).

**Calibration workflow**
1. Estimate intrinsics/extrinsics with a checkerboard (OpenCV).
2. Register the static transform via `static_transform_publisher`.
3. Verify alignment in RViz (image/depth overlay, project landmarks).

**Intrinsics (example placeholders)**
- `fx, fy, cx, cy` (replace with measured values; keep for the exact resolution used, e.g., 640×480).

**Recommended storage**
- `perception/calib/realsense_intrinsics.yaml` (pinhole + distortion), `perception/calib/T_base_link_camera.yaml` (extrinsics).

**Depth alignment**
- Prefer using the **aligned depth to color** topic from RealSense (or enable alignment in the driver), so that RGB pixel `(u, v)` and depth share the same frame.

---

## 3) Sensors & Topics (contract)

| Device / Topic | Type | Frame | QoS | Contract / Notes |
|---|---|---|---|---|
| RealSense D435i (RGB + Depth) | — | `camera_*` | — | 640×480@30Hz typical. Prefer hardware timestamps. |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData (best_effort) | Encoding: `rgb8` or `bgr8`. |
| `/camera/aligned_depth_to_color/image_raw` (preferred) or `/camera/depth/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` (if aligned) | SensorData (best_effort) | Types: `16UC1` (mm) or `32FC1` (m). Convert to meters internally. |
| `/lane_mask` | `sensor_msgs/Image` | `camera_color_optical_frame` | Reliable, depth=5 | SCNN binary mask (0/255), 640×480. |
| `/centerline_3d` (legacy) | `std_msgs/Float32MultiArray` | `base_link` | Reliable, depth=10 | Flat `[x1,y1,z1, ...]` meters. |
| `/centerline_path` (standard) | `nav_msgs/Path` | `base_link` or `map` | Reliable, depth=5 | Path poses sampled along the 3D centerline. |
| `/objects_3d` (legacy) | `std_msgs/Float32MultiArray` | `base_link` | Reliable, depth=10 | `[x,y,z,cls_id,conf] × N`. |
| `/detections_3d` (optional standard) | `vision_msgs/Detection3DArray` | `base_link` | Reliable, depth=5 | 3D boxes/points with class+score. |
| `/yolo_stop` | `std_msgs/Int32` | — | Reliable, depth=1 | 0/1 event for stop-line detection. |

**Timestamping**
- Use the **camera’s header.stamp** for all derived products (mask, centerline, detections) to preserve causality. If processing delays exist, add a `processing_latency_ms` field in diagnostics.

---

## 4) Time Synchronization Policy

- Prefer **ApproximateTimeSynchronizer** (or ROS2 C++ equivalent) on `{RGB, aligned_depth}` for SCNN/YOLO consumers.
- If composition nodes are used, ensure a single **clock domain**. Configure RealSense driver to publish **hardware timestamps** and set `use_sim_time=false` on the robot.
- When replaying rosbag2, set `--clock` appropriately to keep TF and images aligned.

---

## 5) SCNN-based Lane Detection

**Inputs / Outputs**
- Input: 640×480 RGB (aligned depth available).
- Outputs:
  1) `/lane_mask`: binary mask (0/255).
  2) `/centerline_path` (`nav_msgs/Path`) **and** legacy `/centerline_3d` (flat array).

**Post-processing (image domain)**
1. **Row-wise clustering** (DBSCAN) to group lane pixels into left/right lanes.
2. **3rd-order polyfit** per lane.
3. **Centerline**: average the two fits, constrained to a valid y-range (bottom 60–90% of the image).
4. **Outlier rejection**: RANSAC or median-absolute-deviation per row for robustness.

**Depth fusion (camera → base_link)**
Given pixel `(u, v)` and depth `Z` (meters):
\[
X = \frac{(u - cx)}{fx} Z, \quad Y = \frac{(v - cy)}{fy} Z, \quad Z = Z
\]
Pack `(X,Y,Z)` as camera-frame coordinates, then transform to `base_link` with TF.

**Smoothing and sampling**
- Sample N points along the image-space centerline, back-project each via depth.
- Fit a short cubic spline in 3D, then resample at fixed arc-length for `/centerline_path`.

**Drift monitoring (vs Cartographer)**
- Compute cross-track error \(e_\perp\) between the robot pose (from SLAM) and the 3D centerline at a **lookahead distance** `L` meters.
- Maintain a sliding window mean/variance; emit a diagnostic if `mean(|e_⊥|)` exceeds a threshold for `T` seconds.

See also: `perception/scnn/README.md`.

---

## 6) YOLOv7-based Object Detection

**Rationale**
- Jetson compatibility issues with YOLOv8 (certain ops) → choose **YOLOv7** with tuned dependencies and FP16.

**Classes (project-dependent)**
- `stop_line`, `crosswalk`, `sign` (extend as needed).

**Depth extraction**
- Use **windowed median** around the BBox **center** (e.g., 9×9 or area-proportional window). Reject outliers outside `[z_min, z_max]` (e.g., 0.3–6.0 m). If missing, expand window or mark as invalid.

**Outputs**
- Legacy `/objects_3d`: `[x, y, z, cls_id, conf] × N` in **meters** in `base_link` (or camera if chosen consistently).
- Standard `/detections_3d` (optional): `vision_msgs/Detection3DArray` with per-object pose and size.
- `/yolo_stop`: publish `1` when a `stop_line` class is detected with `conf ≥ conf_stop` for `N_stop` consecutive frames.

See also: `perception/yolov7/README.md`.

---

## 7) 3D Fusion & Coordinate Conversion

- For any 2D detection `(u, v)`, fetch aligned depth, compute camera 3D `(X, Y, Z)`, then transform via TF to `base_link`.
- Ensure TF tree includes `map → odom → base_link → camera_*_optical_frame` (place the camera under `base_link`).
- If output in `map` is required (for global logging), apply two transforms: camera→base_link, then base_link→map using the SLAM pose.

---

## 8) Parameters

### 8.1 Global
| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `use_aligned_depth` | bool | true | Use `/aligned_depth_to_color` if available. |
| `output_frame_id` | string | `base_link` | Frame for `/centerline_path` and 3D objects. |
| `sync_slop_ms` | int | 40 | ApproximateTime sync tolerance. |
| `z_min` | float | 0.3 | Min valid depth (m). |
| `z_max` | float | 6.0 | Max valid depth (m). |

### 8.2 SCNN
| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `input_size` | tuple | (640, 480) | Must match intrinsics. |
| `mask_thresh` | float | 0.5 | Binarization threshold. |
| `dbscan_eps` | float | 2.5 | Pixel distance in rows. |
| `dbscan_min_samples` | int | 5 | Row clustering. |
| `poly_order` | int | 3 | 3rd-order fit per lane. |
| `sample_points` | int | 50 | Points along centerline. |
| `spline_smooth` | float | 0.1 | Spline smoothing factor. |

### 8.3 YOLOv7
| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `input_size` | tuple | (640, 480) | Preprocess to this size. |
| `conf_thresh` | float | 0.25 | Detection confidence. |
| `nms_iou` | float | 0.45 | NMS IoU threshold. |
| `depth_window` | int | 9 | Windowed median size (odd). |
| `conf_stop` | float | 0.6 | Stop-line event threshold. |
| `N_stop` | int | 3 | Frames required for event. |
| `use_fp16` | bool | true | FP16 inference on Jetson. |

---

## 9) QoS & Performance

- **Images**: SensorData QoS (`best_effort`, `keep_last(5)`), depth queue sized to avoid staleness.
- **Control-relevant streams** (centerline, objects, stop events): `reliable`, `keep_last(10)`.
- **Composition**: co-locate SCNN + YOLO in a **single process** to reduce intra-process copy; enable **intra-process comms**.
- **Jetson**: set performance mode (e.g., `sudo nvpmodel -m 0` and `sudo jetson_clocks`), run models in FP16, batch=1.

---

## 10) Drift Monitor vs Cartographer

1. At time `t`, get SLAM pose `T_map_base(t)` and the centerline path expressed in `map` or transform it to `map`.
2. Pick a lookahead arc-length `L` along the centerline; compute the closest point `p_L`.
3. Compute **cross-track error** \(e_\perp\) as the shortest distance from the robot position to the tangent line at `p_L`.
4. Maintain `e_⊥` statistics (mean, std). If `mean(|e_⊥|) > e_thresh` for `T_window`, raise a diagnostic and optionally trigger **re-localization**.

---

## 11) Message Contracts (canonical)

### `/centerline_path` (`nav_msgs/Path`)
- `header.frame_id = output_frame_id`
- `poses[i].pose.position = (x, y, z)`; orientation optional (tangent as yaw).

### `/centerline_3d` (`std_msgs/Float32MultiArray`, legacy)
- `data = [x1, y1, z1, ..., xN, yN, zN]` (meters, `output_frame_id`).
- `layout.dim[0] = {label: "xyz_flat", size: 3*N, stride: 1}`.

### `/objects_3d` (`std_msgs/Float32MultiArray`, legacy)
- Single detection: `[x, y, z, cls_id, conf]`.
- Multiple detections (recommended): `[x1, y1, z1, cls1, conf1, x2, y2, z2, cls2, conf2, ...]`.
- Frame must be consistently `output_frame_id`.

### `/yolo_stop` (`std_msgs/Int32`)
- `data ∈ {0, 1}`.
- Debounce with `N_stop` and `conf_stop`.

### Optional: `/detections_3d` (`vision_msgs/Detection3DArray`)
- Each `Detection3D` carries class id, score, and 3D pose/size. Use this if downstream expects standardized messages.

---

## 12) Testing & RViz Recipe

1. **Record** a short drive: RGB, aligned depth, TF, `/lane_mask`, `/centerline_path`, `/objects_3d`, `/yolo_stop`.
2. **RViz**: add Image (RGB), Image (depth), Path (centerline), TF, and Markers for objects.
3. **Metrics**: plot frame delays (Image→Mask, Image→Detections), FPS, and `e_⊥` drift.
4. **Repro**: replay rosbag2 and verify that all outputs re-render identically.

---

## 13) Known Issues & Mitigations

| Issue | Symptom | Mitigation |
|---|---|---|
| RGB–Depth sync mismatch | Scattered 3D points | Use aligned depth, approximate sync with `sync_slop_ms`. |
| YOLOv8 incompat on Jetson | Build/runtime errors | Use YOLOv7; prebuild deps; FP16; keep batch=1. |
| Missing/spiky depth | Holes or sudden jumps | Windowed median; clamp `[z_min, z_max]`; nearest valid fallback. |
| Centerline jumps | Unstable mask or depth | Sliding polyfit window; RANSAC; spline smooth; discard short-lived flips. |
| TF drift vs SLAM | Slow lateral bias | Enable drift monitor; periodic recalibration; IMU fusion in Cartographer. |
---


## 14) Notes
- Keep **frame_id** consistent across all products derived from the same image pair.
- Document **latency** budgets (per-node), especially before controller deadlines.
- When moving to another RealSense resolution, re-measure intrinsics and update projection.

---
