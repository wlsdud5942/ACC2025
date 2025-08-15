# SCNN-based Lane Detection — Process-Centric README (Revised)

This module ports a paper-based **Spatial CNN (SCNN)** to a **640×480, real-vehicle** setup and adds post-processing to generate a **3D lane centerline** consumed by Planning/Control. The system fuses **RGB + Depth** from Intel RealSense D435i and compares the vision-based centerline with **Cartographer SLAM** for drift monitoring and mitigation.

---

## 1) Objectives
- Detect left/right lanes in real time and extract a smooth **centerline**.
- Fuse RGB with Depth to output a **3D drivable centerline** in meters.
- Use the centerline vs SLAM comparison to **monitor/mitigate localization drift**.

---

## 2) Frames & Calibration

**Frames**
- **Camera (RealSense optical)**: x→right, y→down, z→forward (RealSense convention).
- **Vehicle (base_link)**: x→forward, y→left, z→up.
- **Extrinsics**: fixed transform \(T^{\text{camera}}_{\text{base\_link}}\) via TF (`static_transform_publisher`).

**Calibration workflow**
1. Measure intrinsics/extrinsics with a checkerboard (OpenCV).
2. Register the static transform through `static_transform_publisher`.
3. Verify alignment in RViz (overlay color/depth; project landmarks).

**Depth alignment**
- Prefer `/camera/aligned_depth_to_color/image_raw` so RGB pixel `(u,v)` and depth share the same frame.

**Recommended storage**
- `scnn/config/realsense_intrinsics.yaml` and `scnn/config/T_base_link_camera.yaml`.

---

## 3) Sensors & Topics (contract)

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData (best_effort) | Encoding: `rgb8`/`bgr8`. |
| `/camera/aligned_depth_to_color/image_raw` (preferred) or `/camera/depth/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` (if aligned) | SensorData (best_effort) | `16UC1` (mm) or `32FC1` (m). Convert to meters. |
| `/lane_mask` | `sensor_msgs/Image` | `camera_color_optical_frame` | Reliable, depth=5 | 8-bit single-channel (0/255), 640×480. |
| `/centerline_path` (standard) | `nav_msgs/Path` | `base_link` or `map` | Reliable, depth=5 | Tangent yaw optional. |
| `/centerline_3d` (legacy) | `std_msgs/Float32MultiArray` | `base_link` | Reliable, depth=10 | Flat `[x1,y1,z1, ...]` meters. |

**Timestamping**
- Derive all outputs’ `header.stamp` from the **camera image stamp** to preserve causality.

---

## 4) Implementation

### 4.1 Model Adaptation
- Updated model/inference to accept **640×480** inputs (maintain aspect ratio; adjust decoder/upsample).
- Output head produces a **binary mask** (0/255) rather than 4-channel soft labels.
- Kept SCNN backbone/topology; changed pre/post transforms for our camera geometry.

### 4.2 Post-processing Pipeline (Core)
1. **Row-wise DBSCAN** clusters lane pixels; discard small blobs.
2. **3rd-order polyfit** per cluster (left/right lanes).
3. **Centerline** = mean of left/right polynomials at sampled rows; handle missing sides via last-valid carry and continuity penalty.
4. **Depth fusion** (camera → base_link):
   Given pixel `(u, v)` and depth `Z` (m):
   ```
   Xc = (u - cx)/fx * Z
   Yc = (v - cy)/fy * Z
   Zc = Z
   ```
   Transform to `base_link` via TF: `p_b = T_base_link_camera · [Xc, Yc, Zc, 1]^T`.
5. **Temporal smoothing** of polynomial coefficients (EMA or Kalman). Optionally smooth the 3D centerline with a short cubic spline and resample at fixed arc-length for `/centerline_path`.

**Alternatives**
- Replace polyfit with **RANSAC polyfit** for stronger outlier rejection.
- Add **column-wise continuity** or pairwise consistency checks for occlusion/reflection robustness.

### 4.3 ROS2 Integration
- **Subscribe**: RGB, aligned depth.
- **Publish**: `/lane_mask`, `/centerline_path` (preferred), and legacy `/centerline_3d`.
- **QoS**: images → SensorData; processed streams → Reliable `keep_last(10)`.
- **Frames**: outputs default to `base_link`; use `map` if global consumers require it (apply SLAM pose).

---

## 5) Drift Monitor vs Cartographer

- At time `t`, transform the centerline to `map` or transform SLAM pose to `base_link` consistently.
- Choose a lookahead arc-length `L`; get the closest centerline point `p_L` and its tangent.
- Compute **cross-track error** \(e_\perp\) from robot position to the tangent at `p_L`.
- Maintain a sliding window of `e_⊥` (mean/std). If `mean(|e_⊥|) > e_thresh` for `T_window`, raise a diagnostic and optionally trigger **relocalization**.
- Log `timestamp, e_perp, pose_source, decision` → `logs/slam_drift.csv`.

---

## 6) Parameters

| Group | Parameter | Type | Default | Notes |
|---|---|---|---:|---|
| general | `output_frame_id` | string | `base_link` | Output frame for centerline.
| general | `sample_points` | int | 50 | Points along image-space centerline before 3D back-projection.
| dbscan | `eps_px` | float | 3.0 | Pixel distance per row.
| dbscan | `min_samples` | int | 30 | Suppresses small blobs.
| poly | `order` | int | 3 | 2–3 typical.
| poly | `ransac` | bool | false | Enable robust fit.
| depth | `use_aligned_depth` | bool | true | Prefer aligned stream.
| depth | `median_k` | int | 7 | Depth median window (odd).
| smooth | `ema_alpha` | float | 0.3 | Coeff smoothing (lower = smoother).
| smooth | `spline_smooth` | float | 0.1 | Spline smoothing for `/centerline_path`.
| drift | `lookahead_m` | float | 2.0 | For cross-track error.
| drift | `e_thresh_m` | float | 0.25 | Drift threshold.
| drift | `T_window_s` | float | 3.0 | Window length.


---

## 7) QoS & Performance
- **Images**: SensorData (best_effort, keep_last(5)).
- **Centerline/Mask**: Reliable, keep_last(10).
- **Composition**: run SCNN and post-processing in the **same process** with intra-process comms; pin Jetson to max perf.
- Target latency: SCNN < **25 ms**, post-processing < **5–10 ms** @ 30 FPS (tune per device).

---

## 8) Message Contracts (canonical)

### `/lane_mask` (`sensor_msgs/Image`)
- 8-bit, single-channel, 640×480, values {0,255}; `header.frame_id = camera_color_optical_frame`.

### `/centerline_path` (`nav_msgs/Path`)
- `header.frame_id = output_frame_id`.
- `poses[i].pose.position = (x, y, z)`; orientation optional (tangent yaw).

### `/centerline_3d` (legacy — `std_msgs/Float32MultiArray`)
- `data = [x1, y1, z1, ..., xN, yN, zN]` (meters) in `output_frame_id`.
- `layout.dim[0] = {label: "xyz_flat", size: 3*N, stride: 1}`.

---

## 9) Performance, Limits, and Lessons
- **Post-processing quality dominates** overall path usability.
- Challenging cases: low light, glare, lane wear → mitigate with mask-space median filtering, coefficient smoothing, and continuity penalties.
- Track **FPS/latency**, mask **IoU**, and **centerline lateral error** vs ground truth or SLAM.

---



## 10) Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Centerline jumps between frames | Outliers / missing side | Enable RANSAC; enforce continuity; increase EMA smoothing. |
| Depth spikes near paint edges | Invalid pixels | Expand median window; clamp `Z` to `[0.3, 6.0]` m. |
| Mask tears in glare | Over/under-exposure | Adaptive thresholding, brightness normalization pre-SCNN. |
| Mismatched frames | Time sync issues | ApproximateTime sync; composition with co-stamped RGB/Depth. |
```
