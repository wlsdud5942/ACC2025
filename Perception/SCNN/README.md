# SCNN-based Lane Detection 

This module ports a paper-based **Spatial CNN (SCNN)** to a **640×480, real-vehicle** setup and adds post-processing to generate a **3D lane centerline** used by Planning/Control. The system fuses **RGB + Depth** from Intel RealSense D435i and compares the vision-based centerline with **Cartographer SLAM** for drift monitoring and mitigation.

---

## 1) Objectives
- Detect left/right lanes in real time and extract a smooth **centerline**.
- Fuse RGB with Depth to output a **3D drivable centerline** in meters.
- Use the centerline vs SLAM comparison to **monitor/mitigate localization drift**.

---

## 2) Original SCNN Issues and Our Modifications

Item | Original Limitation | Our Modification
--- | --- | ---
Input size | Fixed at 295×128 | Support **640×480** (resize while preserving aspect; adjusted upsampling)
Output format | 4-channel soft labels | **Single-channel binary mask** (0/255) for simpler post-processing
Inference script | Hardcoded file paths | **ROS2 node I/O**: subscribes to images, publishes mask/centerline

Notes
- Kept SCNN backbone/topology; changed pre/post transforms and output head to suit binary lane segmentation.
- Ensured inference latency fits the real-time budget on the target device.

---

## 3) Implementation

### 3.1 Model Adaptation
- Updated `model.py` and inference script (`test.py` or equivalent) to accept **640×480** inputs.
- Adjusted decoder/upsample path so the lane mask aligns with camera intrinsics.
- Directly produce a **binary mask** to simplify downstream clustering (no argmax over multi-channel logits).

### 3.2 Post-processing Pipeline (Core)
1) Row-wise **DBSCAN** to cluster lane pixels and suppress small noisy blobs.  
2) **3rd-order polynomial fit** (polyfit) per cluster to model left/right lane curves.  
3) Compute **centerline** as the mean of left/right fits at sampled rows (guarding for missing sides).  
4) **Depth fusion** to obtain 3D points:
   - Given pixel (u, v) with depth Z:
     - X = (u − cx) / fx * Z
     - Y = (v − cy) / fy * Z
     - Z = Z
   - Transform from camera to `base_link` using TF (T_base_link^camera).  
5) **Temporal smoothing** of polynomial coefficients:
   - EMA (exponential moving average) or Kalman filter on coefficients to stabilize across frames.

Alternatives
- Replace polyfit with **RANSAC polyfit** for stronger outlier rejection.  
- Use **column-wise voting** or continuity penalties for robustness in occlusions/reflections.

### 3.3 ROS2 Integration
- Subscribe: `/camera/color/image_raw`, `/camera/depth/image_raw`.
- Publish: `/lane_mask` (binary), `/centerline_3d` (Float32MultiArray as [x1,y1,z1, x2,y2,z2, ...], meters in `base_link`).
- QoS: image topics `best_effort` or SensorDataQoS; post-processed topics `reliable, keep_last(10)`.
- Frame/time: maintain consistent `frame_id`, use camera HW timestamps when possible.

---

## 4) Cartographer Coupling (Drift Handling)
- Compute lateral error e_y between the **vision centerline-based position** and **SLAM pose**.
- If e_y exceeds a threshold (e.g., 0.2–0.3 m) consistently over a window, trigger **relocalization / map reset** logic.
- Log to `logs/slam_drift.csv`: timestamp, e_y, decision, pose source.

---

## 5) Example Artifacts to Insert (placeholders)
Step | What to show
--- | ---
Input RGB | Example frame (day/night if available)
SCNN mask | Binary lane mask (0/255) overlay on RGB
DBSCAN + polyfit | Cluster visualization and fitted curves
3D centerline | RViz capture of centerline in `base_link`

---

## 6) Performance, Limits, and Lessons
- **Post-processing quality dominates**: clustering and curve fitting have the biggest impact on usable centerlines.
- Challenging cases: low light, glare, lane wear. Mitigation: **median filtering** in mask space, coefficient smoothing, and continuity constraints.
- Record metrics per scenario: **FPS / latency**, mask **IoU**, **centerline lateral error** vs ground truth or SLAM.

---

## 7) Directory Layout (suggested)
scnn/  
├─ README.md  
├─ models/              (SCNN definition, weights)  
├─ scripts/             (inference node, utilities)  
├─ configs/             (intrinsics/extrinsics, thresholds)  
└─ logs/                (drift logs, per-frame diagnostics)

---

## 8) Message Contracts (examples)

Topic | Type | Notes
--- | --- | ---
/lane_mask | sensor_msgs/Image | 8-bit single-channel, 640×480, 0/255
/centerline_3d | std_msgs/Float32MultiArray | Data = [x1,y1,z1, x2,y2,z2, ...], meters in `base_link`

Validation tips
- Check that centerline spacing along y (forward axis in `base_link`) is smooth and monotonic.  
- Verify no sudden yaw jumps when lanes partially vanish.

---

## 9) Tunables (quick reference)

Category | Parameter | Typical Range / Default | Effect
--- | --- | --- | ---
DBSCAN | eps (pixels) | 2–6 | Cluster tightness; higher joins more pixels
DBSCAN | min_samples | 20–60 | Suppresses small blobs
Polyfit | order | 2–3 | 3 captures gentle curvature; avoid overfit
Depth | median window | 3×3 to 7×7 | Stabilizes missing/noisy depth
Smoothing | EMA alpha | 0.2–0.5 | Lower = smoother, higher latency
Drift | e_y threshold | 0.2–0.3 m | Sensitivity of drift alarms

---

## 10) Troubleshooting

Symptom | Likely Cause | Fix
--- | --- | ---
Centerline “jumps” between frames | Outliers / missing side | Use RANSAC, enforce continuity, increase EMA smoothing
Depth spikes near edges | Invalid pixels around lane paint | Expand median window; clamp Z to valid range (e.g., 0.3–6 m)
Mask tears in glare | Over/under-exposed regions | Adaptive thresholding, brightness normalization before SCNN
Mismatched frames | Time sync issues | Use ApproximateTimeSynchronizer or compose nodes to co-stamp RGB/Depth
