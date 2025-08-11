# Perception Module

This directory documents the **full perception stack** for our autonomous vehicle. The stack consists of **SCNN-based lane detection** and **YOLOv7-based object detection**, both driven by **Intel RealSense D435i RGB + Depth**. Outputs are expressed in **metric 3D coordinates** (centerline, stop line / signage, obstacles) and fed to **Planning / Control / ROS2** for path generation, FSM transitions, and avoidance decisions.

---

## 1) Goals & Scope
- Detect a **3D lane centerline** and **road objects** in real time and convert them to **actionable metric units (meters)**.
- Provide lane, stop line, crosswalk, and sign detections under **consistent QoS & timestamping**.
- Compare the **SCNN-derived 3D centerline** against **Cartographer SLAM** poses to monitor and mitigate **long-term drift**.

---

## 2) Frames & Calibration

Frames
- Camera (RealSense optical): x → right, y → down, z → forward (RealSense SDK convention).
- Vehicle (base_link): x → forward, y → left, z → up.
- Extrinsics: fixed transform T_base_link^camera broadcast via ROS TF.

Calibration workflow
1. Estimate intrinsics/extrinsics with a checkerboard (OpenCV).
2. Register the static transform via static_transform_publisher.
3. Verify alignment in RViz (image overlay / depth alignment).

Intrinsics (example placeholders)
- fx, fy, cx, cy (replace with measured values).

Suggested storage
- perception/calib/realsense_intrinsics.yaml

---

## 3) Sensors & Key Topics

Device / Topic | Type | Contract
--- | --- | ---
RealSense D435i (640×480 RGB + Depth) | – | Color/Depth approx sync (or composed callback)
/camera/color/image_raw | sensor_msgs/Image | BGR8, stamp/frame_id = camera frame
/camera/depth/image_raw | sensor_msgs/Image | 16UC1 or 32FC1, meters
/lane_mask | sensor_msgs/Image | SCNN binary mask (0 / 255), 640×480
/centerline_3d | std_msgs/Float32MultiArray | Flat array: [x1,y1,z1, x2,y2,z2, …] (meters, base_link recommended)
/obstacle_info | std_msgs/Float32MultiArray | e.g., [x, y, z, cls_id, conf] (meters, camera or base_link; be explicit)
/yolo_stop | std_msgs/Int32 | 0/1 stop-line event

QoS recommendations
- Images: best_effort or SensorDataQoS.
- Control-relevant streams: reliable, keep_last(10).
- Maintain consistent frame_id & timestamps.

---

## 4) SCNN-based Lane Detection (Summary)

Input / Output
- Input: 640×480 RGB.
- Output: binary lane mask and a 3D centerline.

Post-processing
1. Row-wise DBSCAN to cluster lane pixels.
2. 3rd-order polyfit per cluster.
3. Average left/right fits → centerline in image coords.

Depth fusion (camera frame)
- Given pixel (u, v) and depth Z:
  - X = (u - cx) / fx * Z
  - Y = (v - cy) / fy * Z
- Project (X, Y, Z) to base_link via TF before publishing.

Drift monitoring
- Compare centerline-derived lateral position vs SLAM pose; track the lateral error as a drift indicator.

See also: perception/scnn/README.md

---

## 5) YOLOv7-based Object Detection (Summary)

Why v7
- Jetson compatibility issues with YOLOv8 (C++ ops) → YOLOv7 with tuned dependencies.

Classes (project-dependent examples)
- stop_line, crosswalk, sign (extend as needed).

Depth extraction
- Use a windowed median around the BBox center to reduce missing/noisy depth.

Outputs
- /obstacle_info: [x, y, z, cls_id, conf] (define frame explicitly).
- /yolo_stop: 0/1 for the FSM when a stop line is confidently detected.

See also: perception/yolov7/README.md

---

## 6) RealSense Integration & Depth Use

- From SCNN/YOLO pixel coordinates, fetch depth from /camera/depth/image_raw.
- Convert to 3D (camera frame) → transform to base_link with TF → publish.
- Depth noise mitigation: 3×3–7×7 median or biweight window; interpolate NaNs.
- Time sync: ApproximateTimeSynchronizer or composition node with stamp reordering.

---

## 7) Processing Pipeline (Parallelized)

- Parallel branches to minimize latency:
  - SCNN path: RGB → mask → centerline → depth → 3D centerline
  - YOLOv7 path: RGB → detections → depth window median → 3D objects
- Both publish to ROS2 → Planning / Control (FSM) consume.

---

## 8) Known Issues & Mitigations

Issue | Mitigation
--- | ---
RGB–Depth sync mismatch | Approx sync + consistent frame_id; prefer camera HW timestamp
YOLOv8 incompat on Jetson | Switch to YOLOv7; clean deps; optimize NMS / FP16
Missing / spiky depth | Windowed median around BBox center; clamp valid range (e.g., 0.3–6 m)
Centerline jumps | Sliding polyfit window, outlier rejection (RANSAC option)

---

## 9) Message Contracts (Examples)

- /centerline_3d (Float32MultiArray)
  - data: [x1, y1, z1, x2, y2, z2, ..., xN, yN, zN]   (meters, base_link)
  - layout.dim[0]: size = 3N, stride = 1, label = "xyz_flat"

- /obstacle_info (Float32MultiArray)
  - Single detection: [x, y, z, cls_id, conf]
  - Multiple detections (recommended): concatenate per detection  
    [x1, y1, z1, cls1, conf1, x2, y2, z2, cls2, conf2, ...]
  - Define and keep a consistent coordinate frame (camera or base_link).

- /yolo_stop (Int32)
  - data: 0 | 1 (1 when a stop line is confidently detected)

---

