# Perception Module

Intel RealSense **D435i** (RGB + **Aligned** Depth, **640×480 @ 30 Hz**) feeds two perception paths:

- **3D lane centerline** → `/centerline_path`  
- **Stop-line event** → `/yolo_stop`  
- **(Optional) local-avoidance hints** → `/obstacle_info` as `[x,y,r] × N`

Lanes are produced by **SCNN segmentation** with **spline** post-processing; objects are detected by **YOLOv4** (we initially prototyped with YOLOv8; see the YOLO README for the rationale).

---

## 0) Goals
- Emit control-ready signals in **meters**, with consistent **timestamps** and **QoS**.  
- Compare the SCNN centerline against **Cartographer** to **monitor** long-term drift (we do **not** override SLAM online).

---

## 1) Frames & Calibration
- **Camera (optical)**: x→right, y→down, z→forward  
- **Vehicle (`base_link`)**: x→forward, y→left, z→up  
- **Extrinsics**: static TF `T_base_link_camera`  
- **Depth**: prefer `/camera/aligned_depth_to_color/image_raw` so RGB and depth share the same frame.

---

## 2) ROS 2 Topics (actually used)

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | 640×480, `rgb8`/`bgr8` |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | `16UC1` (mm) or `32FC1` (m) |
| `/lane_mask` | `sensor_msgs/Image` | `camera_color_optical_frame` | Reliable (depth=5) | SCNN binary mask |
| `/centerline_path` | `nav_msgs/Path` | **`base_link`** | Reliable (depth=10) | Equal-arc 3D samples |
| `/yolo_stop` | `std_msgs/Int32` | — | Reliable (depth=1) | 0/1, debounced |
| `/obstacle_info` *(optional)* | `std_msgs/Float32MultiArray` | **`base_link`** | Reliable (depth=10) | `[x,y,r] × N` |

> We **did not** publish `/objects_3d` or `/detections_3d`.

All derived messages reuse the **camera image `header.stamp`**.

---

## 3) End-to-End Pipeline

RGB + Aligned Depth  →  (SCNN lane mask) & (YOLOv4 bbox,class,conf)  
TimeSync (Approximate) keeps RGB/Depth aligned.

- **SCNN post-proc**: 2D lanes → centerline → 3D back-projection → **`/centerline_path`**  
- **YOLO post-proc**: bbox + depth rules → **`/yolo_stop`** (0/1) and *(optional)* **`/obstacle_info [x,y,r]×N`**

---

## 4) Dataset & Training (overview)
- **Common input**: RGB **640×480** (QCar2 default); depth aligned to color.  
- **Split**: train / val / test = **8 / 1 / 1**, balanced by season, lighting, weather.  
- **Versioning**: `dataset_v1.0` → `v1.1` (adds glare, worn paint, wet asphalt).  
- Details per module:
  - **YOLO**: YOLO text labels, training recipe, **mAP@50 / mAP@0.5:0.95**.  
  - **SCNN**: binary masks, augmentation, training, **mIoU / lateral error**.

---

## 5) Why SCNN & Why YOLOv4
- Early **classification-style lane** models were brittle on sharp curvature and lighting jumps. We moved to **segmentation (SCNN)**, **retrained** on our track, and stabilized with **splines**.  
- For objects we **started with YOLOv8** (fast iteration on desktop) but switched to **YOLOv4** for Jetson deployment due to TensorRT maturity and stable low latency. See YOLO README for details.

---
