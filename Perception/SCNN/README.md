# SCNN-based Lane Detection — Dataset, Training, and Evaluation

We migrated from early **classification-style lanes** (unstable on sharp curvature/lighting jumps) to **segmentation-based SCNN**. Public weights were insufficient; we **retrained** on our track and stabilized geometry with **spline post-processing** to publish a smooth **3D centerline**. Input is **640×480 @ 30 Hz** (RGB + aligned depth).

---

## 0) Goals
- Robust lane perception under curvature, glare, wet asphalt, and worn paint.
- Output an **equal-arc 3D centerline** consumable by planning/control.

---

## 1) Frames, Calibration, and Time Sync
- Camera: `camera_color_optical_frame`; Vehicle: `base_link`.  
- Static TF: `T_base_link_camera`.  
- Depth is **aligned to color**; outputs reuse the image **header.stamp**.  
- Use Approximate time sync for `{RGB, aligned_depth}`.

---

## 2) ROS 2 Topics (actually used)

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | 640×480 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | aligned depth |
| `/lane_mask` | `sensor_msgs/Image` | `camera_color_optical_frame` | Reliable, depth=5 | 0/255 mask |
| `/centerline_path` | `nav_msgs/Path` | `base_link` | Reliable, depth=10 | 3D, equal-arc samples |

> We did **not** publish `/centerline_3d` in this project’s final wiring.

---

## 3) Dataset (segmentation)
- Structure: `images/{train,val,test}`, `masks/{train,val,test}` (8-bit, `0=bg`, `255=lane`).
- Split: 8/1/1 with balance across lighting/weather.
- Annotation rules: connect intended lane even if paint is broken; exclude text/arrows; include glare/wet/worn.

---

## 4) Training
- Preproc/Aug: resize/pad 640×480; ColorJitter, gamma, mild blur/noise; light RandomPerspective; HorizontalFlip (mind L/R semantics).
- Hyperparams (example): AdamW, LR 1e-3 (cosine), weight decay 1e-4; Loss = BCEWithLogits + Dice (0.5/0.5); 100 epochs (early stop 10); batch 8–16; EMA.

---

## 5) Post-processing → 3D Centerline
1. Row-wise clustering (DBSCAN) → left/right lane sets.  
2. 3rd-order polyfit per lane (optional RANSAC).  
3. Average left/right → **2D centerline**.  
4. Back-project with depth to camera 3D:  
   `Xc = (u−cx)/fx * Z`, `Yc = (v−cy)/fy * Z`, `Zc = Z`.  
   Transform to `base_link`: `p_base = T_base_link_camera · [Xc,Yc,Zc,1]^T`.  
5. Short **spline (C¹)** + equal-arc resampling → publish **`/centerline_path`**.

---

## 6) Parameters

| Group | Param | Default | Note |
|---|---|---:|---|
| dbscan | `eps_px` | 3.0 | per-row pixel distance |
| dbscan | `min_samples` | 30 | filter small blobs |
| poly | `order` | 3 | 2–3 typical |
| depth | `median_k` | 7 | depth window around pixel |
| path | `sample_points` | 50 | along image centerline |
| path | `spline_smooth` | 0.1 | smoothing factor |

---

## 7) QoS & Performance
- Images: SensorData (`best_effort`, `keep_last(5)`).
- Mask/path: Reliable (`keep_last(10)`).
- Co-locate model + postproc; enable intra-process comms; target SCNN < 25 ms + postproc < 10 ms @ 30 FPS.

---

## 8) Evaluation
- **mIoU** and **Pixel Accuracy** on masks.  
- **Centerline lateral error** (mean / P95) at look-ahead `L` vs SLAM/GT.  
- **Latency / FPS** end-to-end (model → `/centerline_path`).

Template:
- mIoU / PA: __.__ % / __.__ %  
- Lateral error (mean / P95): __.__ m / __.__ m  
- Latency (SCNN+post): __.__ ms, FPS: __.__

---

## 9) Drift vs Cartographer
We attempted online SLAM correction with the vision centerline; due to timing and loop-closure/scale issues the **online correction failed**. We keep a **drift monitor** (sliding stats of cross-track error) and raise diagnostics when thresholds persist for `T` seconds.

---

## 10) Testing & RViz
Record RGB, aligned depth, TF, `/lane_mask`, `/centerline_path`. In RViz: Image (RGB/Depth), Path, TF. Replay bags to confirm deterministic output and timing.

---

## 11) Known Issues & Mitigations
- Mask tears under glare → stronger denoise, coefficient smoothing.  
- Depth spikes on paint edges → enlarge `median_k`, clamp depth range.  
- Left/right swap in flips → disable HFlip or enforce side consistency during aug.
