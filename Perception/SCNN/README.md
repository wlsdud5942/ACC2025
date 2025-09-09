# SCNN-based Lane Detection — Dataset, Training, and Evaluation

We replaced early classification-style lane detection with **SCNN segmentation**. Public weights were not enough, so we **retrained** on our track (curvature, glare, worn paint). Post-processing yields a smooth **3D centerline** for control. Input: **640×480**.

---

## 0) I/O (used)

| Topic | Type | Frame | QoS | Notes |
|---|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | 640×480 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | SensorData | aligned depth |
| `/lane_mask` | `sensor_msgs/Image` | `camera_color_optical_frame` | Reliable (depth=5) | 0/255 mask |
| `/centerline_path` | `nav_msgs/Path` | **`base_link`** | Reliable (depth=10) | equal-arc 3D centerline |

---

## 1) Dataset Format (segmentation)

Structure:
dataset_scnn/
 ├─ images/{train,val,test}/*.jpg|png
 ├─ masks/{train,val,test}/*.png    # 8-bit single-channel, 0=bg, 255=lane
 └─ splits.txt (optional)

Annotation rules: connect broken paint in the intended driving sense; exclude road letters/arrows; include glare/wet/worn cases.

---

## 2) Training Recipe

Preproc/Aug: resize+pad to 640×480, normalize; ColorJitter, gamma, mild blur/noise; small RandomPerspective; HorizontalFlip (mind left/right semantics).  
Hyperparams (example): AdamW, LR 1e-3 (cosine decay), weight decay 1e-4; Loss = BCEWithLogits + Dice (0.5/0.5); 100 epochs (early stop 10); batch 8–16; EMA enabled.

---

## 3) Post-processing → Centerline

1) Row-wise clustering (DBSCAN) → left/right lane clusters  
2) 3rd-order polyfit for each, with optional RANSAC  
3) Average left/right → **2D centerline**  
4) Depth fusion → **3D back-projection**  
5) Short **spline (C¹)** + equal-arc resampling → **`/centerline_path`**

Projection from pixel `(u,v)` and depth `Z` (m):
Xc = (u - cx)/fx * Z
Yc = (v - cy)/fy * Z
Zc = Z
p_base = T_base_link_camera · [Xc, Yc, Zc, 1]^T

---

## 4) Evaluation

• **mIoU** (lane vs bg) and **Pixel Accuracy** on validation masks  
• **Centerline lateral error** (mean / P95) vs SLAM/GT at look-ahead `L`  
• **Latency / FPS** from model through path publication

Template:
Validation @ 640×480
- mIoU / PA            :  __.__ % / __.__ %
- Lateral error (mean) :  __.__ m
- Lateral error (P95)  :  __.__ m
Latency (SCNN+post)    :  __.__ ms    FPS: __.__

---

## 5) Notes & Lessons

• Segmentation + spline produced robust geometry on sharp curves and lighting changes.  
• We attempted to correct Cartographer using the vision centerline; due to timing/loop-closure/scale issues the **online correction failed**. We keep a **drift monitor** instead.
