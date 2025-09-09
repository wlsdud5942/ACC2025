# YOLO (Real-car) — Dataset, Training, and Evaluation

We first prototyped with **YOLOv8** (desktop training speed, strong tooling) and finally **deployed YOLOv4** on the car. YOLOv4 was chosen for **Jetson stability**, **TensorRT maturity**, and **low, predictable latency**. This node raises a **stop-line event** and (optionally) publishes **local-avoidance hints**. Camera input is **640×480 @ 30 Hz** from Intel RealSense D435i (aligned depth).

---

## 0) Goals
- Detect stop lines (and simple road markings) robustly in real time.
- Emit **control-ready events** and, when needed, **metric hints** for local avoidance.

---

## 1) Frames & Time Sync
- Camera frame: `camera_color_optical_frame` (x→right, y→down, z→forward).
- Vehicle frame: `base_link` (x→forward, y→left, z→up).
- Depth is **aligned to color**; all outputs reuse the incoming image **header.stamp**.
- Synchronization: Approximate time on `{RGB, aligned_depth}`.

---

## 2) ROS 2 Topics (actually used)

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | SensorData | 640×480 RGB |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | SensorData | 16UC1 (mm) or 32FC1 (m) |
| `/yolo_stop` | `std_msgs/Int32` | Reliable, depth=1 | 0/1, debounced |
| `/obstacle_info` (optional) | `std_msgs/Float32MultiArray` | Reliable, depth=10 | `[x,y,r] × N` in `base_link` |


---

## 3) Dataset & Annotation (YOLO format)
Directory:
- `images/{train,val,test}/*.jpg|png`
- `labels/{train,val,test}/*.txt`
- `classes.txt` (e.g., `stop_line`, `crosswalk`, `sign`)

Label file line (normalized 0–1):  
`<class_id> <x_center> <y_center> <width> <height>`

Annotation rules: tight boxes; for crosswalk, box the drivable region; ignore heavy occlusion (>50%).

---

## 4) Training Setup (YOLOv4, Darknet/AlexeyAB)
- **Camera / model input**: **640×480** (QCar2 default) → letterboxed to **512×512**.
- **Iterations actually run**: **1,000** (fine-tuning phase).
- **Augmentation**: `random=1`, `mosaic=1`, `mixup=1`.
- **Scheduler**: `max_batches=6000`, **`steps=4800`**.
- Typical cfg: `batch=64`, `subdivisions=16`, `lr=0.001`, `momentum=0.949`, `decay=0.0005`.
- TRT inference: export to TensorRT; **FP16** recommended (INT8 with 200–500 image calibration).

---

## 5) Results (validation)
- **mAP@50** ≈ **99.7 %**.  
- **IoU** and **Recall** both **near 100 %** on the validation set.  
- PR curves and confusion matrices tracked during training.

---

## 6) Event & Avoidance Logic
- `/yolo_stop`: publish `1` when class `stop_line` is detected with `confidence ≥ conf_stop` for **N_stop** consecutive frames, **and** the box center lies inside a **bottom-band ROI**, **and** the fused depth is within `[z_stop_min, z_stop_max]`. Cooldown prevents re-trigger spam.
- `/obstacle_info` (optional): for non-stop objects, take a windowed median of depth at the box center, project to `base_link`, compress as `[x,y,r]` for local avoidance.

---

## 7) Parameters (640×480)

| Group | Param | Default | Note |
|---|---|---:|---|
| model | `img_size` | 640 | letterbox size |
| model | `conf_thresh` | 0.25 | pre-NMS filter |
| model | `nms_iou` | 0.45 | NMS IoU |
| runtime | `use_fp16` | true | TensorRT FP16 |
| stop | `conf_stop` | 0.60 | stop-line event threshold |


---

## 8) QoS & Performance
- Images: SensorData (`best_effort`, `keep_last(5)`).
- Events/hints: Reliable (`keep_last(10)`).
- Jetson performance mode; co-locate RGB/Depth consumers with intra-process comms; FP16 TRT path only.

---

## 9) Message Contracts
- `/yolo_stop` (`std_msgs/Int32`): `0/1` after debounce, ROI, and range checks.
- `/obstacle_info` (`std_msgs/Float32MultiArray`, optional): data = `[x1,y1,r1, x2,y2,r2, …]` in **meters**, `base_link`.

---

## 10) Testing & RViz
Record RGB, aligned depth, TF, `/yolo_stop`, and `/obstacle_info`. In RViz: RGB, depth, TF, and text markers for event state. Replay rosbag2 to verify deterministic outputs and latency.

---

## 11) Why v8 → v4 (decision log)
- v8 excelled in **desktop** iteration but had **TRT export/ops** gaps and **latency spikes** on Jetson.
- v4 offered stable TRT plugins, FP16/INT8 readiness, and met control deadlines with margin.
