# YOLO (Real-car) — Dataset, Training, and Evaluation

We prototyped with **YOLOv8** and deployed **YOLOv4** on the car. This node raises a **stop-line event** and can publish **local-avoidance hints**. Input resolution: **640×480**.

---

## 0) Why we started with **YOLOv8**
- Strong accuracy/tooling on desktop; fast fine-tuning and validation.
- High developer velocity for dataset bootstrapping.

## 0.1) Why we switched to **YOLOv4** on Jetson
- **TensorRT export / ops issues** and **latency spikes** with v8 (even FP16).
- **Dependency collisions** (CUDA/cuDNN/PyTorch) on our onboard stack.
- **YOLOv4** had mature TRT plugins and stable FP16/INT8 performance → reliably met controller deadlines.

---

## 1) I/O (used in this project)

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | SensorData | 640×480 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | SensorData | convert to meters |
| `/yolo_stop` | `std_msgs/Int32` | Reliable (depth=1) | 0/1, debounced |
| `/obstacle_info` *(optional)* | `std_msgs/Float32MultiArray` | Reliable (depth=10) | `[x,y,r] × N` in `base_link` |

> We **did not** publish `/objects_3d` or `/detections_3d`. All outputs keep the input image `header.stamp`.

---

## 2) Dataset Format

Directory layout (YOLO):
dataset_yolo/
 ├─ images/{train,val,test}/*.jpg|png
 ├─ labels/{train,val,test}/*.txt
 └─ classes.txt  # one class per line (stop_line, crosswalk, sign, …)

Label `.txt` (normalized 0–1 per line):
<class_id> <x_center> <y_center> <width> <height>

Example:
0 0.512 0.673 0.280 0.050   # stop_line
1 0.430 0.610 0.320 0.190   # crosswalk

---

## 3) Training Recipe (YOLOv4, Darknet)

• **Camera / model input**: **640×480** (QCar2 default).  
• **Fine-tuning iterations** actually run: **1,000 iters**.  
• **Augmentation**: `random=1`, `mosaic=1`, `mixup=1`.  
• **Scheduler**: `max_batches=6000`, `steps=4800`.  
• Typical cfg knobs: `batch=64`, `subdivisions=16`, `lr=0.001`, `momentum=0.949`, `decay=0.0005`.

Example command (Darknet/AlexeyAB):
./darknet detector train data/obj.data cfg/yolov4-512.cfg yolov4.conv.137 -map

**Inference (Jetson)**: export to TensorRT; use FP16 (optionally INT8 with 200–500 image calibration).

---

## 4) Evaluation Results

• **mAP@50** ≈ **99.7%** (validation)  
• **IoU** and **Recall**: both **near 100%** on our val set  
• Logged PR curves / confusion matrices via `-map`.

Reporting template (future runs):
Validation @ 512 letterbox
- mAP@50 (IoU=0.5)  :  99.7 %
- Precision / Recall:  0.96 / 0.99


---

## 5) Event & Avoidance Logic

• `/yolo_stop`  : class `stop_line` with `conf ≥ conf_stop` for **N_stop** consecutive frames **AND**
                   ROI gating (bottom band) **AND** valid range (e.g., 0.5–8 m) → publish `1` (with cooldown).  
• `/obstacle_info` (opt): windowed depth median at bbox center → 3D point, compressed as `[x,y,r]`.

---

## 6) Key Parameters (640×480)

| Group  | Param             | Default | Note                       |
|-------|--------------------|--------:|----------------------------|
| model | img_size           |     640 | letterbox                  |
| model | conf_thresh        |    0.25 | pre-NMS filter             |
| model | nms_iou            |    0.45 | NMS IoU                    |
| rt    | use_fp16           |    true | Jetson                     |
| stop  | conf_stop          |    0.60 | stop event threshold       |
| stop  | N_stop             |       3 | debounce frames            |
| stop  | z_stop_min,max     | 0.5, 8.0| range gating (m)           |
| stop  | roi_band_bottom    |    0.40 | lower image band fraction  |
