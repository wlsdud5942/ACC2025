# Perception

## Introduction

This module processes RGB and depth camera data from the vehicle to recognize key driving environment features such as traffic signs and lane markings.  
It provides the control and localization systems with accurate, real-time information extracted through deep learning models.  
Two separate perception pipelines are implemented:

- **YOLOv7** for traffic object detection
- **SCNN** for lane centerline detection and localization refinement

All outputs are published over ROS2 and used either in Simulink-based control systems or Cartographer SLAM correction modules.

---

## System Overview

The perception system is divided into two core components:

- **YOLOv7 (Object Detection)**  
  Detects traffic signs (e.g., Stop, Crosswalk, Roundabout) in real time.  
  Detection results are published as ROS2 messages and used by Simulink’s Stateflow controller to trigger behavioral transitions (e.g., stopping, slowing).

- **SCNN (Lane Detection for Localization Refinement)**  
  Detects lane centerlines from RGB images and converts them into real-world coordinates using depth data.  
  These points are matched against the pre-built Cartographer SLAM map to refine the vehicle’s estimated pose.

---

## Data Flow Architecture (Visualized)

The RealSense D435i camera streams both RGB and Depth frames to the perception nodes.  
- The YOLO node publishes detection results to `/traffic_sign_topic`, consumed by the Simulink controller.  
- The SCNN pipeline extracts lane masks and computes 3D (x, y, z) coordinates of the road centerline using depth data.  
- These coordinates are then aligned with Cartographer’s SLAM map to refine vehicle localization.

> See the Mermaid graph at the bottom of this file for the full system flow.

---

## Role of Each Model

### YOLOv7

- Real-time inference using PyTorch-based YOLOv7  
- Publishes object class index via `/traffic_sign_topic`  
- Enables vehicle state transitions in Simulink Stateflow  
- Detected objects: Stop Sign, Crosswalk, Roundabout, Yield, etc.

### SCNN + Depth

- Extracts lane masks from RGB images  
- Uses pixel coordinates and depth values to compute (x, y, z) lane centerline points  
- Publishes coordinates to `/lane_points_3D`  
- These are then compared with Cartographer’s static map to refine SLAM-based localization  
- Improves positioning accuracy beyond LiDAR-only SLAM

---

## Directory Structure

- `YOLO/` – YOLOv7 ROS2 node, weight files, and launch scripts  
- `SCNN/` – SCNN inference code, postprocessing, depth projection, and ROS2 node  
- `launch/` – Combined ROS2 launch files for both models  
- `image/` – Test outputs, visualizations, and debugging images

---

## Results and Performance

- YOLOv7 successfully detects traffic signs in real-time (~15 FPS) on embedded hardware  
- SCNN lane detection operates stably across curves, occlusions, and shadows  
- Cartographer-based localization accuracy improved when matched with SCNN+Depth-derived coordinates  
- Integrated system showed stable control transitions (via Simulink) and improved lane-level localization

---

## Future Work

- Evaluate LaneFormer or BEV-based lane detectors for better generalization  
- Fuse additional perception outputs (e.g., curb or sidewalk) into localization  
- Apply bundle adjustment or EKF to further integrate perception-localization fusion

