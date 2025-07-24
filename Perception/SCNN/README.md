# SCNN – Lane Detection and Localization Refinement via Centerline Extraction

## Introduction

This module performs real-time lane detection from RGB camera input using the SCNN model.  
It extracts the centerline of the lane from segmentation masks and converts the pixel coordinates into real-world (x, y) positions using depth data.  
The resulting 3D coordinates are then used to refine the vehicle’s localization accuracy by aligning with Cartographer SLAM maps.

## Why SCNN?

- Compared to other models (e.g., Ultra-Fast Lane Detection, PolyLaneNet), SCNN offers strong robustness against:
  - broken lanes  
  - curved lanes  
  - partially occluded markings  
- Simple architecture suitable for real-time inference  
- Clean binary segmentation output enables reliable postprocessing

## Dataset

- Collected using RGB camera in an indoor test track environment  
- Manually labeled binary masks: white pixels for lane, black for background  
- Format: 1-channel binary mask (0/1)  
- Split: 80% for training, 20% for validation  
- Resolution: 640x480, Frame rate: 30 FPS

## Postprocessing Pipeline

1. Apply row-wise DBSCAN clustering to the SCNN output mask  
2. Fit 3rd-degree polynomial curves to left and right lane clusters  
3. Compute the centerline by averaging both curves  
4. Use depth data to convert each centerline pixel to real-world (x, y) coordinates  
5. Publish the results via the `/lane_points_3D` ROS2 topic

## ROS2 Integration(수정필요)

- **Input Topics**:  
  - `/camera/color/image_raw` (RGB image)  
  - `/camera/depth/image_raw` (Depth image)  

- **Output Topic**:  
  - `/lane_points_3D` (`Float32MultiArray` or `geometry_msgs/PointCloud`)  

- **Inference Rate**: 10–15 Hz  
- **Hardware**: Intel RealSense D435i + Jetson Orin

## Test Results

- The SCNN model demonstrated stable detection performance across curves, shadows, and partial occlusions  
- Extracted centerlines aligned well with static structures in Cartographer SLAM maps (e.g., lane edges, walls)  
- Real-world tests showed improved localization accuracy by approximately 3–5 cm over Cartographer-only estimates  
- (Optional) Extracted centerlines can also be used directly as inputs to control algorithms such as Pure Pursuit

## Example Outputs(수정필요)

| Step | Image |
|------|--------|
| SCNN mask output | ![](../image/scnn_mask.png) |
| DBSCAN clustering | ![](../image/scnn_dbscan.png) |
| Final centerline with (x, y) projection | ![](../image/scnn_centerline_xyz.png) |
