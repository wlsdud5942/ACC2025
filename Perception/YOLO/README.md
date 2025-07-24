# YOLOv7 – Traffic Object Detection

## Introduction

This module performs real-time object detection using forward-facing camera input to recognize various traffic signs such as stop signs, crosswalks, yield signs, and roundabouts.  
The detection results are published via ROS2 and consumed by a Simulink-based control system, where Stateflow logic uses the object class information to trigger vehicle behavior transitions (e.g., stop, slow down).

## Why YOLOv7?

Initially, the team planned to use YOLOv8 with the Ultralytics library.  
However, during integration on the **QCar2 platform (Jetson AGX Orin)**, we encountered **compatibility issues with the Ultralytics framework**, particularly related to ROS2 integration, Python environment conflicts, and inference crashes.

As a result, we migrated to **YOLOv7** for the following reasons:

- PyTorch-based, with better compatibility with ROS2  
- Lightweight "tiny" version available for real-time inference on Jetson platforms  
- Actively maintained with many open-source implementations  
- Balanced performance in terms of speed and accuracy (mAP)

## Dataset

- Images were collected using the Intel RealSense camera while driving on the actual indoor track  
- Label format follows YOLO convention:  
  `<class_index> <x_center> <y_center> <width> <height>` (all values normalized)  
- Class list:
  - 0: Stop Sign  
  - 1: Red Light 

- Dataset split: 80% train, 20% valid
- Testing was performed in real-world driving scenarios

## Training Configuration

- Model: YOLOv7-tiny  
- Epochs: 100  
- Optimizer: SGD  
- Loss functions: CIoU, BCE  
- Input image size: 640x480  
- Data augmentation: brightness variation, motion blur, horizontal flip  
- Validation accuracy: ~92% mAP@0.5

## ROS2 Integration

- **Input Topic**: `/camera/color/image_raw`  
- **Output Topic**: `/traffic_sign_topic` (`std_msgs/Int32`)  
- **Published Message**: Class index of detected object (e.g., STOP → 0)  
- **Inference Speed**: ~15 FPS on Jetson AGX Orin  
- Simulink subscribes to this topic to trigger state transitions in Stateflow

## Example Output

| Input Image | Detection Result |
|-------------|------------------|
| ![](../image/yolo_input.png) | ![](../image/yolo_output.png) |

## Simulink Integration Example

- `/traffic_sign_topic` → Connected to Simulink `Subscribe` block  
- Used in Stateflow to define transitions such as:  
  - Detecting a Stop Sign → transition from "Driving" to "Stop"

## File Structure

- `yolo_node.py`: ROS2 node for YOLOv7 inference  
- `weights/yolov7.pt`: Pretrained YOLOv7 model weights  
- `launch/yolo_launch.py`: ROS2 launch file  
- `config.yaml`: Class names and path configuration  
- `label/`: Folder containing YOLO-format annotation files  

