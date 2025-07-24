# ROS2 Integration Overview(완전 초안)

## Introduction

This directory documents the ROS2-based communication architecture used to integrate perception, path planning, and control modules on a real autonomous vehicle.  
All processing nodes are interconnected through lightweight ROS2 topics, and the control system—originally built in Simulink—is deployed as a C++ ROS2 node via code generation.

SLAM and localization are handled by Cartographer. In the real-vehicle implementation, IMU data was additionally fused to enhance pose accuracy.

---

## System Architecture

- **Sensors**: Intel RealSense D435i (RGB + Depth), RPLiDAR, IMU  
- **Perception Nodes**: YOLOv7 (object detection), SCNN (lane centerline detection)  
- **Path Distribution**: HelperPathSender (DQN-based path sender), AreaChecker  
- **Control Node**: Simulink control logic (converted to C++ ROS2 node)  
- **Localization**: Cartographer SLAM (LiDAR + IMU)

All nodes communicate through ROS2 topics using compact message types like `Float32`, `Int32`, `Point`, and `Bool`, supporting modular real-time execution.

---

## Cartographer and IMU Integration

| Aspect        | Simulation                          | Real Vehicle                      |
|---------------|-------------------------------------|-----------------------------------|
| Sensors used  | LiDAR only                          | LiDAR + IMU                       |
| IMU usage     | ❌ Disabled due to noise/sync issues | ✅ Enabled for improved accuracy   |
| Config changes| Default launch                      | Enabled `use_imu_data = true`, added `imu.lua`, adjusted topic remaps |
| Result        | 10–20 cm drift in loop closures      | Reduced to ~5 cm positional error |

> **Note**:  
> In simulation, the IMU sensor output was too unstable for SLAM, so Cartographer was used with LiDAR only.  
> In the real-vehicle system, we **enabled IMU integration** and modified Cartographer parameters to use `/imu` data for better pose estimation.  
> This significantly improved localization accuracy and stability, especially in curves and loop closures.

---

## Topic Structure & QoS Configuration

| Topic                   | Type                   | QoS Settings                      | Description                             |
|-------------------------|------------------------|-----------------------------------|-----------------------------------------|
| `/path_x`, `/path_y`    | `Float32`              | depth=10, reliable                | Waypoint coordinates                    |
| `/lane_points_3D`       | `Float32MultiArray` or `PointCloud` | depth=10, best_effort | SCNN-projected lane centerline         |
| `/traffic_sign_topic`   | `Int32`                | depth=5, transient_local          | Detected traffic sign index             |
| `/pickup_dropoff`, `/stop` | `Int32`            | depth=5, reliable                 | Pickup/Dropoff trigger and stop signal  |
| `/imu`                  | `sensor_msgs/Imu`      | keep_last=10, reliable            | Used by Cartographer in real vehicle    |

---

## Time Synchronization Issues & Fixes

- **Problem**: Mismatched timestamps across RealSense, LiDAR, and Cartographer led to `/tf` initialization failures.
- **Fixes**:
  - Always launch Cartographer **last** to allow full TF tree construction  
  - Used `--use_sim_time` and adjusted frame broadcasts to align clocks  
  - For IMU sync, applied `imu_time_offset` and verified alignment using `ros2 topic echo`  

---

## Lessons Learned / Practical Fixes

| Issue                                           | Resolution                                      |
|------------------------------------------------|-------------------------------------------------|
| Cartographer launch fails (no map data)         | Launch Cartographer last                        |
| TF frame missing                                | Added `custom_tf_broadcaster` utility node      |
| Simulink ROS2 node not receiving topics         | Set explicit QoS: `RELIABLE`, `depth=10`        |
| IMU drift caused incorrect SLAM poses           | Tuned IMU parameters and added `imu.lua` config |

---

## Directory Structure

- `datahelper/`: Helper nodes for waypoint publishing, stop control, and DQN sequencing  
- `launch/`: ROS2 launch files for the full system  
- `scripts/`: Experiment logging and automation scripts  
- `cartographer_config/`: Real-vehicle Cartographer config with IMU support

