# ACC2025

<img src="image/kdasmain.gif" alt="KDAS" width="800"/>

---

# Introduction  
This repository documents the development process and outcomes of the K-DAS Team from Kookmin University for the Quanser Student Self-Driving Car Competition held as part of the American Control Conference.  
This document provides an overview of the entire project, while details of each component are explained in their respective directories.

---

# Whole Project  
The GIFs below demonstrate the operation of each module in the project.  
Clicking on the links under each GIF will take you to documents that describe the corresponding module in more detail.

- **Control**: Receives information from the perception and decision-making systems to ultimately control the vehicle to either stop or follow a path. Research was conducted on various control algorithms, including Pure Pursuit and Stanley.

- **Perception**: Processes input data from camera sensors to recognize lane markings, traffic signs, and other elements necessary for autonomous driving.

- **Planning**: Based on the perceived information, it generates an optimal path by considering the vehicle’s current position and target waypoints. This module includes research on reinforcement learning-based algorithms such as Deep Q-Network (DQN).

- **ROS2**: Ensures reliable communication between the vehicle and the onboard computer through timestamp synchronization, QoS configuration, data interpolation, and other enhancements to improve system stability and control reliability.
