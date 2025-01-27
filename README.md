# Flexobot-Robotics-Arm

This repository showcases a 6-degree-of-freedom (6-DOF) robotic arm designed and built from scratch. It includes details of the 3D design, ROS2 integration, MoveIt2 configuration, and joystick control setup. Below are the key aspects of the project:

---

## Table of Contents
1. [3D Model Design](#3d-model-design)
2. [ROS2 Framework Configuration](#ros2-framework-configuration)
3. [MoveIt2 Framework Configuration](#moveit2-framework-configuration)
4. [Joystick Control](#joystick-control)


---

## 3D Model Design
The robotic arm was designed in SolidWorks. The design process focused on achieving precision, durability, and smooth motion across all six degrees of freedom. Below are key highlights:

- **Joints and Axes**: The arm includes six rotational joints, allowing a wide range of motion.
- **Actuators**: Powered by three MG996R and three MG90S servo motors.

Key Features:
- Modular assembly for easy maintenance.
- Custom-designed brackets and holders for servos and sensors.
- Compact and efficient design to minimize weight and maximize stability.



https://github.com/user-attachments/assets/8f252433-98d1-481b-a00f-6d7066f3ebcc



---

## ROS2 Framework Configuration
The robotic arm was integrated with ROS2, enabling advanced robotic functionalities. Some key features include:

### 1. Flexobot Description Package
- **URDF File Creation**: Designed a URDF file using SolidWorks for accurate representation of the robotic arm. ![URDF Design](images/urdf_design.png)
- **RViz Visualization**: Created RViz2 visualization with Joint State Publisher GUI for real-time visualization. [Video](videos/rviz2_visualization.mp4)

### 2. Flexobot Control Package
- **Joint Trajectory Controller**: Configured the joint trajectory controller in the ROS2 control system for both the robotic arm and the gripper.

- [Screencast from 01-27-2025 09:39:27 AM.webm](https://github.com/user-attachments/assets/1eb9428e-02c6-4f70-a86b-f1c98212eb7b)


---

## MoveIt2 Framework Configuration
Using MoveIt2, the robotic arm is capable of path planning and motion execution. The configuration includes:

- **Flexobot MoveIt Package**: Configured the MoveIt configuration files, including SRDF, kinematic solver, MoveIt controller, joint limits, and other essential parameters.
- **Launch MoveIt with GUI**: Enabled MoveIt GUI for interactive motion planning and debugging.

  
[Screencast from 01-27-2025 09:42:30 AM.webm](https://github.com/user-attachments/assets/832c6697-161f-4ad3-8f15-f82fa94ecf4c)

---

## Joystick Control
A joystick controller provides an intuitive way to operate the robotic arm in real-time. Features include:

- Mapping of joystick axes to joint movements.
- Adjustable speed control for precise operations. (TODO)
---

## Hardware Testing
The fully assembled robotic arm is currently in the testing phase. Key focus areas include:

- **Actuator Calibration**: Verifying servo motor functionality and ensuring precise joint control.
- **Stability Testing**: Assessing the arm’s ability to maintain stable poses under load.
- **Real-World Validation**: Testing the integration of ROS2 and MoveIt2 configurations with the physical hardware.
- **Joystick Control Validation**: Ensuring smooth and intuitive real-time control of the arm via the joystick.

Preliminary results indicate smooth motion and accurate path execution. However, fine-tuning is ongoing to optimize performance and reliability.




https://github.com/user-attachments/assets/b326969a-dba4-4c95-a78f-aac0ed1b5916





