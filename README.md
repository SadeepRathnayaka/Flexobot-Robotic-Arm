# Flexobot-Robotics-Arm

This repository showcases a 6-degree-of-freedom (6-DOF) robotic arm designed and built from scratch. It includes details of the 3D design, ROS2 integration, MoveIt2 configuration, and joystick control setup. Below are the key aspects of the project:

---

## Table of Contents
1. [3D Model Design](#3d-model-design)
2. [ROS2 Framework Configuration](#ros2-framework-configuration)
3. [MoveIt2 Framework Configuration](#moveit2-framework-configuration)
4. [Joystick Control](#joystick-control)
5. [Media](#media)
6. [How to Run](#how-to-run)
7. [Contributing](#contributing)

---

## 3D Model Design
The robotic arm was designed in SolidWorks. The design process focused on achieving precision, durability, and smooth motion across all six degrees of freedom. Below are key highlights:

- **Joints and Axes**: The arm includes six rotational joints, allowing a wide range of motion.
- **Actuators**: Powered by three MG996R and three MG90S servo motors.

![SolidWorks Design](images/solidworks_design.png)

Key Features:
- Modular assembly for easy maintenance.
- Custom-designed brackets and holders for servos and sensors.
- Compact and efficient design to minimize weight and maximize stability.

---

## ROS2 Framework Configuration
The robotic arm was integrated with ROS2, enabling advanced robotic functionalities. Some key features include:

### 1. Flexobot Description Package
- **URDF File Creation**: Designed a URDF file using SolidWorks for accurate representation of the robotic arm. ![URDF Design](images/urdf_design.png)
- **RViz Visualization**: Created RViz2 visualization with Joint State Publisher GUI for real-time visualization. [Video](videos/rviz2_visualization.mp4)

### 2. Flexobot Control Package
- **Joint Trajectory Controller**: Configured the joint trajectory controller in the ROS2 control system for both the robotic arm and the gripper. [Video](videos/joint_trajectory_controller.mp4)

---

## MoveIt2 Framework Configuration
Using MoveIt2, the robotic arm is capable of path planning and motion execution. The configuration includes:

- **Flexobot MoveIt Package**: Configured the MoveIt configuration files, including SRDF, kinematic solver, MoveIt controller, joint limits, and other essential parameters.
- **Launch MoveIt with GUI**: Enabled MoveIt GUI for interactive motion planning and debugging. [Video](videos/moveit_gui_demo.mp4)

---

## Joystick Control
A joystick controller provides an intuitive way to operate the robotic arm in real-time. Features include:

- Mapping of joystick axes to joint movements.
- Adjustable speed control for precise operations. (TODO)
---

## Media
### Images
- **SolidWorks Design**
  ![SolidWorks Design](images/solidworks_design.png)

- **Assembled Robotic Arm**
  ![Assembled Arm](images/assembled_arm.jpg)

### Videos
- [Motion Demonstration](videos/motion_demo.mp4)
- [Path Planning in MoveIt2](videos/path_planning.mp4)

---

## How to Run
### Prerequisites
1. Install [ROS2](https://docs.ros.org/en/rolling/Installation.html).
2. Install MoveIt2.
3. Clone this repository:
   ```bash
   git clone https://github.com/your_username/6dof_robotic_arm.git
   ```
4. Build the workspace:
   ```bash
   colcon build
   ```

### Running the Robot
1. Launch the ROS2 control node:
   ```bash
   ros2 launch robotic_arm_control control.launch.py
   ```
2. Launch MoveIt2 for motion planning:
   ```bash
   ros2 launch moveit2_config demo.launch.py
   ```
3. Connect the joystick and run the joystick control node:
   ```bash
   ros2 run joystick_control joystick_node
   ```

---

## Contributing
Contributions are welcome! Feel free to submit issues or pull requests for improvements and additional features.


