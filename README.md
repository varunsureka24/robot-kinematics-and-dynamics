# RKD Capstone Project:

## Video Demonstration
🔗 *(Add YouTube link if applicable after submission)*

## Overview
This capstone project involves programming a **Franka Emika Panda** robot to autonomously **pick up a pen, move it to a whiteboard, draw lines and curves, and place the pen in a drop bin**. The robot must successfully complete this sequence three times while adapting to different pen locations, whiteboard positions, and orientations.

The project focuses on **robot kinematics, inverse kinematics (IK), trajectory generation, and collision avoidance**, showcasing our approach to **robot motion planning in constrained environments**.

## Key Features
- **Pen Grasping & Handling** – The robot uses its gripper to pick up a pen from a holder and properly orient it for drawing.
- **Forward & Inverse Kinematics** – Implemented DH parameters for accurate kinematic transformations and IK solvers for end-effector control.
- **Trajectory Planning** – Used **joint-space and Cartesian-space interpolation** to generate smooth and precise drawing motions.
- **Time-Parameterized Motion Planning** – Implemented trapezoidal velocity profiles to ensure smooth acceleration and deceleration, avoiding instantaneous changes in speed that could lead to unstable motion.
- **Collision Avoidance** – Ensured safe movement by avoiding obstacles such as the table, pen holder, and whiteboard.
- **Drawing Execution** – Successfully traced lines and arbitrary curves while maintaining pen contact with the whiteboard.
- **Adaptive Placement** – The robot adapts to varying pen and board positions, placing the pen securely in a drop location after use.
- **Human-in-the-loop Recovery** – Allows manual inputs to recover from errors like failed grasping or misalignment.

## Project Structure
```
📂 rkd-capstone/
│── 📁 src/            # Source code for motion planning, kinematics, and control
│── 📁 config/         # Configuration files, calibration parameters
│── 📄 README.md       # This file
```

## Demo Execution
- The robot follows a **predefined trajectory** to draw a shape on the whiteboard.
- The script allows manual input for recovery policies in case of failures.
