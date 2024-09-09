# Robotic Manipulator with 3 Rotational Degrees of Freedom (DOF)

## Project Overview

This project is part of the Robotics I course (2022-2023) and involves the analysis and simulation of a robotic manipulator with three rotational degrees of freedom. The manipulator's motion is analyzed through forward and inverse kinematics, and its trajectory is simulated using MATLAB.

## Table of Contents
- [Objectives](#objectives)
- [Simulation Tasks](#simulation-tasks)
- [Installation](#installation)

## Objectives

The project focuses on analyzing the robotic manipulator's kinematics and simulating its motion between two points on a horizontal plane. The manipulator has 3 rotational degrees of freedom, and the simulation involves applying the Denavit-Hartenberg (D-H) method for kinematic analysis, as well as calculating forward and inverse kinematics.

Key analysis steps include:
1. **Denavit-Hartenberg Analysis:**  
   Define reference frames and obtain the DH parameter table.
2. **Forward Kinematics:**  
   Compute the position and orientation of the end effector.
3. **Jacobian Matrix:**  
   Derive the Jacobian matrix for the manipulator.
4. **Singular Configurations:**  
   Identify singular configurations based on the differential kinematics.
5. **Inverse Kinematics:**  
   Calculate the joint angles given the desired position of the end effector.

## Simulation Tasks

Using MATLAB, the following tasks are simulated:
1. **Trajectory Generation:**  
   Simulate the periodic linear motion of the end effector between two points A and B on a horizontal plane, maintaining a height `h` from the reference base.
   
2. **Kinematic Profiles:**  
   - Plot the desired trajectory of the end effector in terms of position and velocity over time.
   - Plot the joint angles `{q1, q2, q3}` and joint velocities `{q1_dot, q2_dot, q3_dot}` over time.
   
3. **Animation:**  
   Create an animation showing the motion of the robotic manipulator as it moves between points A and B, showing intermediate configurations.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/gchar00/Robotics.git

