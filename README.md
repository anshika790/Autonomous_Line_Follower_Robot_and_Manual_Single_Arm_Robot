# Autonomous_Line_Follower_Robot_and_Manual_Single_Arm_Robot

##  Project Overview
This repository contains the hardware logic for the **Lam Research Challenge 2025: Hardware Hustle**. The project implements a coordinated dual-robot system designed to autonomously navigate a "Smart Arena," manipulate obstacles, and manage precise fluid dynamics tasks.

The system features two distinct robotic units working in tandem:
1.  **ALFR (Autonomous Line Follower Robot):** Handles navigation and task triggering.
2.  **SARM (Single Arm Robot):** Handles obstacle removal and path clearing.

## Hardware & System Architecture

### 1. Advanced Line Follower Robot (ALFR)
* **Role:** Autonomous Navigation & Task Triggering
* **Drive Logic:** PID-based line following algorithm.
* **Behavior:** Implements "Wait & Resume" logic. The robot pauses at blocked junctions and resumes only after obstacles are cleared by the SARM.
* **Key Function:** Triggers arena automation gates (Fluid, LED, Weighing).

### 2. Single Arm Robot (SARM)
* **Role:** Obstacle Removal
* **Chassis:** Omni-directional base (Mecanum/Omni wheels) for holonomic movement.
* **Manipulator:** Multi-DOF robotic arm designed to pick and place obstacles away from the ALFR's path.
* **Control:** Manual teleoperation for precision handling.

### 3. Smart Arena (The Environment)
The arena acts as an active component of the game loop, featuring three sensor-integrated "Gates":

| Gate | Component | Function |
| :--- | :--- | :--- |
| **Gate 1** | **Peristaltic Pump** | Automatically dispenses exactly **125 ml** of fluid upon ALFR detection. |
| **Gate 2** | **LED Array** | Visual checkpoint that illuminates the "LAM" logo when triggered. |
| **Gate 3** | **Load Cell & LCD** | Final parking station. Weighs the robot + fluid to verify accuracy and displays status on an LCD. |

---

##  Game Logic & Workflow

1.  **Initialization:** Both robots spawn in the arena.
2.  **Obstacle Clearance (SARM):** The SARM identifies blocked paths and physically moves obstacles to neutral zones.
3.  **Navigation (ALFR):** The ALFR traverses the cleared line path.
4.  **Fluid Task:** ALFR triggers Gate 1 -> Pump runs -> Stops at exactly 125 ml.
5.  **Signaling:** ALFR passes Gate 2 -> LAM LED activates.
6.  **Validation:** ALFR parks at Gate 3 -> Load Cell checks weight -> LCD displays "Success" or "Error".

---

## Videos of the Game

https://drive.google.com/drive/folders/1Zcncyc2R8OWFf-8eOukFQoBXjkjSzbJk?usp=drive_link


