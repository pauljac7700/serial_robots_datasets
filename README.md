# Datasets for Hybrid Error Compensation in Serial Industrial Robots

## Introduction

This repository contains the experimental datasets collected for the Master's thesis titled **"A Hybrid Error Compensation Method for Enhancing the Pose Accuracy of Serial Industrial Robots"** by Paul Jacobi, submitted to Tsinghua University (Department of Mechanical Engineering) in May 2025, as part of a double degree program with RWTH Aachen University. The research was supervised by Professor Shao Zhufeng.

The primary goal of releasing these datasets is to provide a benchmark for the robotics research community, fostering further advancements in robot calibration, error compensation techniques, and the development of learning-based models for robotics. We gratefully acknowledge the inspiration from publicly available datasets like that of Landgraf et al. (2021) in our own work.

## Thesis Abstract

With advancing industrial automation, industrial robots are increasingly utilized in manufacturing, assembly, precision machining, and other fields. Although industrial robots usually have excellent repetitive positioning accuracy, their absolute positioning accuracy often fails to meet the requirements of high‑precision tasks, where positional errors can lead to significant performance degradation and operational instability. Improving the absolute positioning accuracy of industrial robots has thus become an important research direction to ensure the reliability and flexibility of intelligent manufacturing.

To address these problems, this paper proposes a hybrid error compensation framework that innovatively combines the modified Denavit–Hartenberg (MDH) geometric calibration method with an Attention‑based Spatial‑Temporal Graph Convolutional Network (ASTGCN), aiming to comprehensively improve the positioning accuracy of serial industrial robots. The approach comprises two phases: first, an accurate geometric kinematic model is constructed via MDH calibration for modeling and compensating geometric errors. Second, a non‑geometric positional residual map, informed by the serial kinematic structure, is developed. An improved ASTGCN then models and predictively compensates for residual non‑geometric errors (e.g., joint clearance, thermal deformation, control system lag), thereby creating a high‑precision compensation mechanism that integrates mechanistic and data‑driven models.

Experimental validation was performed on two types of industrial robots with different drive mechanisms and degree‑of‑freedom configurations: a 6‑DOF gear‑driven Universal Robots UR5 and a 7‑DOF cable‑driven Barrett WAM. Using high‑precision measurements from a laser tracker, the proposed hybrid method was comparatively evaluated against baseline conditions: uncalibrated performance, MDH calibration alone, and ASTGCN compensation alone. The results demonstrate that the proposed hybrid compensation method significantly improves positioning accuracy for both robots. The average absolute positional error for the UR5 was reduced from $2.5664 \text{ mm}$ to $0.1549 \text{ mm}$, approaching its repeatability of $0.1 \text{ mm}$. For the WAM, the error was reduced from $17.7661 \text{ mm}$ to $2.9178 \text{ mm}$, showcasing the method’s versatility and robustness. Finally, the robot dataset used in this study is publicly released to provide foundational data for related research.

In summary, this paper demonstrates that the organic integration of mechanism‑based kinematic calibration with a deep neural network possessing spatio‑temporal modeling capabilities can effectively enhance the absolute positioning accuracy of industrial robots. This approach offers a feasible and viable technical path for deploying industrial robots in high‑precision applications.

## Robotic Platforms

Experiments were conducted on two distinct serial robotic manipulators:

1.  **Universal Robots UR5:**
    * A 6-degrees-of-freedom (DOF) collaborative robotic arm (gear-driven).
    * Manufacturer-specified repeatability: $\pm 0.1 \text{ mm}$.
    * Controlled using the Robotics Toolbox for Python.
2.  **Barrett WAM (Whole Arm Manipulator):**
    * A 7-DOF cable-driven robotic arm.
    * Manufacturer-specified repeatability: $2 \text{ mm}$.
    * Controlled using Libbarrett (native C++ library).

## Measurement System

All end-effector pose data was acquired using a **GTS3800 laser tracker system** with a Spherically Mounted Retroreflector (SMR) of nominal diameter 1.5 inches (approximately $38.1 \text{ mm}$). The laser tracker has a specified volumetric accuracy of $15 \text{ µm} + 6 \text{ µm/m}$. Tool-Center Frame (TCF) and robot base frame to tracker frame calibrations were performed prior to data collection.

## Dataset Description

For each robot, the following datasets are provided:

### 1. Calibration and Training/Validation Dataset (Grid-like Poses)

* **UR5:** 1000 poses distributed in a grid-like pattern across its operational workspace.
* **WAM:** 215 poses distributed in a grid-like pattern across its operational workspace. (The reduced number for WAM was due to practical experimental constraints detailed in the thesis.)

### 2. Test Dataset (Random Poses)

* **UR5:** 20 distinct poses, randomly sampled within the operational workspace, unseen during training.
* **WAM:** 20 distinct poses, randomly sampled within the operational workspace, unseen during training.

## Data Format

Data is primarily provided in CSV (`.csv`) format. The columns in these files generally follow the structure below:

* `step_order`: An integer representing the sequence or index of the pose measurement.
* `x_t`, `y_t`, `z_t`: The X, Y, and Z coordinates of the **desired target position** of the robot's end-effector. Units are in **millimeters (mm)**.
* `x_dif`, `y_dif`, `z_dif`: The difference or residual error between the target position and the actual achieved position. Units are in **millimeters (mm)**.
* `joint_1`, `joint_2`, ..., `joint_N`: The **commanded joint angles** for each joint of the robot (where N is 6 for UR5 and 7 for WAM). Units are in **degrees (°)**.

## How to Use

These datasets can be used for:
* Benchmarking novel robot calibration algorithms (geometric, non-geometric, or hybrid).
* Training and evaluating machine learning models for robot error prediction and compensation.
* Replicating and extending the results presented in the associated thesis.
* Educational purposes in robotics and machine learning courses.

## Citation

If you use these datasets in your research, please cite the following thesis:

Jacobi, P. (2025). *A Hybrid Error Compensation Method for Enhancing the Pose Accuracy of Serial Industrial Robots*. Master's Thesis, Department of Mechanical Engineering, Tsinghua University.

## Contact and Issues
For any questions regarding these datasets or to report issues, please contact Paul Jacobi at paul.jacobi@rwth-aachen.de or open an issue in this GitLab repository.

## License
This dataset is made available under the MIT License. Please see the LICENSE file in this repository.
