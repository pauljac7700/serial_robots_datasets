# Datasets for Hybrid Error Compensation in Serial Industrial Robots

## Introduction

This repository contains the experimental datasets collected for the Master's thesis titled **"A Hybrid Error Compensation Method for Enhancing the Pose Accuracy of Serial Industrial Robots"** by Paul Jacobi, submitted to Tsinghua University (Department of Mechanical Engineering) in May 2025, as part of a double degree program with RWTH Aachen University. The research was supervised by Professor Shao Zhufeng.

The primary goal of releasing these datasets is to provide a benchmark for the robotics research community, fostering further advancements in robot calibration, error compensation techniques, and the development of learning-based models for robotics. We gratefully acknowledge the inspiration from publicly available datasets like that of Landgraf et al. (2021) in our own work.

## Thesis Abstract

Industrial robots, despite high repeatability, often exhibit insufficient absolute positioning accuracy, limiting their use in precision-critical applications. This thesis addresses this by developing and evaluating an innovative hybrid error compensation model to significantly enhance the pose accuracy of serial industrial robots. The objective was to systematically address both geometric and non-geometric errors by integrating interpretable parametric kinematic modeling (Modified Denavit-Hartenberg - MDH) with a modern deep learning architecture (Attention-based Spatial-Temporal Graph Convolutional Network - ASTGCN).

The proposed hybrid framework features a two-stage process: MDH calibration first establishes an accurate geometric baseline, after which a specifically adapted ASTGCN predicts and compensates for remaining complex residual errors. This MDH-ASTGCN methodology was experimentally validated on two distinct serial manipulators—a 6-DOF gear-driven Universal Robots UR5 and a 7-DOF cable-driven Barrett WAM—using laser tracker measurements, and benchmarked against uncalibrated states and standalone MDH or ASTGCN approaches.

Experimental results confirm that the hybrid MDH-ASTGCN framework yields substantial accuracy improvements for both robots, outperforming baseline methods. For the UR5, mean Euclidean error was reduced from $2.5664 \text{ mm}$ to $0.1549 \text{ mm}$, approaching its $0.1 \text{ mm}$ repeatability. For the more challenging WAM, error was reduced from $17.7661 \text{ mm}$ to $2.9178 \text{ mm}$, a significant improvement, though still outside its $2 \text{ mm}$ repeatability. Key contributions include the novel design and validation of this specific MDH-ASTGCN model for serial robots, the architectural adaptation of ASTGCN for this task, insights from cross-platform validation, and the public release of experimental datasets.

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

* **Content for each point in these datasets typically includes:**
    * Desired Target Pose ($P_{target}$) (X, Y, Z coordinates).
    * Nominal Commanded Joint Angles ($\theta_{cmd}$) derived via $IK(\cdot, k_{nom})$ to reach $P_{target}$.
    * Measured Actual End-Effector Pose ($P_{actual, nom}$) when $\theta_{cmd}$ is executed on the uncalibrated robot.
    * Calculated "MDH Residuals" ($\Delta P_{residual, MDH}$ = $P_{target}$ - $FK$($\theta_{cmd}$, $k_{cal}$)) after MDH geometric calibration parameters ($k_{cal}$) have been identified using the $(\theta_{cmd}, P_{actual, nom})$ pairs. These residuals were used as targets for ASTGCN training.
    * The identified $k_{cal}$ parameters for each robot are provided in the thesis (Tables 4.2 and 4.5 respectively).

* **Purpose:**
    * The $(\theta_{cmd}, P_{actual, nom})$ data was used for identifying the MDH parameters ($k_{cal}$).
    * The dataset of (features from $P_{target}, \theta_{cmd}$, and the target $\Delta P_{residual, MDH}$) was used for training (90%) and validating (10%) the ASTGCN model.

### 2. Test Dataset (Random Poses)

* **UR5:** 20 distinct poses, randomly sampled within the operational workspace, unseen during training.
* **WAM:** 20 distinct poses, randomly sampled within the operational workspace, unseen during training.

* **Content for each point in these datasets typically includes:**
    * Desired Target Pose ($P_{target}$) (X, Y, Z coordinates).
    * Measured Actual End-Effector Pose under different compensation schemes:
        * No Calibration (using $k_{nom}$).
        * MDH-Only compensation (using $k_{cal}$).
        * ASTGCN-Only compensation (ASTGCN trained on uncalibrated residuals).
        * Hybrid (MDH + ASTGCN) compensation.

* **Purpose:** Used for the final evaluation and comparison of all error compensation methods.

## Data Format

Data is primarily provided in CSV (`.csv`) format. The columns in these files generally follow the structure below:

* `step_order`: An integer representing the sequence or index of the pose measurement.
* `x_t`, `y_t`, `z_t`: The X, Y, and Z coordinates of the **desired target position** of the robot's end-effector. Units are in **millimeters (mm)**.
* `x_dif`, `y_dif`, `z_dif`: The difference or residual error between the target position and the actual achieved position (or a predicted position, depending on the specific dataset file). The precise definition (e.g., $P_{target} - P_{actual}$ or $P_{target} - P_{predicted}$) for these difference columns is detailed in the thesis, particularly in the context of how $\Delta P_{residual, MDH}$ is calculated (Section 4.1.2.1) and how ASTGCN inputs/outputs are structured. Units are in **millimeters (mm)**.
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
