# Serial robot positioning error datasets: UR5 and Barrett WAM

Laser-tracker measurements of commanded against realised end-effector positions for
two serial manipulators with fundamentally different actuation: a 6-DOF gear-driven
Universal Robots UR5 and a 7-DOF cable-driven Barrett WAM.

These are the datasets behind **"A graph-based hybrid position error compensation
method for serial industrial robots"**, Yao, Jacobi, Shao and Kibireva, *Applied Soft
Computing* 203 (2026) 116099, and the Tsinghua master's thesis it grew out of. They
are released, as promised in the paper, so the results can be reproduced and built on.

- Paper: [doi.org/10.1016/j.asoc.2026.116099](https://doi.org/10.1016/j.asoc.2026.116099)
- Code: [pauljac7700/ASTGCN-robot-pose-deviation](https://github.com/pauljac7700/ASTGCN-robot-pose-deviation)

## Why these measurements are useful

Industrial robots are repeatable but not accurate. The UR5 returns to a taught point
within 0.1 mm, yet its absolute position is out by roughly 2.6 mm on average, because
the controller's kinematic model does not match the physical arm. Roughly 90% of that
gap is geometric, from manufacturing tolerance and assembly misalignment. The rest is
non-geometric and state-dependent: joint clearance, gear backlash, thermal drift,
cable elasticity.

Most published calibration work is validated on a single rigid gear-driven arm. What
makes this pair useful is the contrast: the WAM is cable-driven and starts at 17.8 mm
of error, an order of magnitude worse and with a different error structure. A method
that works on both is doing something more general than fitting one machine.

Using these datasets, the hybrid method in the paper reached:

| Robot | DOF | Drive | Uncompensated | After hybrid compensation | Reduction |
|---|---|---|---|---|---|
| Universal Robots UR5 | 6 | Gear | 2.566 mm | 0.155 mm | 94.0% |
| Barrett WAM | 7 | Cable | 17.766 mm | 2.918 mm | 83.6% |

The UR5 figure sits near that robot's own 0.1 mm repeatability, which is the practical
floor for any compensation method.

## Contents

| File | Poses | Purpose |
|---|---|---|
| `UR5/3D_UR5_uncalibrated_grid_cleaned.csv` | 1000 | Calibration, training and validation |
| `UR5/3D_UR5_uncalibrated_random_cleaned.csv` | 20 | Held-out test |
| `WAM/3D_WAM_uncalibrated_grid_cleaned.csv` | 216 | Calibration, training and validation |
| `WAM/3D_WAM_uncalibrated_random_cleaned.csv` | 20 | Held-out test |

Grid poses are distributed systematically across the operational workspace. Random
poses are sampled independently and were never seen during training, so they are the
honest generalisation test. The WAM grid is smaller than the UR5's because of
practical constraints during data collection on that platform.

All measurements are uncalibrated, meaning they record the robot as it comes, before
any geometric or learned correction. That is deliberate: it lets you evaluate your own
calibration pipeline end to end rather than inheriting ours.

## Columns

| Column | Meaning | Units |
|---|---|---|
| `step_order` | Index of the pose in the measurement sequence | integer |
| `x_t`, `y_t`, `z_t` | Commanded target position of the end-effector | mm |
| `x_dif`, `y_dif`, `z_dif` | Measured minus commanded position, the positioning error | mm |
| `joint_1` … `joint_N` | Commanded joint angles, N = 6 for the UR5 and 7 for the WAM | degrees |

The realised position is `(x_t + x_dif, y_t + y_dif, z_t + z_dif)`. Orientation was not
measured, so these are position-only datasets.

## Quick start

```python
import pandas as pd
import numpy as np

df = pd.read_csv("UR5/3D_UR5_uncalibrated_random_cleaned.csv")

joints = df[[f"joint_{i}" for i in range(1, 7)]].to_numpy()   # inputs, degrees
error  = df[["x_dif", "y_dif", "z_dif"]].to_numpy()           # targets, mm

print(f"mean absolute position error: {np.linalg.norm(error, axis=1).mean():.3f} mm")
# 2.565 mm
```

### Sanity check

Mean absolute position error straight out of each file, with no processing:

| File | Mean absolute error |
|---|---|
| `UR5/..._grid_cleaned.csv` | 2.635 mm |
| `UR5/..._random_cleaned.csv` | 2.565 mm |
| `WAM/..._grid_cleaned.csv` | 17.114 mm |
| `WAM/..._random_cleaned.csv` | 17.623 mm |

The UR5 held-out set reproduces the paper's uncompensated baseline of 2.566 mm to three
decimal places, so if your loader gives you that number, you are reading the data
correctly. Use these values to check a pipeline before trusting anything it tells you
about a new method.

## Measurement setup

Poses were recorded with a GTS3800 laser tracker, using a spherically mounted
retroreflector of 1.5 inch nominal diameter. The tracker's specified volumetric
accuracy is 15 µm + 6 µm/m, roughly two orders of magnitude finer than the errors
being measured, so tracker noise is not a limiting factor. Tool-centre frame and
base-to-tracker frame calibrations were performed before collection.

The UR5 was commanded through the Robotics Toolbox for Python. The WAM was commanded
through Libbarrett, its native C++ library.

## Citation

Please cite the paper:

```bibtex
@article{yao2026graph,
  title   = {A graph-based hybrid position error compensation method for serial industrial robots},
  author  = {Yao, Ming and Jacobi, Paul and Shao, Zhufeng and Kibireva, Anna},
  journal = {Applied Soft Computing},
  volume  = {203},
  pages   = {116099},
  year    = {2026},
  doi     = {10.1016/j.asoc.2026.116099}
}
```

Ming Yao and Paul Jacobi contributed equally and are co-first authors.

The datasets were originally collected for the master's thesis *A Hybrid Error
Compensation Method for Enhancing the Pose Accuracy of Serial Industrial Robots*,
Paul Jacobi, Department of Mechanical Engineering, Tsinghua University, 2025,
supervised by Prof. Shao Zhufeng, as part of a double degree with RWTH Aachen
University.

We gratefully acknowledge the example set by openly published calibration datasets,
in particular Landgraf et al. (2021), which informed how this release was prepared.

## License

MIT, see [LICENSE](LICENSE). Use them for anything, including commercially; an
attribution via the citation above is appreciated.

## Contact

Paul Jacobi, [paul.jacobi@rwth-aachen.de](mailto:paul.jacobi@rwth-aachen.de). Questions
and corrections are welcome as GitHub issues.
