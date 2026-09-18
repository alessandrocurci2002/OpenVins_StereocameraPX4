# introduction
in this folder, named sc_fucina02, there will be the evaluation material related to the dataset collected on the 31/07/2026.

# Running the ov_eval trajectory evaluation

## 1. Directory layout

All paths below are relative to `OpenVINS_eval/`.

```text
OpenVINS_eval/
├── truths/
│   └── <test>.txt                      # ground truth (TagSLAM), one file per test
├── algorithms/
│   └── <config>/                       # one folder per OpenVINS configuration
│       └── <test>/                     # must match the ground-truth file name (without .txt)
│           └── run_N.txt               # one file per run
├── rosbags/<test>/<config>/            # raw OpenVINS outputs, configs, calibration
├── results/                            # saved console output of the evaluation tools
├── extract_gt.py                       # TagSLAM bag -> ground-truth file
└── state_to_pose.py                    # OpenVINS state dump -> ov_eval trajectory file
```

The `<test>` folder name under each algorithm must match the ground-truth file name exactly. If it does not, `error_dataset` and `error_comparison` silently skip that algorithm.

## 2. File format

ov_eval reads space-separated text files, one pose per line, sorted by increasing time:

```text
# timestamp(s) tx ty tz qx qy qz qw [Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33]
```

- The first 8 columns are required.
- The 12 optional covariance columns (upper triangle of the 3x3 orientation and position covariances) are only needed for NEES.
- The ground truth may omit the covariance. It is then treated as exact.
- Estimate and ground truth must share the same time base.

## 3. Producing the input files

**Ground truth** (from the TagSLAM output bag):

```bash
python3 extract_gt.py <tagslam_output>.bag --topic /tagslam/odom/body_rig --out truths/<test>.txt
```

**OpenVINS estimate.** Run OpenVINS with `save_total_state:=true` (`make openvins_saveall`). This writes `ov_estimate.txt` and `ov_estimate_std.txt`. These are not ov_eval-compatible as they are: the columns are ordered `t qx qy qz qw px py pz ...` and include extra state columns. Feeding them in directly does not crash, but silently misparses the columns and gives meaningless numbers. Convert them:

```bash
python3 state_to_pose.py rosbags/<test>/<config>/ov_estimate.txt \
    --std rosbags/<test>/<config>/ov_estimate_std.txt \
    --out algorithms/<config>/<test>/run_1.txt
```

Omit `--std` to write a pose-only file, which gives no NEES. Passing `--std` adds a diagonal covariance to the estimate.

## 4. Running the evaluation

Run inside the ROS 2 environment, after `colcon build --packages-select ov_eval` and `source install/setup.bash`:

```bash
cd <...>/OpenVINS_eval

# Single run: ground truth, then estimate
ros2 run ov_eval error_singlerun posyaw truths/<test>.txt algorithms/<config>/<test>/run_1.txt \
    | tee results/<test>_<config>_singlerun.txt

# One dataset, all algorithms and runs (averaged over runs, LaTeX table)
ros2 run ov_eval error_dataset posyaw truths/<test>.txt algorithms/ \
    | tee results/<test>_error_dataset.txt

# All datasets and algorithms (per-dataset and average LaTeX tables)
ros2 run ov_eval error_comparison posyaw truths/ algorithms/ \
    | tee results/error_comparison.txt

# Trajectory plot: XY and Z over time (needs a display)
ros2 run ov_eval plot_trajectories posyaw truths/<test>.txt algorithms/<config>/<test>/run_1.txt
```

## 5. How the tools behave

- **Alignment mode.** The first argument selects how the estimate is aligned to the ground truth:
  - `posyaw`: translation plus yaw, all poses (suitable for gravity-aligned VIO)
  - `posyawsingle`: translation plus yaw, first pose only
  - `se3`: full rotation plus translation
  - `se3single`: same, first pose only
  - `sim3`: also estimates scale
  - `none`: no alignment
- **Different trajectory lengths.** Timestamps are matched by nearest neighbour within 20 ms. Ground-truth poses before OpenVINS finishes initialising (about 1.6 s here) are dropped automatically, so no manual trimming is needed.
- **Metrics.** The tools report:
  - ATE (3D and 2D)
  - RPE
  - RMSE
  - NEES (only if the estimate has covariance columns)
- **RPE segment lengths.** These are hard-coded in `error_singlerun.cpp`, `error_dataset.cpp` and `error_comparison.cpp`. Defaults are 7-35 m for `error_dataset`, 8-40 m for `error_singlerun` and 8-48 m for `error_comparison`. If the trajectory is shorter than the largest segment, those bins report 0 samples. Edit the `segments` vector and rebuild `ov_eval` to fit your trajectory length.
- **Output files.** The tools never write files themselves. Results go to the console, and `plot_trajectories` opens an interactive window. Use `| tee results/<name>.txt` to keep a copy of the console output.
