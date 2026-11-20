# makl-uplink-scheduling

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.24383545.svg)](https://doi.org/10.5281/zenodo.24383545)

Code and data for our paper:

Y.-C. Tung, I G. N. A. S. Diputra, Y.-N. Lin, V. R. L. Shen. *Margin-aware uplink scheduling for edge-assisted mobile robots:
bounding the waiting time of safety-relevant sensor data.*

When a robot sends LiDAR scans and camera frames to an edge computer over one congested uplink, the scans used by
the planner can become several seconds old. MAKL (Margin-Aware Keep-Last) is a scheduler on the robot side. It keeps
only the newest message of each stream and sends a camera frame only if the predicted scan age at the planner stays
below a budget computed from the robot's speed and its distance to the nearest obstacle.

The repo has the schedulers, the ROS 2 / Nav2 simulation stack, the test on a real Wi-Fi link, and all experiment
packages as they were frozen before the runs.

## Contents

```
experiments/          one folder per experiment (P0-J, J1-J5b): frozen package + results/
analysis/exploratory/ offline replays used to choose parameters (J0) and the J3 thresholds
ros2_ws/src/          ROS 2 package network_aware_nav (uplink emulator, safety layer, Nav2 config, worlds)
reproduce/            offline checks, figure and table scripts
tools/                freeze check, manifest, pre-release scan, Zenodo staging
docs/                 experiment list, environment, Wi-Fi test bed, how to reproduce
```

Raw logs of all runs and the packages of the earlier experiments E1-E8 are on Zenodo (https://doi.org/10.5281/zenodo.24383545).

## Quick check

Needs only Python 3.10+ (no ROS), takes about 15 s:

```bash
git clone https://github.com/antonsurya22/makl-uplink-scheduling
cd makl-uplink-scheduling
bash reproduce/run_checks.sh
```

It checks every package against its hash file and runs the scheduler unit tests and the offline tests of J5/J5b.

The closed-loop experiments (J1-J4) need Ubuntu 22.04 with ROS 2 Humble, Gazebo Classic 11 and Nav2
(see `docs/ENVIRONMENT.md`). J5 and J5b need two machines on one Wi-Fi network (`docs/NETWORK_TESTBED.md`).

## Experiments

| | Question | Runs | Folder |
|---|---|---|---|
| P0-J | scheduler code and unit tests | offline | `experiments/P0J_schedulers` |
| J0 | offline replay to choose MAKL's parameters (exploratory) | 300 traces | `analysis/exploratory` |
| J1 | MAKL vs. seven baselines, closed loop | 240 | `experiments/J1_closed_loop` |
| J2 | jitter, random/burst loss, varying capacity | 252 | `experiments/J2_link_impairments` |
| J3 | fixed distance switch vs. MAKL | 75 | `experiments/J3_distance_switch` |
| J4 | effect of the speed term | 75 | `experiments/J4_speed` |
| J5 | real ROS 2 / Wi-Fi stack, limit in the scheduler or in the kernel | 120 replays | `experiments/J5_network_stack` |
| J5b | pacing at 0.75c on the real stack | 40 (+20) replays | `experiments/J5b_pacing` |

`docs/EXPERIMENTS.md` lists the package, protocol and hash file of each experiment, including E1-E8.

## How the experiments were run

Before the first counted run of each experiment, we wrote down the protocol (hypotheses, thresholds, analysis) and
saved the SHA256 of every file in the package. The runner stops if a file has changed. Changes after that point were
written as amendments and are part of the package. We publish the packages unchanged so the hashes can still be
checked (`python3 tools/verify_freezes.py`). For the same reason some names from our working files were kept: the
run scripts are called `JALANKAN_*.sh` ("run" in Indonesian), and three of the earlier experiments have different
names in the archive than in the paper (see `docs/EXPERIMENTS.md`).

## Citation and license

Please cite the paper and the Zenodo record (`CITATION.cff`).
Code: MIT. Result tables and data: CC BY 4.0.
