# Exploratory replays

Not fixed in advance; marked as exploratory in the paper.

- `j0_explore_v2.py`: J0, replay of the recorded E3-B, E5 and E6 traces through all schedulers at 0.10-0.25 Mbit/s,
  used to choose MAKL's parameters. Output in `results/j0/`.
- `dswitch_replay.py`: distance switch and fixed-budget MAKL in the same replay, used to choose the J3 thresholds.
  Output in `results/dswitch/`.

Both use the P0-J code in `experiments/P0J_schedulers/`. The traces are in the P0-J results archive on Zenodo.

```bash
python3 -I -B analysis/exploratory/j0_explore_v2.py TRACES_DIR OUT_DIR
python3 -I -B analysis/exploratory/dswitch_replay.py TRACES_DIR OUT_DIR
```

Running them again gives the same rows; only the timing columns (`wall_s`, `cpu_med_us`) change.
