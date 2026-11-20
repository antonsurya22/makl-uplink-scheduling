# P0-J: schedulers and unit tests

Package `SCI_P0J_SCHEDULERS_20261111` (offline, no ROS).

- `j_links.py`: FQ-CoDel (B4), max-age-first (B5), priority + keep-last (B7) and MAKL, on top of the link model of
  the earlier experiments (`frozen_deps/`)
- `j_replay.py`: open-loop replay of recorded traces through a scheduler, with the age-speed violation metric
- `tests/test_p0j.py`: unit tests T1-T7 (table of a*, ledger conservation, Propositions 1 and 2, MAKL limits, replay
  equivalence, decision time, bridge interface)
- `j0_explore.py`: first version of J0; the paper uses `analysis/exploratory/j0_explore_v2.py`

```bash
python3 -I -B experiments/P0J_schedulers/SCI_P0J_SCHEDULERS_20261111/tests/test_p0j.py
```

T5 needs the exported traces from Zenodo (`test_p0j.py TRACES_DIR`). `results/` has the test output of the frozen run
and the J0 v1 summary.
