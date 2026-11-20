# reproduce

- `run_checks.sh` - freeze check, P0-J unit tests, J5/J5b offline tests (no ROS)
- `figures/` - `make_fig_*.py`, one script per figure
- `tables/` - `gen_esm_*.py`, tables of the supplementary material

The scripts read `experiments/*/results/`. Needs Python 3.10+, matplotlib and numpy.
