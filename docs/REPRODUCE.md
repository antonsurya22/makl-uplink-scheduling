# Reproducing the results

1. Offline checks (any machine with Python 3.10+):

   ```bash
   bash reproduce/run_checks.sh
   ```

2. Figures and supplementary tables: the scripts in `reproduce/figures/` and `reproduce/tables/` only read the files in
   `experiments/*/results/`.

3. Final tables from the raw logs: download the results ZIP of an experiment from Zenodo, unzip it into an empty
   folder and run the analysis script of the package, e.g.

   ```bash
   python3 -I experiments/J5_network_stack/SCI_J5_NETSTACK_20261118/j5_analysis.py RESULTS/J5_MAIN OUT
   python3 -I experiments/J5b_pacing/SCI_J5B_PACE75_A1_20261118/j5b_analysis.py RESULTS/J5B_MAIN_A1 OUT
   ```

   We checked for J1-J5b that this gives the same files as in `results/`.

4. Running an experiment again: see the README of the package. J1-J4 need the simulation setup
   (`docs/ENVIRONMENT.md`) and the E3-B, E5 and E6 packages from Zenodo in the workspace. One closed-loop batch takes
   several hours, J5 about 1.5 h and J5b about 50 min.
