# MAKL: results and frozen experiment packages

Data for the paper "Margin-aware uplink scheduling for edge-assisted mobile robots: bounding the waiting time of
safety-relevant sensor data" (Y.-C. Tung, I G. N. A. S. Diputra, Y.-N. Lin, V. R. L. Shen).

Code: https://github.com/antonsurya22/makl-uplink-scheduling

Files:
- `*_RESULTS_*.zip`: raw logs of all counted runs of E1-E8 and J1-J5b (message ledgers, scheduler decisions,
  10 Hz state grids, Nav2 and safety-layer logs, edge logs of J5/J5b) and the final tables
- `SCI_*.zip`: the experiment packages as they were run (E1-E8; the J packages are also on GitHub)
- the traces used by the offline replays (J0 and P0-J test T5), inside the P0-J results ZIP
- `SHA256SUMS.txt`: checksums of all files

The archives are uploaded unchanged. Three package ZIPs (SCI_V3I_E1_FIXED_Q1_20261030, SCI_E2_ANALYSIS_PIPELINE_20261030, SCI_E4_OFFLINE_REPLAY_20261101) were zipped again from the copy kept in our earlier code repository because the original ZIP files were no longer available; the files inside are unchanged. E1_E2_DDQN_CHECKPOINTS.zip holds the trained policies of E1/E2, and SCI_P0_STACK_FIX* the stack fixes made before E5. Some experiments had other names while we ran them (paper E6 = E7, E7 = X1,
E8 = X2/X3); see `docs/EXPERIMENTS.md` in the code repository.

License: CC BY 4.0
