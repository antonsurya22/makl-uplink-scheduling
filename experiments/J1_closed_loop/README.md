# J1: MAKL vs. baselines (closed loop)

Package `SCI_J1_SCHEDULERS_V2_20261113`, protocol `J1_PROTOCOL_FREEZE.md`. 240 runs: 8 schedulers, 6 cells,
5 start shifts (Gazebo + Nav2).

`superseded/SCI_J1_SCHEDULERS_20261112` is the first version. Its gate check stopped the run before any counted run;
amendment A1 (section 12 of the protocol of the second version) fixed this. We keep it for completeness; it was not
used for any result.

The runner also needs the E3-B, E5 and E6 packages (on Zenodo), see the package README.

`results/`: `J1_RUNS.csv`, `J1_CELLS.csv`, `J1_PAIRED.csv` (all paired tests), `J1_VERDICTS.json`,
`J1_CHANGES_VS_B0_B7.csv` (exploratory)
