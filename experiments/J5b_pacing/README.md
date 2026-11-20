# J5b: pacing at 0.75c

Package `SCI_J5B_PACE75_A1_20261118`. It contains the J5 package unchanged (`vendor/`) and changes one constant:
the pace of mode NP is 0.75c instead of 0.9c.

- protocol `J5B_PROTOCOL_FREEZE.md` (hashes in `J5B_FREEZE_HASHES.json`)
- amendment `J5B_AMENDMENT_A1.md` (hashes in `J5B_A1_FREEZE_HASHES.json`): the first attempt stopped in the pilots
  because one read of the Wi-Fi association failed. A1 retries such reads; a real disconnect still makes the replay
  invalid. Written before any counted replay.
- run with `JALANKAN_J5B_A1_EDGE.sh` and `JALANKAN_J5B_A1_ROBOT.sh`

40 counted replays (Q1, limit in the scheduler and paced at 0.75c, 4 schedulers x 5 traces) and 20 exploratory ones
(Q2 at 0.75c). The predictions fixed before the run are in `predictions/J5B_PREDICTIONS.json`.

`results/`: `J5B_REPLAYS.csv`, `J5B_CELLS.csv`, `J5B_TESTS.csv`, `J5B_VERDICTS.json`, `J5B_PLATFORM.json`,
`J5B_PILOTS.json` (with the background traffic measurement)
