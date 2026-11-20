# J2: jitter, loss and varying capacity

Package `SCI_J2_LINK_REALISM_20261114`, protocol `J2_PROTOCOL_FREEZE.md`. 252 runs: 6 schedulers, 2 cells,
7 link conditions, 3 start shifts.

The impairments are in `j2_impair.py` (per-packet jitter that keeps the order, Bernoulli and Gilbert-Elliott loss,
alternating capacity). All random draws depend only on the run seed and the packet.

`results/`: `J2_RUNS.csv`, `J2_CELLS.csv`, `J2_TESTS.csv`, `J2_RANKS.csv` (exploratory), `J2_VERDICTS.json`
