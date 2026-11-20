# J5: real ROS 2 / Wi-Fi stack

Package `SCI_J5_NETSTACK_20261118`, protocol `J5_PROTOCOL_FREEZE.md`.

Open loop: the TurtleBot3's Raspberry Pi (not moving) replays the traffic and trajectory of five uncongested J1 runs
per camera profile in real time through the schedulers, and publishes each message over CycloneDDS and one Wi-Fi hop
to the edge. 120 replays: 4 schedulers x 2 cells x 3 positions of the capacity limit (in the scheduler; in the kernel,
not paced; in the kernel, paced at 0.9c) x 5 traces.

- `j5_common.py`: replay loop using the scheduler code in `vendor/`, message header, run order
- `j5_robot_core.py`, `j5_robot.py`: sender
- `j5_edge_core.py`, `j5_edge.py`: receiver
- `j5_ros.py`: ROS 2 transport; `j5_sysops.py`: calls to `tc`, `iw`, `chronyc`, `ping`, `iperf3`
- `j5_env.sh`: domain and CycloneDDS settings
- `j5_analysis.py`: analysis; `tests/test_j5_offline.py`: offline tests
- `JALANKAN_J5_EDGE.sh`, `JALANKAN_J5_ROBOT.sh`: run scripts

Setup: `docs/NETWORK_TESTBED.md`.

`results/`: `J5_REPLAYS.csv`, `J5_CELLS.csv`, `J5_TESTS.csv`, `J5_VERDICTS.json`, `J5_PLATFORM.json`, `J5_PILOTS.json`.
`J5_PLATFORM.json` pools all modes; the per-mode values are given in the paper.
