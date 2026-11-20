# J5b with amendment A1 — read this first

A1 (`J5B_AMENDMENT_A1.md`): BSSID reader with retries; new results folder `J5B_MAIN_A1`.
Use the `JALANKAN_J5B_A1_*` scripts, not the `JALANKAN_J5B_*` ones. Freeze: `J5B_A1_FREEZE_HASHES.json`.
Everything else as in `README.md` (copy, VM first, detach tmux at step 7/7, about 50 min).

VM:  bash ~/ros2_ws/SCI_J5B_PACE75_A1_20261118/JALANKAN_J5B_A1_EDGE.sh
Pi:  tmux new -s j5b1   then   bash ~/ros2_ws/SCI_J5B_PACE75_A1_20261118/JALANKAN_J5B_A1_ROBOT.sh
Results: VM ~/Downloads/J5B_A1_RESULTS_<time>.zip ; Pi ~/J5B_A1_ROBOT_SIDE_<time>.zip
