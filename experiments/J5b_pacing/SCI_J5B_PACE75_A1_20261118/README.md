# J5b package — pace 0.75 c over the real ROS 2 / Wi-Fi stack (open loop, the robot does not move)

Protocol: `J5B_PROTOCOL_FREEZE.md`. Freeze: `J5B_FREEZE_HASHES.json`. The J5 package is inside, unchanged
(`vendor/SCI_J5_NETSTACK_20261118`). Setup as J5: the sudo rule, chrony and power save from J5 stay as they are.
Do not run `apt upgrade` (the Pi checks that the versions equal those at the J5 lock).

## Copy (put the ZIP in a new, empty folder first)
VM:
```
mkdir -p ~/j5b_zip && cp ~/Downloads/SCI_J5B_PACE75_20261118.zip ~/j5b_zip/ && cd ~/j5b_zip
sha256sum SCI_J5B_PACE75_20261118.zip
python3 -m zipfile -e SCI_J5B_PACE75_20261118.zip ~/ros2_ws/
scp SCI_J5B_PACE75_20261118.zip ubuntu@192.168.200.159:~/
```
Pi:
```
python3 -m zipfile -e ~/SCI_J5B_PACE75_20261118.zip ~/ros2_ws/
```

## Run (VM first, then the Pi)
VM: `bash ~/ros2_ws/SCI_J5B_PACE75_20261118/JALANKAN_J5B_EDGE.sh` and wait for `8/8 edge receiver`.
Pi: `tmux new -s j5b` then `bash ~/ros2_ws/SCI_J5B_PACE75_20261118/JALANKAN_J5B_ROBOT.sh`.
As soon as step `7/7` appears, detach tmux (Ctrl+b, then d), so that no screen output travels over Wi-Fi during the
pilots and the background measurement. Look again with `tmux attach -t j5b`.

About 50 min. `J5_NOT_STARTED`: a setup problem before anything was counted; fix it and run again.
`J5_HALT` after LOCK: send the console logs, delete nothing. A finished run is reported as finished even if the
process aborts on exit (rc 134, known from J5); do not run it again.

## Results
VM: `~/Downloads/J5B_RESULTS_<time>.zip`. Pi: `~/J5B_ROBOT_SIDE_<time>.zip`.
