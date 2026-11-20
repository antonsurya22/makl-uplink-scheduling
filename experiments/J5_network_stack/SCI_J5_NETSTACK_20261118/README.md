# J5 package — real ROS 2 / Wi-Fi stack (open loop, the robot does not move)

Protocol: `J5_PROTOCOL_FREEZE.md`. Freeze: `J5_FREEZE_HASHES.json` (checked by both scripts before anything runs).

## Before you start
- No `apt upgrade` on the Pi or the VM until J5 ends.
- Pi: on wall power, still, TurtleBot bring-up NOT running. VM: bridged adapter, IP 192.168.200.235.
- In VMware, "Synchronize guest time with host" should be off (default), so that only chrony sets the VM clock.
- Both scripts ask for the sudo password once.
  - VM: installs `/etc/chrony/conf.d/j5.conf` (time server for the Pi) and, if `ufw` is active, allows traffic from the Pi.
  - Pi: installs `/etc/sudoers.d/j5` (no password for `/usr/sbin/tc` and `/usr/sbin/iw` only) and
    `/etc/chrony/sources.d/j5.sources` (follow the VM clock).
- To undo after J5: `sudo rm /etc/sudoers.d/j5 /etc/chrony/sources.d/j5.sources` (Pi) and
  `sudo rm /etc/chrony/conf.d/j5.conf && sudo systemctl restart chrony` (VM).

## Copy (put the ZIP in a new, empty folder first)
VM:
```
mkdir -p ~/j5_zip && cd ~/j5_zip   # put SCI_J5_NETSTACK_20261118.zip here
python3 -m zipfile -e SCI_J5_NETSTACK_20261118.zip ~/ros2_ws/
scp SCI_J5_NETSTACK_20261118.zip ubuntu@192.168.200.159:~/      # user name of the Pi
```
Pi:
```
python3 -m zipfile -e ~/SCI_J5_NETSTACK_20261118.zip ~/ros2_ws/
```

## Run (VM first, then the Pi)
VM terminal: `bash ~/ros2_ws/SCI_J5_NETSTACK_20261118/JALANKAN_J5_EDGE.sh`
Wait for `8/8 edge receiver`, then on the Pi (SSH is fine; use `tmux` or `screen` so a dropped SSH does not stop it):
`bash ~/ros2_ws/SCI_J5_NETSTACK_20261118/JALANKAN_J5_ROBOT.sh`

About 1.5–2 h. Both scripts can be stopped with Ctrl+C and started again; the Pi continues where it stopped.
`J5_NOT_STARTED` means a setup problem before anything was counted: fix it and run again.
`J5_HALT` after LOCK needs review: send the console logs, delete nothing.

## Results
VM: `~/Downloads/J5_RESULTS_<time>.zip` (all raw logs and `J5_MAIN/final/`). Pi: `~/J5_ROBOT_SIDE_<time>.zip`.
