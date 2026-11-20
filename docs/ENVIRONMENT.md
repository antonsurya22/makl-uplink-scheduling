# Environment

## Simulation (E1-E8, J1-J4)

- Ubuntu 22.04 in a VMware VM, ROS 2 Humble, Gazebo Classic 11 (lock-step, 10 Hz), Nav2 (DWB, AMCL)
- TurtleBot3 Burger model, world `train_00`, route R3
- RMW: `rmw_cyclonedds_cpp`
- Python 3.10 (system Python; PyYAML and NumPy from the Ubuntu/ROS packages)

## Wi-Fi test (J5, J5b)

| | Hardware | Software |
|---|---|---|
| sender | Raspberry Pi 3 Model B+ (TurtleBot3 computer), `wlan0` | Ubuntu 22.04.5 aarch64, ROS 2 Humble, CycloneDDS 0.10.5 (rmw_cyclonedds_cpp 1.3.6), chrony 4.2, iproute2 5.15, iperf3 3.9 |
| edge | Ubuntu 22.04 VM with bridged adapter, on a laptop with a cable to the router | same, x86_64, kernel 6.8 |
| access point | AX3000 dual-band router, 2.4 GHz, channel 9, 20 MHz | |

The package versions of each session are saved with the results (`*_VERSIONS.txt` and the lock in `STATE.json`).

## Build

```bash
mkdir -p ~/makl_ws/src && cp -r ros2_ws/src/* ~/makl_ws/src/
cd ~/makl_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```
