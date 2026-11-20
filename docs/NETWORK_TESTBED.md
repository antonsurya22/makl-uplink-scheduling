# Wi-Fi test bed (J5, J5b)

The J5/J5b scripts do the setup and the checks themselves. This is a short summary.

## Machines

- Sender: the Raspberry Pi of a TurtleBot3. The robot does not move (no bring-up, LiDAR, camera or motors).
- Edge: Ubuntu VM (bridged network) on a laptop connected by cable to the access point.
- One 2.4 GHz Wi-Fi hop. Power saving is off on the sender, and it has to stay on the same access point
  (BSSID checked every second).

## DDS

- `ROS_DOMAIN_ID=77`, `rmw_cyclonedds_cpp`
- CycloneDDS uses only the Wi-Fi interface on the sender and the bridged interface on the edge; no multicast,
  unicast discovery with the other machine as the only peer (`j5_env.sh`)
- data topics: best effort, keep-last 1 (writer) and 50 (reader); control topics: reliable

## Clock

chrony. The edge is the server (`allow` for the lab subnet, `local stratum 10`) and the sender uses
`server EDGE iburst minpoll 0 maxpoll 2 prefer trust`. A replay only counts if the offset is within 2 ms before and
after it. We measured at most 0.28 ms.

## Capacity limit in the kernel (modes N and NP)

```
tc qdisc replace dev wlan0 root tbf rate 3000000bit burst 1600 limit 768000
tc qdisc change  dev wlan0 root tbf rate <c>bit     burst 1600 limit 768000   # at the trigger; back to 3 Mbit/s after 25 s
tc qdisc del     dev wlan0 root
```

The sender may run only `tc` and `iw` with sudo (sudoers rule for these two programs).

## Running

Start the edge first, then the sender:

```bash
bash experiments/J5_network_stack/SCI_J5_NETSTACK_20261118/JALANKAN_J5_EDGE.sh    # edge
bash experiments/J5_network_stack/SCI_J5_NETSTACK_20261118/JALANKAN_J5_ROBOT.sh   # sender
```

The scripts expect the package in `~/ros2_ws/` and write to `~/ros2_ws/results/<package>/`.

## Cleanup

```bash
sudo rm /etc/sudoers.d/j5 /etc/chrony/sources.d/j5.sources && sudo chronyc reload sources   # sender
sudo rm /etc/chrony/conf.d/j5.conf && sudo systemctl restart chrony                          # edge
```
