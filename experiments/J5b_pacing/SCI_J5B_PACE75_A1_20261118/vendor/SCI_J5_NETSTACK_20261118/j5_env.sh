# J5 environment, sourced by both JALANKAN scripts. Usage: source j5_env.sh <edge|robot>
# Domain 77, CycloneDDS, unicast discovery between the two fixed hosts (no multicast over Wi-Fi).
J5_ROLE="$1"
J5_SENDER_IP=192.168.200.159
J5_EDGE_IP=192.168.200.235
if [ "$J5_ROLE" = edge ]; then J5_IF=ens33; J5_SELF=$J5_EDGE_IP; J5_PEER=$J5_SENDER_IP
else J5_IF=wlan0; J5_SELF=$J5_SENDER_IP; J5_PEER=$J5_EDGE_IP; fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE FASTRTPS_DEFAULT_PROFILES_FILE || true
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface name=\"$J5_IF\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex><Peers><Peer address=\"$J5_PEER\"/></Peers></Discovery></Domain></CycloneDDS>"
export PYTHONDONTWRITEBYTECODE=1
J5_RES="$HOME/ros2_ws/results/SCI_J5_NETSTACK_20261118"
