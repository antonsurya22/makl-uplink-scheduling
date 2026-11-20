# J5b environment = the frozen J5 environment (vendor/SCI_J5_NETSTACK_20261118/j5_env.sh) with the J5b results folder.
# Usage: source j5b_env.sh <edge|robot>
_J5B_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$_J5B_PKG/vendor/SCI_J5_NETSTACK_20261118/j5_env.sh" "$1"
J5_RES="$HOME/ros2_ws/results/SCI_J5B_PACE75_20261118"
