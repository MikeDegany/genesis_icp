#!/usr/bin/env bash
# Copyright (c) 2026 Mike Degany <mike.degany@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

set -eo pipefail

# Required: colcon workspace root (contains install/setup.bash)
: "${WS:?Set WS to your colcon workspace root, e.g. export WS=/path/to/ws}"
# Required: rosbag2 paths (directories or .db3)
: "${BAG_JK3:?Set BAG_JK3 to robot A bag path}"
: "${BAG_JK5:?Set BAG_JK5 to robot B bag path}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
LOGDIR="${LOGDIR:-${TMPDIR:-/tmp}/genesis_icp_logs}"
mkdir -p "$LOGDIR"

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Stop previous runs (narrow patterns avoid killing unrelated bag play)
pkill -f "ros2 bag play.*${BAG_JK3}" 2>/dev/null || true
pkill -f "ros2 bag play.*${BAG_JK5}" 2>/dev/null || true
pkill -f "genesis_icp_node" 2>/dev/null || true
pkill -f "genesis_icp_socket_relay" 2>/dev/null || true
sleep 1

# Start fusion + relays before bags: rosbag2 play republishes /tf_static as volatile, so a
# one-shot static message at bag start is missed if subscribers start late.
cd "$WS"
# shellcheck source=/dev/null
source install/setup.bash
export ROS_DOMAIN_ID=0
set +e
timeout 50 ros2 launch genesis_icp genesis_icp_with_relays.launch.py use_sim_time:=true >"$LOGDIR/gicp_launch.log" 2>&1 &
LAUNCH_PID=$!
set -e
sleep 4

( export ROS_DOMAIN_ID=3; exec ros2 bag play "$BAG_JK3" --clock ) >"$LOGDIR/bag_jk3.log" 2>&1 &
( export ROS_DOMAIN_ID=5; exec ros2 bag play "$BAG_JK5" --clock ) >"$LOGDIR/bag_jk5.log" 2>&1 &
wait $LAUNCH_PID 2>/dev/null || true
EC=$?

pkill -f "ros2 bag play.*${BAG_JK3}" 2>/dev/null || true
pkill -f "ros2 bag play.*${BAG_JK5}" 2>/dev/null || true
pkill -f "genesis_icp_node" 2>/dev/null || true
pkill -f "genesis_icp_socket_relay" 2>/dev/null || true

echo "launch_exit=$EC"
echo "=== bag jk3 head ==="
head -5 "$LOGDIR/bag_jk3.log"
echo "=== grep launch ==="
grep -E "Connected|client connected|Scan match|below threshold|ERROR|FATAL|No tf|Send failed|listening|Could not connect" "$LOGDIR/gicp_launch.log" | head -60 || true
echo "=== tail launch ==="
tail -45 "$LOGDIR/gicp_launch.log"

exit "$EC"
