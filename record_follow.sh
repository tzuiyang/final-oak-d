#!/usr/bin/env bash
# Record a person-following session for offline debug.
#
# Usage:
#   ./record_follow.sh                # writes bags/follow_<timestamp>
#   ./record_follow.sh my_run         # writes bags/my_run
#
# Topics chosen to answer:
#   - what did the camera see?           /oakd/detections, /oakd/frame_jpeg
#   - what target did we lock onto?      /oakd/target, /oakd/select_target
#   - did mission_controller block us?   /oakd/path_clear, /oakd/error
#   - what command did we publish?       /person_following_cmd_vel
#   - did the mux forward it?            /cmd_vel, /cmd_vel_mux/active_source
#   - was teleop preempting us?          /teleop_cmd_vel, /joy, /oakd/mode
#   - did the robot actually move?       /imu/data, /odom (if present)

set -euo pipefail

cd "$(dirname "$0")"
mkdir -p bags

name="${1:-follow_$(date +%Y%m%d_%H%M%S)}"
out="bags/${name}"

if [[ -e "$out" ]]; then
  echo "refusing to overwrite existing bag: $out" >&2
  exit 1
fi

topics=(
  /oakd/detections
  /oakd/target
  /oakd/select_target
  /oakd/path_clear
  /oakd/error
  /oakd/mode
  /oakd/frame_jpeg
  /person_following_cmd_vel
  /cmd_vel
  /teleop_cmd_vel
  /joy
  /imu_sensor_broadcaster/imu
)

echo "recording to $out"
echo "topics: ${topics[*]}"
echo "Ctrl+C to stop."

# --include-unpublished-topics so /odom etc. still get recorded if they
# come up after the recorder starts. The QoS overrides are essential —
# without them the recorder subscribes RELIABLE and silently drops every
# message from our BEST_EFFORT publishers.
exec ros2 bag record \
  -o "$out" \
  -s mcap \
  --include-unpublished-topics \
  --qos-profile-overrides-path recorder_qos.yaml \
  "${topics[@]}"
