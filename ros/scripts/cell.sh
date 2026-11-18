#!/usr/bin/env bash
# The ROS side (compose service "ros"): cell.launch.py with the gripper picked on the web page.
# Order, as ur_api/gripper_select.py: env UR_GRIPPER forces one; else state/gripper_model (POST /gripper/model);
# else gripper.model in the cell config (the URDF/SRDF xacro fall back to it when UR_GRIPPER is empty).
# When the web page's choice changes, the launch is stopped and this script exits: Docker (restart: unless-stopped)
# starts the container again, now with the new gripper. Arguments go to the launch file.
SEL=/app/state/gripper_model
sel() { { tr -d '[:space:]' < "$SEL"; } 2>/dev/null || true; }

forced=${UR_GRIPPER:-}
picked=$(sel)
if [ -z "$forced" ] && [ -n "$picked" ] && [ -f "/app/config/grippers/$picked.yaml" ]; then
  export UR_GRIPPER="$picked"
fi
echo "[cell.sh] gripper: ${UR_GRIPPER:-gripper.model in the cell config}${forced:+ (forced by env UR_GRIPPER)}"

ros2 launch /app/ros/launch/cell.launch.py "$@" &
pid=$!

# SIGTERM, not SIGINT: a script's background job ignores SIGINT; ros2 launch shuts down cleanly on either
stop_launch() {
  kill -TERM "$pid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || return; sleep 0.5; done
  echo "[cell.sh] launch did not stop within 10 s: killing it"
  kill -KILL "$pid" 2>/dev/null
}
trap 'stop_launch; exit 0' TERM INT

while kill -0 "$pid" 2>/dev/null; do
  sleep 1
  if [ -z "$forced" ] && [ "$(sel)" != "$picked" ]; then
    echo "[cell.sh] gripper changed on the web page ('$picked' -> '$(sel)'): restarting the ROS side"
    stop_launch
    exit 0
  fi
done
wait "$pid"
