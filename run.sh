#!/usr/bin/env bash
# ./run.sh up [ip]   real arm (UR_ROBOT_IP or cell.yaml robot.ip)  | ./run.sh mock    no robot at all
# ./run.sh ursim     URSim container (tests/ursim) + the stack      | ./run.sh down | logs | test [url]
# ./run.sh view [rg6|robotiq_2f85]   mock robot with that gripper + RViz  | ./run.sh rviz   RViz on the running stack
# ./run.sh token     prints the API token (state/api_token)
# ./run.sh save-start  the real arm's current pose becomes the mock robot's start pose
# ./run.sh save-calibration  reads the arm's factory calibration into config/calibration/ (restart afterwards)
# Gripper: gripper.model in config/cell.yaml, or UR_GRIPPER=<model> in front of any command.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mkdir -p state data
# RViz runs as root in its container; allow just that local user on this desktop
cell_ip() { python3 -c 'import yaml; print(yaml.safe_load(open("config/cell.yaml"))["robot"]["ip"])'; }
allow_x() { xhost +SI:localuser:root >/dev/null || echo "could not allow RViz on the display (is DISPLAY set?)"; }
# made here, not in the container, so that it belongs to you and is readable only by you
[ -s state/api_token ] || (umask 077; python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > state/api_token)
case "${1:-}" in
  up)    export UR_ROBOT_IP="${2:-${UR_ROBOT_IP:-$(cell_ip)}}"
         echo "real arm at $UR_ROBOT_IP"
         docker compose up -d --build ;;
  mock)  UR_MOCK=true UR_MOCK_HARDWARE=1 UR_CAMERA=none docker compose up -d --build ;;
  ursim) docker build -q -f docker/Dockerfile -t ur-rest-api:latest . >/dev/null
         docker tag ur-rest-api:latest ur-rest-api:dev
         tests/ursim/start_ursim.sh
         IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' urapi-ursim)
         UR_ROBOT_IP="$IP" UR_CELL_CONFIG=/app/tests/ursim/cell_ursim.yaml UR_CAMERA=none docker compose up -d ;;
  view)  export UR_GRIPPER="${2:-${UR_GRIPPER:-}}"
         docker build -q -f docker/Dockerfile -t ur-rest-api:latest . >/dev/null
         UR_MOCK=true UR_MOCK_HARDWARE=1 UR_CAMERA=none docker compose up -d --no-build --force-recreate ros api
         allow_x; docker compose --profile rviz up -d --force-recreate rviz ;;
  rviz)  allow_x; docker compose --profile rviz up -d --force-recreate rviz ;;
  down)  docker compose --profile rviz down ;;
  logs)  docker compose logs -f --tail 50 ;;
  token) cat state/api_token ;;
  save-start) python3 tools/save_start_pose.py ;;
  save-calibration) python3 tools/save_calibration.py "${2:-$(cell_ip)}" ;;
  test)  docker run --rm --network host -v "$PWD:/app" -e PYTHONDONTWRITEBYTECODE=1 ur-rest-api:latest python3 /app/tests/api_smoke.py \
           --url "${2:-http://127.0.0.1:5000}" ${3:+--ursim-ip "$3"} ;;
  *)     sed -n '2,8p' "$0"; exit 1 ;;
esac
