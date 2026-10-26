#!/usr/bin/env bash
# ./run.sh up [ip]   real arm (UR_ROBOT_IP or cell.yaml robot.ip)  | ./run.sh mock    no robot at all
# ./run.sh ursim     URSim container (tests/ursim) + the stack      | ./run.sh down | logs | test [url]
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mkdir -p state data
case "${1:-}" in
  up)    UR_ROBOT_IP="${2:-${UR_ROBOT_IP:-}}" docker compose up -d --build ;;
  mock)  UR_MOCK=true UR_MOCK_HARDWARE=1 UR_CAMERA=none docker compose up -d --build ;;
  ursim) docker build -q -f docker/Dockerfile -t ur-rest-api:latest . >/dev/null
         docker tag ur-rest-api:latest ur-rest-api:dev
         tests/ursim/start_ursim.sh
         IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' urapi-ursim)
         UR_ROBOT_IP="$IP" UR_CELL_CONFIG=/app/tests/ursim/cell_ursim.yaml UR_CAMERA=none docker compose up -d ;;
  down)  docker compose down ;;
  logs)  docker compose logs -f --tail 50 ;;
  test)  docker run --rm --network host -v "$PWD:/app" ur-rest-api:latest python3 /app/tests/api_smoke.py \
           --url "${2:-http://127.0.0.1:5000}" ${3:+--ursim-ip "$3"} ;;
  *)     sed -n '2,3p' "$0"; exit 1 ;;
esac
