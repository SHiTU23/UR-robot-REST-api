#!/usr/bin/env bash
# Close this PC to the Wi-Fi: nothing comes in over Wi-Fi except SSH, and nothing is forwarded to the robot network.
#   sudo tools/lockdown_firewall.sh [wifi interface] [robot interface] [robot ip]
# Undo with: sudo ufw disable
set -euo pipefail
WIFI="${1:-wlx00e04cc6440f}"; ROBOT_IF="${2:-enp0s31f6}"; ROBOT_IP="${3:-192.168.56.101}"
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
ufw default deny incoming
ufw default allow outgoing
ufw allow in on "$WIFI" to any port 22 proto tcp comment 'SSH, also for the API tunnel'
ufw allow in on "$ROBOT_IF" from "$ROBOT_IP" comment 'UR controller: driver reverse connections 50001-50004'
ufw allow in on docker0 comment 'URSim container (./run.sh ursim)'
ufw route deny in on "$WIFI" out on "$ROBOT_IF" comment 'no Wi-Fi to robot routing'
ufw --force enable
ufw status verbose
