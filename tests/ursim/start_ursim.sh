#!/usr/bin/env bash
# URSim (UR10e, PolyScope 5.17) as a stand-in robot for the API tests: container urapi-ursim on the default
# bridge network, Remote Control pre-enabled, stand-in RG6 programs installed. Prints the simulator IP.
#   tests/ursim/start_ursim.sh            # create / start, then run prepare.py (Remote mode, power on)
#   tests/ursim/start_ursim.sh --rm       # remove the container
# The stand-in gripper programs read the width/force registers like the real ones, echo the width (mm) into
# output_double_register 12 (the "measured width" of tests/ursim/cell_ursim.yaml) and wait 1 s; the close
# program acts as if it closed on a 13 mm tube (pipeFit's tube size), so a grasp test has something to hold.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=urapi-ursim
IMAGE=universalrobots/ursim_e-series:5.17
if [[ "${1:-}" == "--rm" ]]; then docker rm -f "$NAME"; exit 0; fi
if ! docker inspect "$NAME" >/dev/null 2>&1; then
  docker create --name "$NAME" -e ROBOT_MODEL=UR10 "$IMAGE" >/dev/null
  # PolyScope reads these on its first boot: Remote Control enabled, no welcome screen
  tmp=$(mktemp -d); mkdir -p "$tmp/.polyscope"
  echo "RemoteControlEnabled=true" > "$tmp/.polyscope/remotecontrol.properties"
  echo "showWelcomeScreen=false" > "$tmp/.polyscope/welcome.properties"
  docker cp "$tmp/.polyscope" "$NAME:/ursim/"
  rm -rf "$tmp"
fi
docker start "$NAME" >/dev/null
IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$NAME")
# stand-in programs (a .urp is gzipped XML; its Script node holds the URScript)
tmp=$(mktemp -d)
mk() {  # name, script (XML-escaped in the .urp: a raw "<" makes PolyScope refuse to load it)
  printf '%s' "$2" > "$tmp/$1.script"
  local esc
  esc=$(printf '%s' "$2" | sed -e 's/&/\&amp;/g' -e 's/</\&lt;/g' -e 's/>/\&gt;/g')
  cat > "$tmp/$1.xml" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<URProgram createdIn="5.17.0" lastSavedIn="5.17.0" name="$1" directory="/programs" installation="default">
  <children><MainProgram runOnlyOnce="true"><children>
    <Script type="File"><cachedContents>$esc</cachedContents><file resolves-to="file">/programs/$1.script</file></Script>
  </children></MainProgram></children>
</URProgram>
XML
  gzip -c "$tmp/$1.xml" > "$tmp/$1.urp"; rm "$tmp/$1.xml"
}
GRIP='write_output_float_register(12, read_input_float_register(18))
write_output_float_register(13, read_input_float_register(19))
sleep(1.0)
'
mk gripper_open "$GRIP"
# the close stand-in behaves as if a 13 mm tube is between the fingers: it stops at 13 mm or the command, whichever is wider
# (URScript has no max(): an if)
mk gripper_close 'w = read_input_float_register(18)
if w < 13.0:
  w = 13.0
end
write_output_float_register(12, w)
write_output_float_register(13, read_input_float_register(19))
sleep(1.0)
'
docker exec "$NAME" mkdir -p /ursim/programs.UR10
docker cp "$tmp/." "$NAME:/ursim/programs.UR10/" >/dev/null
rm -rf "$tmp"
echo "URSim $NAME at $IP (VNC $IP:5900, web VNC http://$IP:6080/vnc.html)"
docker run --rm --network bridge -e PYTHONPATH=/vnclib \
  -v "${VNCLIB:-/tmp/vnclib}:/vnclib:ro" -v "$HERE:/t:ro" ur-rest-api:dev python3 /t/prepare.py "$IP"
