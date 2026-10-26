"""URSim after boot: confirm the safety configuration and switch to Remote Control by clicking over VNC (PolyScope
always boots in Local mode and neither choice is kept in a file), then power on + brake release through the
Dashboard. Coordinates are URSim 5.17's 1280x800 screen (measured from screenshots, ~/sota/ui/ursim.py,
2026-09-16). PolyScope lags ~5 s behind input, and each new VNC connection closes open popups, so a menu
open + item click happen in ONE vncdo session."""
import os
import socket
import subprocess
import sys
import tempfile
import time

IP = sys.argv[1]
SAFETY_DIALOG_PIXELS = {(636, 99): (65, 137, 205), (560, 681): (82, 161, 213)}
SAFETY_CONFIRM_XY, MODE_MENU_XY, REMOTE_ITEM_XY = (639, 681), (1117, 58), (1054, 88)


def dash(cmd):
    s = socket.create_connection((IP, 29999), timeout=5)
    f = s.makefile("rwb")
    f.readline()
    f.write((cmd + "\n").encode())
    f.flush()
    ans = f.readline().decode().strip()
    s.close()
    return ans


def wait(pred, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if pred():
                return
        except OSError:
            pass
        time.sleep(1)
    raise SystemExit(f"timed out waiting for {what}")


def vnc(*actions):
    r = subprocess.run([sys.executable, "-m", "vncdotool.command", "-s", f"{IP}::5900", *map(str, actions)],
                       capture_output=True, text=True, timeout=120)
    if r.returncode:
        raise SystemExit(f"vncdo failed: {r.stderr[-300:]}")


def click(x, y, wait_s):
    return ["move", x, y, "pause", 0.5, "mousedown", 1, "pause", 0.3, "mouseup", 1, "pause", wait_s]


def safety_dialog():
    from PIL import Image
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "s.png")
        vnc("capture", p)
        im = Image.open(p).convert("RGB")
        return all(im.getpixel(xy) == rgb for xy, rgb in SAFETY_DIALOG_PIXELS.items())


wait(lambda: dash("robotmode") is not None, 240, "the dashboard (port 29999)")
for _ in range(3):
    if dash("is in remote control").lower() == "true":
        break
    if safety_dialog():
        print("confirming the safety configuration (VNC)", flush=True)
        vnc(*click(*SAFETY_CONFIRM_XY, 12))
    print("switching to Remote Control (VNC)", flush=True)
    vnc(*click(*MODE_MENU_XY, 8), *click(*REMOTE_ITEM_XY, 10))
if dash("is in remote control").lower() != "true":
    raise SystemExit("still in Local mode: switch it by hand over VNC")
print("remote control: on", flush=True)
if not dash("robotmode").endswith("RUNNING"):
    dash("close popup")
    dash("power on")
    wait(lambda: dash("robotmode").endswith("IDLE"), 90, "robot mode IDLE")
    dash("brake release")
    wait(lambda: dash("robotmode").endswith("RUNNING"), 90, "robot mode RUNNING")
print("robot mode:", dash("robotmode"), "| safety:", dash("safetystatus"), flush=True)
