#!/usr/bin/env python3
"""Watch the tool connector through RTDE while you open and close the gripper. READ-ONLY; it sends no command.
    tools/watch_tool_io.py [seconds] [robot ip]
Prints a line whenever an input changes, to show whether any of them follows the finger width."""
import socket
import struct
import sys
import time

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
IP = sys.argv[2] if len(sys.argv) > 2 else "192.168.56.101"
NAMES = ["tool_analog_input0", "tool_analog_input1", "tool_analog_input_types", "tool_output_current",
         "tool_output_voltage", "actual_digital_input_bits"]
NOISE_V = 0.03         # the inputs jitter by about 0.01 V at rest
FORMATS = {"INT32": "i", "UINT32": "I", "DOUBLE": "d", "UINT64": "Q", "UINT8": "B"}

s = socket.create_connection((IP, 30004), 3)
s.settimeout(3)


def send(cmd, payload=b""):
    s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)


def recv():
    n, c = struct.unpack(">HB", s.recv(3, socket.MSG_WAITALL))
    return c, s.recv(n - 3, socket.MSG_WAITALL) if n > 3 else b""


send(86, struct.pack(">H", 2))
recv()
send(79, struct.pack(">d", 20.0) + ",".join(NAMES).encode())
_, d = recv()
fmt = ">" + "".join(FORMATS[t] for t in d[1:].decode().split(","))
send(83)
last, end = None, time.time() + SECONDS
print("time   analog0   analog1 types  current_A  volt  tool_DI0 tool_DI1")
while time.time() < end:
    c, d = recv()
    if c != 85:
        continue
    a0, a1, types, cur, volt, bits = struct.unpack(fmt, d[1:])
    now = (a0, a1, types, (bits >> 16) & 1, (bits >> 17) & 1)
    if last is None or now[2:] != last[2:] or max(abs(now[0] - last[0]), abs(now[1] - last[1])) > NOISE_V:
        print(f"{time.strftime('%H:%M:%S')} {a0:8.4f} {a1:8.4f} {types:5d} {cur:9.3f} {volt:5d} {now[3]:8d} {now[4]:8d}", flush=True)
        last = now
send(80)
s.close()
