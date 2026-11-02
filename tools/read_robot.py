#!/usr/bin/env python3
"""Print the controller's active TCP, payload and joints. READ-ONLY: RTDE outputs (30004) and the read-only primary
port (30011); it sends no command and can run next to the driver.   tools/read_robot.py [robot ip]"""
import math
import socket
import struct
import sys

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.56.101"


def rtde():
    s = socket.create_connection((IP, 30004), 3)
    s.settimeout(3)

    def send(cmd, payload=b""):
        s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)

    def recv():
        n, c = struct.unpack(">HB", s.recv(3, socket.MSG_WAITALL))
        return c, s.recv(n - 3, socket.MSG_WAITALL) if n > 3 else b""
    send(86, struct.pack(">H", 2))
    recv()
    send(79, struct.pack(">d", 10.0) + b"tcp_offset,payload,payload_cog,actual_q")
    recv()
    send(83)
    while True:
        c, d = recv()
        if c == 85:
            send(80)
            s.close()
            return struct.unpack(">6d d 3d 6d", d[1:])


v = rtde()
print("TCP offset x y z (m):", [round(x, 5) for x in v[:3]])
print("TCP offset rotation vector (rad):", [round(x, 5) for x in v[3:6]])
print("payload (kg):", round(v[6], 3), " centre of gravity (m):", [round(x, 4) for x in v[7:10]])
print("joints (rad):", [round(x, 5) for x in v[10:16]])
print("joints (deg):", [round(math.degrees(x), 2) for x in v[10:16]])
