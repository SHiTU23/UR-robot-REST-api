"""Minimal RTDE (protocol v2, TCP 30004) client for exactly the registers the RG6 programs read.

Why not ur_rtde's RTDEIOInterface (as the sota UI used without ROS): its input recipe also claims
speed_slider_mask and standard_digital_output_mask, which ur_robot_driver already holds, so the controller
refuses the WHOLE recipe ("One of the RTDE input registers are already in use!"). This client claims only
the given input_double_register_N fields (checked free next to the driver on URSim 5.17, 2026-10-26).

Each write is one short session: protocol version -> output recipe (timestamp, needed to start) -> input
recipe -> start -> one data package -> read back -> pause -> close. The values stay in the registers after
the session ends (read back over a second connection in the same test).
"""
import socket
import struct
import time

PORT = 30004
V, O, I, S, P, U = (ord(c) for c in "VOISPU")


class RTDEError(RuntimeError):
    pass


class _Session:
    def __init__(self, host, timeout_s):
        self.s = socket.create_connection((host, PORT), timeout=timeout_s)
        self.s.settimeout(timeout_s)

    def send(self, t, payload=b""):
        self.s.sendall(struct.pack(">HB", 3 + len(payload), t) + payload)

    def recv(self, want=None, timeout_s=2.0):
        """Next package (of type ``want``); skips data packages and messages in between."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            hdr = self._exact(3)
            n, t = struct.unpack(">HB", hdr)
            body = self._exact(n - 3)
            if want is None or t == want:
                return t, body
        raise RTDEError(f"no RTDE package {chr(want) if want else ''} within {timeout_s} s")

    def _exact(self, n):
        b = b""
        while len(b) < n:
            chunk = self.s.recv(n - len(b))
            if not chunk:
                raise RTDEError("RTDE connection closed")
            b += chunk
        return b

    def close(self):
        try:
            self.send(P)
        except OSError:
            pass
        self.s.close()


def _setup(sess, kind, fields, freq=None):
    payload = (struct.pack(">d", freq) if freq is not None else b"") + ",".join(fields).encode()
    sess.send(kind, payload)
    _, b = sess.recv(kind)
    rid, types = b[0], b[1:].decode().split(",")
    bad = [f"{f}={t}" for f, t in zip(fields, types) if t in ("IN_USE", "NOT_FOUND")]
    if rid == 0 or bad:
        raise RTDEError(f"RTDE refused the {'input' if kind == I else 'output'} recipe: {', '.join(bad) or types}")
    return rid, types


class RegisterWriter:
    def __init__(self, host, timeout_s=3.0):
        self.host, self.timeout_s = host, timeout_s

    def write_doubles(self, values):
        """{register: value} into input_double_register_N; returns the values read back."""
        regs = sorted(values)
        sess = _Session(self.host, self.timeout_s)
        try:
            sess.send(V, struct.pack(">H", 2))
            _, b = sess.recv(V)
            if not b[0]:
                raise RTDEError("the controller does not accept RTDE protocol version 2")
            fields = [f"input_double_register_{r}" for r in regs]
            _setup(sess, O, ["timestamp"] + fields, freq=10.0)        # read back the same registers as outputs
            rid, _ = _setup(sess, I, fields)
            sess.send(S)
            _, b = sess.recv(S)
            if not b[0]:
                raise RTDEError("RTDE start was refused")
            sess.send(U, struct.pack(">B", rid) + b"".join(struct.pack(">d", float(values[r])) for r in regs))
            t0 = time.time()
            while time.time() - t0 < 1.0:                   # until an output package shows the new values
                _, b = sess.recv(U)
                got = struct.unpack(">" + "d" * (1 + len(regs)), b[1:1 + 8 * (1 + len(regs))])[1:]
                if all(abs(g - float(values[r])) < 1e-9 for g, r in zip(got, regs)):
                    return dict(zip(regs, got))
            raise RTDEError(f"registers {regs} did not take the new values within 1 s (read back {got})")
        except OSError as e:
            raise RTDEError(f"RTDE {self.host}:{PORT}: {e}") from e
        finally:
            sess.close()

    def read_doubles(self, regs, output=True):
        """Current values of output_double_register_N (or input_... with output=False)."""
        kind = "output" if output else "input"
        sess = _Session(self.host, self.timeout_s)
        try:
            sess.send(V, struct.pack(">H", 2))
            sess.recv(V)
            _setup(sess, O, ["timestamp"] + [f"{kind}_double_register_{r}" for r in regs], freq=10.0)
            sess.send(S)
            sess.recv(S)
            _, b = sess.recv(U)
            return dict(zip(regs, struct.unpack(">" + "d" * (1 + len(regs)), b[1:1 + 8 * (1 + len(regs))])[1:]))
        except OSError as e:
            raise RTDEError(f"RTDE {self.host}:{PORT}: {e}") from e
        finally:
            sess.close()
