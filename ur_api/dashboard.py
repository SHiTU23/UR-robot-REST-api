"""UR Dashboard Server client (TCP 29999, one text line per command and per answer).

Used for what the ROS driver does not cover here: playing the RG6 pendant programs, protective-stop
recovery (unlock + close popup), and reading the remote-control / program state. A broken connection is
re-opened once per command; every command holds a lock, so answers never interleave."""
import socket
import threading
import time

PORT = 29999


class DashboardError(RuntimeError):
    pass


class Dashboard:
    def __init__(self, host, port=PORT, timeout_s=5.0):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self._sock = None
        self._buf = b""
        self._lock = threading.RLock()

    def _connect(self):
        s = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        s.settimeout(self.timeout_s)
        self._sock, self._buf = s, b""
        self._readline()                      # "Connected: Universal Robots Dashboard Server"

    def _readline(self):
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("dashboard closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8", "replace").strip()

    def close(self):
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                finally:
                    self._sock = None

    def cmd(self, text):
        """Send one command, return its answer line."""
        with self._lock:
            for attempt in (0, 1):
                try:
                    if self._sock is None:
                        self._connect()
                    self._sock.sendall((text + "\n").encode())
                    return self._readline()
                except (OSError, ConnectionError) as e:
                    self.close()
                    if attempt:
                        raise DashboardError(f"dashboard {self.host}:{self.port}: {e}") from e
        raise DashboardError("unreachable")

    # ---- queries ----

    def running(self):
        return self.cmd("running").lower().endswith("true")

    def robot_mode(self):
        return self.cmd("robotmode").split(":", 1)[-1].strip()          # "Robotmode: RUNNING"

    def safety_status(self):
        return self.cmd("safetystatus").split(":", 1)[-1].strip()       # "Safetystatus: PROTECTIVE_STOP"

    def program_state(self):
        return self.cmd("programState")                                  # "STOPPED gripper_open.urp"

    def remote_control(self):
        try:
            return self.cmd("is in remote control").lower().strip() == "true"
        except DashboardError:
            raise
        except Exception:      # noqa: BLE001  (CB3 has no remote-control mode)
            return True

    # ---- actions ----

    def load(self, program):
        ans = self.cmd(f"load {program}")
        if not ans.lower().startswith("loading program"):
            raise DashboardError(f"load {program}: {ans}")
        return ans

    def play(self):
        ans = self.cmd("play")
        if not ans.lower().startswith("starting program"):
            raise DashboardError(f"play: {ans}")
        return ans

    def stop(self):
        return self.cmd("stop")

    def unlock_protective_stop(self):
        return self.cmd("unlock protective stop")

    def close_safety_popup(self):
        return self.cmd("close safety popup")

    def close_popup(self):
        return self.cmd("close popup")

    def wait_stopped(self, timeout_s=5.0):
        """Stop the running program until the dashboard reports nothing running."""
        t0 = time.time()
        while True:
            try:
                if not self.running():
                    return True
            except DashboardError:
                pass
            if time.time() - t0 > timeout_s:
                return False
            try:
                self.stop()
            except DashboardError:
                pass
            time.sleep(0.2)
