# comms/command_listener.py — Background daemon thread for GCS → CubeSat commands
#
# Listens on COMMAND_PORT for JSON commands from the ground station.
# Commands are placed in a thread-safe queue; the main state machine drains
# them at safe points via get_pending().
#
# Each connection from GCS can send multiple commands (one JSON object per line).
# The listener accepts new connections automatically after one closes, so the
# GCS can reconnect between passes without restarting the CubeSat software.

import json
import queue
import socket
import threading

from config import COMMAND_PORT
from protocol import ACK, NACK


# Valid command names — used for validation before queuing.
_KNOWN_COMMANDS = {
    "retransmit",
    "priority_cell",
    "set_cell",
    "adjust_exposure",
    "enter_safe_mode",
    "resume_normal",
    "status_request",
    "retry_downlink",
    "start_pass",
    "end_pass",
    "cell",
}


class CommandListener:
    def __init__(self):
        self._queue = queue.Queue()
        self._thread = None

    def start(self):
        """Start the background daemon thread. Safe to call once at boot."""
        self._thread = threading.Thread(
            target=self._accept_loop, daemon=True, name="CommandListener"
        )
        self._thread.start()

    def get_pending(self):
        """Drain and return all commands received since the last call.

        Returns:
            list of dicts — each dict is one parsed GCS command.
            Returns empty list if no commands have arrived.
        """
        commands = []
        while True:
            try:
                commands.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return commands

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _accept_loop(self):
        """Main loop: bind server socket, accept connections one at a time."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", COMMAND_PORT))
        server.listen(1)

        while True:
            try:
                conn, addr = server.accept()
                self._handle_connection(conn)
            except OSError:
                # Server socket closed — daemon thread will exit naturally.
                break

    def _handle_connection(self, conn):
        """Read newline-delimited JSON commands from one GCS connection.
        Runs until the connection is closed by GCS or a socket error occurs.
        """
        buf = b""
        try:
            conn.settimeout(5.0)
            while True:
                data = conn.recv(4096)
                if not data:
                    break                   # GCS closed the connection cleanly
                buf += data
                # Process all complete lines in the buffer.
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    ok = self._parse_and_queue(line)
                    try:
                        conn.sendall(ACK if ok else NACK)
                    except OSError:
                        pass
        except OSError:
            pass                            # Socket error — connection dropped
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _parse_and_queue(self, line_bytes):
        """Parse one JSON line and put it in the command queue if valid."""
        try:
            cmd = json.loads(line_bytes.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return False  # Malformed command

        if not isinstance(cmd, dict) or "cmd" not in cmd:
            return False  # Not a command dict

        if cmd["cmd"] not in _KNOWN_COMMANDS:
            return False  # Unknown command

        self._queue.put(cmd)
        return True
