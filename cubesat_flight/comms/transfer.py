# comms/transfer.py — Throttled TCP client for CubeSat → GCS transfers
#
# All image data is sent in THROTTLE_BYTES_PER_SEC (1200 byte) chunks with a
# real time.sleep(1.0) between chunks, faithfully simulating a 9600-baud UHF link.
# A 28 KB image takes approximately 23 real seconds to transfer.
#
# Tracks consecutive connection failures. After MAX_GCS_CONNECT_FAILURES
# consecutive failures, raises GCSUnreachableError so the main loop can
# suspend downlink rather than burning the entire window on timeouts.

import json
import socket
import time

from config import (
    ACK_TIMEOUT_SEC,
    GROUND_STATION_IP,
    DATA_PORT,
    MAX_GCS_CONNECT_FAILURES,
    THROTTLE_BYTES_PER_SEC,
)
from comms.packet import build_image_header, build_telemetry_header
from protocol import ACK, NACK


class GCSUnreachableError(Exception):
    """Raised when consecutive connection failures reach MAX_GCS_CONNECT_FAILURES."""


class Transfer:
    def __init__(self):
        self._sock = None
        self.consecutive_failures = 0
        self.bytes_sent = 0         # Cumulative bytes sent this downlink window
        self.last_error = None      # Last error string ("NACK", "timeout", etc.)

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self):
        """Open TCP connection to the ground station DATA_PORT.
        Resets consecutive_failures counter on success.
        Raises GCSUnreachableError once MAX_GCS_CONNECT_FAILURES is reached.
        """
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(ACK_TIMEOUT_SEC)
            self._sock.connect((GROUND_STATION_IP, DATA_PORT))
            self.consecutive_failures = 0
        except OSError as exc:
            self._close_socket()
            self.consecutive_failures += 1
            self.last_error = f"connect_failed: {exc}"
            if self.consecutive_failures >= MAX_GCS_CONNECT_FAILURES:
                raise GCSUnreachableError(
                    f"GCS unreachable after {self.consecutive_failures} attempts"
                ) from exc
            raise

    def disconnect(self):
        """Graceful shutdown — flushes socket before closing."""
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._close_socket()

    def force_disconnect(self):
        """Immediate close with no shutdown handshake. Use on link failure."""
        self._close_socket()

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------

    def send_telemetry(self, telem_dict):
        """Send a telemetry packet to the GCS.

        Sends JSON header (newline-terminated) then the telemetry JSON bytes.
        Waits for ACK/NACK. Telemetry is small enough (~500 B) that it fits in
        a single chunk — no throttle sleep needed.

        Returns:
            True on ACK, False on NACK or socket error.
        """
        header, payload = build_telemetry_header(telem_dict)
        try:
            self._send_header(header)
            self._sock.sendall(payload)
            self.bytes_sent += len(payload)
            return self._wait_for_ack()
        except OSError as exc:
            self.last_error = f"socket_error: {exc}"
            self.force_disconnect()
            return False

    def send_file(self, filepath, metadata, watchdog=None):
        """Send one image file to the GCS with throttled chunked transfer.

        Protocol:
          1. Send JSON header (newline-terminated)
          2. Send raw JPEG bytes in THROTTLE_BYTES_PER_SEC chunks,
             sleeping 1.0 second between chunks (real sleep — not simulated)
          3. Wait for ACK/NACK byte from GCS

        Args:
            filepath: Full path to the JPEG on disk.
            metadata: Metadata dict to embed in the header.
            watchdog: Optional Watchdog instance — petted each chunk to prevent
                      timeout during long transfers (~30s for a 35 KB image).

        Returns:
            True on ACK, False on NACK or socket error.
            Sets self.last_error on failure: "NACK", "timeout", or "socket_error: ..."
        """
        header = build_image_header(filepath, metadata)
        try:
            self._send_header(header)
        except OSError as exc:
            self.last_error = f"socket_error: {exc}"
            self.force_disconnect()
            return False

        # Send raw bytes in throttled chunks.
        try:
            with open(filepath, "rb") as f:
                while True:
                    chunk = f.read(THROTTLE_BYTES_PER_SEC)
                    if not chunk:
                        break
                    self._sock.sendall(chunk)
                    self.bytes_sent += len(chunk)
                    if watchdog is not None:
                        watchdog.pet()
                    time.sleep(1.0)  # Real sleep — faithful UHF link simulation
        except OSError as exc:
            self.last_error = f"socket_error: {exc}"
            self.force_disconnect()
            return False

        return self._wait_for_ack()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send_header(self, header_dict):
        """Serialise header dict to JSON and send with newline terminator."""
        line = json.dumps(header_dict, default=str) + "\n"
        self._sock.sendall(line.encode("utf-8"))

    def _wait_for_ack(self):
        """Block until one ACK/NACK byte is received from GCS.

        Returns True on ACK, False on NACK or timeout.
        Sets self.last_error on failure.
        """
        try:
            self._sock.settimeout(ACK_TIMEOUT_SEC)
            byte = self._sock.recv(1)
            if byte == ACK:
                self.last_error = None
                return True
            elif byte == NACK:
                self.last_error = "NACK"
                return False
            else:
                self.last_error = f"unexpected_byte: {byte!r}"
                return False
        except socket.timeout:
            self.last_error = "timeout"
            return False
        except OSError as exc:
            self.last_error = f"socket_error: {exc}"
            return False

    def _close_socket(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

def send_telemetry_now(telem_dict):
    """Open a fresh TCP connection to DATA_PORT, send one telemetry packet, close.

    Independent of any Transfer instance — safe to call at any state transition.
    Non-fatal: returns True on ACK, False if GCS is unreachable or refused.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(ACK_TIMEOUT_SEC)
        sock.connect((GROUND_STATION_IP, DATA_PORT))
        header, payload = build_telemetry_header(telem_dict)
        line = json.dumps(header, default=str) + "\n"
        sock.sendall(line.encode("utf-8"))
        sock.sendall(payload)
        sock.settimeout(ACK_TIMEOUT_SEC)
        byte = sock.recv(1)
        return byte == ACK
    except OSError:
        return False
    finally:
        try:
            sock.close()
        except OSError:
            pass
