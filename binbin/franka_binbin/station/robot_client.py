"""
Robot-side TCP client (station).

Uses the fixed-size binary protocol in `station/robot_protocol.py`.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from station.robot_protocol import (
    COMMAND_MSG_V1_SIZE,
    STATE_MSG_V1_SIZE,
    CmdMode,
    CmdType,
    CommandMsgV1,
    Flags,
    StateMsgV1,
    now_ns,
)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)


@dataclass
class RobotConnectionParams:
    host: str
    command_port: int = 5555
    state_port: Optional[int] = None  # default: command_port + 1
    connect_timeout_s: float = 3.0


class RobotClient:
    def __init__(self, params: RobotConnectionParams) -> None:
        self._params = params
        self._cmd_sock: Optional[socket.socket] = None
        self._state_sock: Optional[socket.socket] = None

        self._seq = 0
        self._latest_state: Optional[StateMsgV1] = None
        self._latest_state_rx_ns: Optional[int] = None  # station monotonic ns
        self._state_error: Optional[str] = None
        self._latest_state_lock = threading.Lock()

        self._stop_evt = threading.Event()
        self._state_thread: Optional[threading.Thread] = None

    def connect(self) -> None:
        if self._cmd_sock is not None:
            return

        # Command channel is required.
        self._cmd_sock = socket.create_connection(
            (self._params.host, int(self._params.command_port)),
            timeout=self._params.connect_timeout_s,
        )
        self._cmd_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # State stream is best-effort: stop/jog should still work if state port is unavailable.
        state_port = self._params.state_port
        if state_port is None:
            state_port = int(self._params.command_port) + 1
        try:
            self._state_sock = socket.create_connection(
                (self._params.host, int(state_port)),
                timeout=self._params.connect_timeout_s,
            )
            self._state_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Keep recv loop interruptible so shutdown can join cleanly.
            self._state_sock.settimeout(0.2)
        except Exception as e:
            self._state_sock = None
            with self._latest_state_lock:
                self._state_error = f"state stream connect failed: {e}"

        self._stop_evt.clear()
        with self._latest_state_lock:
            self._latest_state = None
            self._latest_state_rx_ns = None
            # Preserve any state connection error set above.
            if self._state_sock is not None:
                self._state_error = None
        if self._state_sock is not None:
            self._state_thread = threading.Thread(target=self._state_loop, name="robot_state_loop", daemon=False)
            self._state_thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        state_thread = self._state_thread
        self._state_thread = None

        # Close sockets first so a blocked recv/send unblocks immediately.
        for s in (self._state_sock, self._cmd_sock):
            if s is None:
                continue
            try:
                s.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                s.close()
            except Exception:
                pass
        self._state_sock = None
        self._cmd_sock = None

        if state_thread is not None:
            state_thread.join(timeout=2.0)

    def is_connected(self) -> bool:
        # Command channel is what we need to send commands; state stream is optional.
        return self._cmd_sock is not None

    def is_state_connected(self) -> bool:
        return self._state_sock is not None

    def get_latest_state(self) -> Optional[StateMsgV1]:
        with self._latest_state_lock:
            return self._latest_state

    def latest_state_age_s(self) -> Optional[float]:
        with self._latest_state_lock:
            rx_ns = self._latest_state_rx_ns
        if rx_ns is None:
            return None
        return (time.monotonic_ns() - int(rx_ns)) / 1e9

    def get_state_error(self) -> Optional[str]:
        with self._latest_state_lock:
            return self._state_error

    def send_stop(self, *, validity_ms: int = 500) -> None:
        self._send(
            cmd_type=CmdType.STOP,
            mode=CmdMode.HOLD,
            q=(0.0,) * 7,
            gripper=0.0,
            validity_ms=validity_ms,
            flags=0,
        )

    def send_shutdown(self, *, validity_ms: int = 500) -> None:
        """
        Request the robot_server process to shut down.

        Note: This only works if robot_server was started with shutdown enabled
        (e.g. `--allow-shutdown`).
        """
        self._send(
            cmd_type=CmdType.SHUTDOWN,
            mode=CmdMode.HOLD,
            q=(0.0,) * 7,
            gripper=0.0,
            validity_ms=validity_ms,
            flags=0,
        )

    def send_arm_command(
        self,
        *,
        q: Tuple[float, float, float, float, float, float, float],
        gripper: float,
        mode: int,
        validity_ms: int,
        enable_arm: bool,
        enable_gripper: bool = False,
    ) -> None:
        flags = 0
        if enable_arm:
            flags |= Flags.ENABLE_ARM
        if enable_gripper:
            flags |= Flags.ENABLE_GRIPPER
        self._send(
            cmd_type=CmdType.ARM_COMMAND,
            mode=mode,
            q=q,
            gripper=gripper,
            validity_ms=validity_ms,
            flags=flags,
        )

    def send_velocity_command(
        self,
        *,
        vel: Tuple[float, float, float, float, float, float, float],
        gripper: float,
        validity_ms: int,
        enable_arm: bool,
        enable_gripper: bool = False,
    ) -> None:
        """Send normalized joint velocities [-1, 1] directly.

        robot_server converts to position deltas internally:
            q_setpoint += clip(vel, -1, 1) * max_joint_delta
        """
        flags = 0
        if enable_arm:
            flags |= Flags.ENABLE_ARM
        if enable_gripper:
            flags |= Flags.ENABLE_GRIPPER
        self._send(
            cmd_type=CmdType.ARM_COMMAND,
            mode=CmdMode.JOINT_VELOCITY,
            q=vel,
            gripper=gripper,
            validity_ms=validity_ms,
            flags=flags,
        )

    # -----------------------
    # Internals
    # -----------------------

    def _send(
        self,
        *,
        cmd_type: int,
        mode: int,
        q: Tuple[float, float, float, float, float, float, float],
        gripper: float,
        validity_ms: int,
        flags: int,
    ) -> None:
        if self._cmd_sock is None:
            raise ConnectionError("robot command socket not connected")
        self._seq += 1
        msg = CommandMsgV1(
            seq=self._seq,
            sent_time_ns=now_ns(),
            cmd_type=int(cmd_type),
            mode=int(mode),
            validity_ms=int(validity_ms),
            flags=int(flags),
            q=q,
            gripper=float(gripper),
        )
        payload = msg.pack()
        assert len(payload) == COMMAND_MSG_V1_SIZE
        self._cmd_sock.sendall(payload)

    def _state_loop(self) -> None:
        assert self._state_sock is not None
        sock = self._state_sock
        while not self._stop_evt.is_set():
            try:
                data = _recv_exact(sock, STATE_MSG_V1_SIZE)
                st = StateMsgV1.unpack(data)
                with self._latest_state_lock:
                    self._latest_state = st
                    self._latest_state_rx_ns = time.monotonic_ns()
            except socket.timeout:
                continue
            except Exception as e:
                if self._stop_evt.is_set():
                    break
                # If the state stream breaks, stop updating; caller can reconnect.
                with self._latest_state_lock:
                    self._state_error = f"state stream stopped: {e}"
                break
        try:
            sock.close()
        except Exception:
            pass
        if self._state_sock is sock:
            self._state_sock = None

