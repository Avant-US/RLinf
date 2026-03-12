"""
Station ↔ Robot wire protocol (binary, fixed-size).

Design goals:
- No heavy dependencies on the station (only stdlib `socket` + `struct`)
- Simple + robust framing (fixed-size messages)
- Easy to implement in C++ without third-party libs

Endianness:
- Little-endian (`<` in struct format). This assumes x86_64 on both sides.

TCP ports (recommended):
- control / commands: 5555
- state stream:      5556 (control_port + 1)
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import ClassVar, Tuple


def now_ns() -> int:
    # Use monotonic ns for age computations; robot will use its own clock.
    return time.monotonic_ns()


class CmdType:
    HEARTBEAT = 0
    ARM_COMMAND = 1
    STOP = 2
    SHUTDOWN = 3  # stop robot_server process (if enabled on robot side)


class CmdMode:
    HOLD = 0
    JOINT_POSITION = 1
    JOINT_DELTA = 2
    JOINT_VELOCITY = 3


class Flags:
    ENABLE_ARM = 1 << 0
    ENABLE_GRIPPER = 1 << 1


@dataclass
class CommandMsgV1:
    """
    104 bytes total.
    """

    MAGIC: ClassVar[bytes] = b"FCM1"
    VERSION: ClassVar[int] = 1

    seq: int
    sent_time_ns: int
    cmd_type: int
    mode: int
    validity_ms: int
    flags: int
    q: Tuple[float, float, float, float, float, float, float]
    gripper: float

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<4sIQQIIII8d")

    def pack(self) -> bytes:
        q7 = tuple(float(x) for x in self.q)
        if len(q7) != 7:
            raise ValueError("q must have 7 elements")
        return self._STRUCT.pack(
            self.MAGIC,
            self.VERSION,
            int(self.seq),
            int(self.sent_time_ns),
            int(self.cmd_type),
            int(self.mode),
            int(self.validity_ms),
            int(self.flags),
            *q7,
            float(self.gripper),
        )

    @classmethod
    def unpack(cls, data: bytes) -> "CommandMsgV1":
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"bad CommandMsgV1 size: got={len(data)} expected={cls._STRUCT.size}")
        (
            magic,
            version,
            seq,
            sent_time_ns,
            cmd_type,
            mode,
            validity_ms,
            flags,
            d0,
            d1,
            d2,
            d3,
            d4,
            d5,
            d6,
            gripper,
        ) = cls._STRUCT.unpack(data)
        if magic != cls.MAGIC or version != cls.VERSION:
            raise ValueError(f"bad magic/version: magic={magic!r} version={version}")
        return cls(
            seq=int(seq),
            sent_time_ns=int(sent_time_ns),
            cmd_type=int(cmd_type),
            mode=int(mode),
            validity_ms=int(validity_ms),
            flags=int(flags),
            q=(float(d0), float(d1), float(d2), float(d3), float(d4), float(d5), float(d6)),
            gripper=float(gripper),
        )


@dataclass
class StateMsgV1:
    """
    160 bytes total.
    """

    MAGIC: ClassVar[bytes] = b"FST1"
    VERSION: ClassVar[int] = 1

    robot_time_ns: int
    last_cmd_seq: int
    robot_mode: int
    error_code: int
    q: Tuple[float, float, float, float, float, float, float]
    dq: Tuple[float, float, float, float, float, float, float]
    gripper: float
    cmd_age_ms: float

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<4sIQQII16d")

    @classmethod
    def unpack(cls, data: bytes) -> "StateMsgV1":
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"bad StateMsgV1 size: got={len(data)} expected={cls._STRUCT.size}")
        fields = cls._STRUCT.unpack(data)
        magic = fields[0]
        version = fields[1]
        if magic != cls.MAGIC or version != cls.VERSION:
            raise ValueError(f"bad magic/version: magic={magic!r} version={version}")
        robot_time_ns = int(fields[2])
        last_cmd_seq = int(fields[3])
        robot_mode = int(fields[4])
        error_code = int(fields[5])
        doubles = [float(x) for x in fields[6:]]
        q = tuple(doubles[0:7])
        dq = tuple(doubles[7:14])
        gripper = float(doubles[14])
        cmd_age_ms = float(doubles[15])
        return cls(
            robot_time_ns=robot_time_ns,
            last_cmd_seq=last_cmd_seq,
            robot_mode=robot_mode,
            error_code=error_code,
            q=q,  # type: ignore[arg-type]
            dq=dq,  # type: ignore[arg-type]
            gripper=gripper,
            cmd_age_ms=cmd_age_ms,
        )


# Sizes are part of the contract.
COMMAND_MSG_V1_SIZE = CommandMsgV1._STRUCT.size
STATE_MSG_V1_SIZE = StateMsgV1._STRUCT.size

