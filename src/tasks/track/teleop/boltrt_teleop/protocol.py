"""Wire protocol shared by the teleop publishers and `boltrt --task tracking`."""

from __future__ import annotations

import struct

import numpy as np

# Default endpoints. boltrt subscribes with --motion tcp://<host>:<port>.
MOTION_PORT = 28701  # robot reference JSON
SMPL_PORT = 28702  # SONIC SMPL reference (JSON or packed)
PICO_CONTROLLER_PORT = 5592  # Pico button states (binary)
HAND_GRIP_PORT = 5593  # analog trigger grips (binary)

SMPLX_T_NS_KEY = "smplx_t_ns"
PICO_RECV_TIME_NS_KEY = "pico_recv_time_ns"
PUBLISH_T_NS_KEY = "publish_t_ns"
SEQ_KEY = "seq"
MOTION_FIRST_FRAME_KEY = "motion_first_frame"
JOINT_NAMES_KEY = "joint_names"
BODY_NAMES_KEY = "body_names"
JOINT_POS_KEY = "joint_pos"
JOINT_VEL_KEY = "joint_vel"
BODY_POS_W_KEY = "body_pos_w"
BODY_QUAT_W_KEY = "body_quat_w"
XROBOT_BODY_NAMES_KEY = "xrobot_body_names"
XROBOT_BODY_POS_W_KEY = "xrobot_body_pos_w"
XROBOT_BODY_QUAT_W_KEY = "xrobot_body_quat_w"

_PICO_CONTROLLER_STATE_STRUCT = struct.Struct("<QBBBB")
_HAND_GRIP_STRUCT = struct.Struct("<Qff")


class PicoControllerStateMessage:
    """Binary message containing Pico controller button states."""

    def __init__(self, timestamp_ns: int = 0, A: bool = False, B: bool = False, X: bool = False, Y: bool = False):
        self.timestamp_ns = int(timestamp_ns)
        self.A, self.B, self.X, self.Y = bool(A), bool(B), bool(X), bool(Y)

    def to_bytes(self) -> bytes:
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be non-negative")
        return _PICO_CONTROLLER_STATE_STRUCT.pack(self.timestamp_ns, int(self.A), int(self.B), int(self.X), int(self.Y))

    @classmethod
    def from_bytes(cls, data: bytes) -> "PicoControllerStateMessage":
        if len(data) != _PICO_CONTROLLER_STATE_STRUCT.size:
            raise ValueError(f"expected {_PICO_CONTROLLER_STATE_STRUCT.size} bytes, got {len(data)}")
        t, a, b, x, y = _PICO_CONTROLLER_STATE_STRUCT.unpack(data)
        return cls(t, bool(a), bool(b), bool(x), bool(y))


class HandGripMessage:
    """Binary message containing normalized left/right hand grip commands."""

    def __init__(self, timestamp_ns: int = 0, left_grip: float = 0.0, right_grip: float = 0.0):
        self.timestamp_ns = int(timestamp_ns)
        self.left_grip, self.right_grip = float(left_grip), float(right_grip)
        self._validate()

    def _validate(self) -> None:
        if not 0 <= self.timestamp_ns <= 2**64 - 1:
            raise ValueError("timestamp_ns must fit uint64")
        for name, value in (("left_grip", self.left_grip), ("right_grip", self.right_grip)):
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be finite and within [0, 1]")

    def to_bytes(self) -> bytes:
        self._validate()
        return _HAND_GRIP_STRUCT.pack(self.timestamp_ns, self.left_grip, self.right_grip)

    @classmethod
    def from_bytes(cls, data: bytes) -> "HandGripMessage":
        if len(data) != _HAND_GRIP_STRUCT.size:
            raise ValueError(f"expected {_HAND_GRIP_STRUCT.size} bytes, got {len(data)}")
        return cls(*_HAND_GRIP_STRUCT.unpack(data))
