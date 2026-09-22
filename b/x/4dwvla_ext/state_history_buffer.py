"""Buffers arm joint angles actually executed between two inference calls.

Fixes R1/修复 A in ``b/d/frk1/grperr_1.md``: the keypoint-history clock
(``observation.his_len``) must advance once per *control step* (matching the
30 Hz training semantics of ``Extract3DKeypointTransformFn``), not once per
*inference call*. Since inference only happens every ``n_exec`` control
steps, the client records every measured arm pose it executes in between two
inferences and ships them to the server, which replays them through
``FKKeypointComputer.step()`` before computing the keypoints for the current
frame. See ``b/x/4dwvla_ext/fk_keypoints.py`` and
``4WVLA/src/lerobot/policies/internvla_a1_5/transform_internvla_a1_5.py``
(``Extract3DKeypointTransformFn``) for the training-side semantics this is
meant to reproduce.
"""
from __future__ import annotations

from collections import deque
from typing import Iterable

import numpy as np


class ExecutedStateBuffer:
    """FIFO buffer of measured 7D arm joint angles, drained once per inference.

    ``max_len`` only bounds pathological cases (e.g. a very large ``n_exec``);
    under normal operation at most ``n_exec`` entries accumulate between two
    calls to :meth:`drain`.
    """

    def __init__(self, max_len: int = 512) -> None:
        self._buffer: deque[list[float]] = deque(maxlen=max_len)

    def record(self, arm_q7: Iterable[float]) -> None:
        """Append one measured arm pose (radians, 7 joints)."""
        self._buffer.append([float(v) for v in np.asarray(arm_q7).reshape(-1)[:7]])

    def drain(self) -> list[list[float]]:
        """Return and clear all poses recorded since the last drain/clear."""
        drained = list(self._buffer)
        self._buffer.clear()
        return drained

    def clear(self) -> None:
        """Discard any buffered poses (call at episode boundaries)."""
        self._buffer.clear()

    def __len__(self) -> int:
        return len(self._buffer)
