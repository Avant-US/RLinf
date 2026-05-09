# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Centralised gripper [-1, 1] <-> business pct [0, 100] mapping.

Per the design doc r1pro6op47.md §3.4, the Galaxea G1 gripper has a
physical command range of ``[0, 100]`` (treated as a linear stroke
percentage by the SDK), but business logic often wants to expose a
narrower range to the policy -- for example a tighter ``[0, 90]`` to
avoid the gripper bouncing against the case at full open, or a
``[10, 80]`` to leave both ends as safe margin.

This helper does two things and is shared by every place that needs to
convert between the policy-side normalised value in ``[-1, 1]`` and the
ROS2-side physical value in ``[0, 100]``:

1. ``action11_to_pct(a)``: policy action -> physical pct, linear map
   from ``[-1, 1]`` onto the configurable business range ``[gmin_pct,
   gmax_pct]``, then clipped to physical ``[0, 100]`` as a defensive
   bound (in case ``gmax_pct`` is misconfigured > 100).

2. ``pct_to_obs11(pct)``: physical pct -> policy observation, the
   inverse map.  The pct is first clipped to the business range so that
   feedback values outside the policy's visible range don't pollute
   observations with constant ``+/-1``.

Edge cases handled:

* ``gmin_pct >= gmax_pct`` -> ``ValueError`` at construction (no silent
  fallback; mis-configuration is a deployment bug we want to catch
  early).
* ``gmin_pct < 0`` or ``gmax_pct > 100`` -> ``ValueError`` (physical
  device range is hard).
* Action input outside ``[-1, 1]`` -> defensively clipped (rather than
  raising) because the L1 safety layer is supposed to clip first; we
  don't want the mixer to crash if upstream forgot.
"""

from __future__ import annotations

import numpy as np


class GripperMixer:
    """Linear bidirectional map between policy ``[-1, 1]`` and physical
    pct ``[0, 100]`` through a configurable business range
    ``[gmin_pct, gmax_pct]``.

    Args:
        gmin_pct: Lower bound of the policy-visible range, in the
            physical pct units of the device (defaults to 0.0).
        gmax_pct: Upper bound of the policy-visible range
            (defaults to 90.0 -- a common conservative open limit
            that avoids bouncing the case).

    Raises:
        ValueError: When ``gmax_pct <= gmin_pct`` or either bound is
            outside ``[0, 100]``.
    """

    PHYSICAL_MIN: float = 0.0
    PHYSICAL_MAX: float = 100.0

    def __init__(self, gmin_pct: float = 0.0, gmax_pct: float = 90.0):
        gmin = float(gmin_pct)
        gmax = float(gmax_pct)
        if not (self.PHYSICAL_MIN <= gmin < gmax <= self.PHYSICAL_MAX):
            raise ValueError(
                f"GripperMixer requires "
                f"{self.PHYSICAL_MIN} <= gmin_pct < gmax_pct "
                f"<= {self.PHYSICAL_MAX}; "
                f"got gmin_pct={gmin_pct}, gmax_pct={gmax_pct}."
            )
        self._gmin = gmin
        self._gmax = gmax

    @property
    def gmin_pct(self) -> float:
        return self._gmin

    @property
    def gmax_pct(self) -> float:
        return self._gmax

    @property
    def span_pct(self) -> float:
        return self._gmax - self._gmin

    def action11_to_pct(self, a: float) -> float:
        """Map policy action ``a`` in ``[-1, 1]`` to physical pct.

        ``-1`` -> ``gmin_pct``; ``+1`` -> ``gmax_pct``; ``0`` -> midpoint.
        Out-of-range action is clipped first; result is finally clipped
        to physical ``[0, 100]`` as a defensive bound.
        """
        a = float(np.clip(a, -1.0, 1.0))
        pct = self._gmin + (a + 1.0) * 0.5 * self.span_pct
        return float(np.clip(pct, self.PHYSICAL_MIN, self.PHYSICAL_MAX))

    def pct_to_obs11(self, pct: float) -> float:
        """Map physical pct in ``[0, 100]`` to policy observation in
        ``[-1, 1]``.

        The pct is first clipped to the business range ``[gmin_pct,
        gmax_pct]`` so that feedback values outside the policy's
        visible range saturate at ``+/-1`` instead of producing
        out-of-range observations.
        """
        pct = float(np.clip(pct, self._gmin, self._gmax))
        norm01 = (pct - self._gmin) / max(self.span_pct, 1e-9)
        return float(np.clip(2.0 * norm01 - 1.0, -1.0, 1.0))

    def __repr__(self) -> str:
        return f"GripperMixer(gmin_pct={self._gmin:.1f}, gmax_pct={self._gmax:.1f})"
