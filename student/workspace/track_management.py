"""Track initialization, scoring, and deletion helpers.

Part H supplies lidar-driven existence decisions (docs/HUONG_DAN_KY_THUAT.md §2).
Use tracking parameters for the score window, thresholds, and covariance limit.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from fusion_lab.workspace_support import get_tracking_params


def init_track_state_from_meas(meas: Any) -> dict[str, Any]:
    """Initialize track state, covariance, lifecycle state, and score from a measurement.

    Args:
        meas: Lidar measurement with ``z``, ``R``, ``sensor``.

    Returns:
        Dict with keys ``x``, ``P``, ``state``, ``score`` (matrices as ``np.matrix``).
    """
    params = get_tracking_params()
    sens_to_veh = np.asarray(meas.sensor.sens_to_veh, dtype=float)
    rot = sens_to_veh[:3, :3]
    trans = sens_to_veh[:3, 3]

    pos_sens = np.asarray(meas.z, dtype=float).reshape(3)
    pos_veh = rot @ pos_sens + trans

    x = np.zeros((6, 1), dtype=float)
    x[:3, 0] = pos_veh

    R_sens = np.asarray(meas.R, dtype=float)
    P_pos = rot @ R_sens @ rot.T
    P_vel = np.diag([params.sigma_p44**2, params.sigma_p55**2, params.sigma_p66**2])

    P = np.zeros((6, 6), dtype=float)
    P[:3, :3] = P_pos
    P[3:, 3:] = P_vel

    score = 1.0 / params.window
    state = "initialized"

    return {
        "x": np.asmatrix(x),
        "P": np.asmatrix(P),
        "state": state,
        "score": float(score),
    }


def update_track_score(track: dict[str, Any], associated: bool) -> dict[str, Any]:
    """Update existence once per lidar frame; camera passes never call this helper.

    A hit adds 1/window, capped at one; an in-FOV miss subtracts 1/window.
    Confirm above confirmed_threshold, and preserve confirmed state after misses.

    Args:
        track: Dict-like track with ``score``, ``state``.
        associated: True for a lidar hit; False for a lidar miss within the lidar FOV.

    Returns:
        Updated track dict.
    """
    params = get_tracking_params()
    score = track["score"]
    state = track["state"]
    step = 1.0 / params.window

    if associated:
        score = min(1.0, score + step)
        if score > params.confirmed_threshold:
            state = "confirmed"
        elif state != "confirmed":
            state = "tentative"
    else:
        score = score - step

    track["score"] = float(score)
    track["state"] = state
    return track


def should_delete_track(track: dict[str, Any]) -> bool:
    """Return whether a lidar lifecycle pass should remove this track.

    Delete if either horizontal variance exceeds max_P, or if a confirmed
    track has score < delete_threshold, or an unconfirmed track has score <= 0.
    Camera passes never trigger deletion.

    Args:
        track: Dict with ``score``, ``state``, ``P``.

    Returns:
        True if track should be removed.
    """
    params = get_tracking_params()
    P = np.asarray(track["P"])
    if P[0, 0] > params.max_P or P[1, 1] > params.max_P:
        return True

    state = track.get("state")
    score = track.get("score", 0.0)

    if state == "confirmed":
        return bool(score < params.delete_threshold)
    else:
        return bool(score <= 0.0)
