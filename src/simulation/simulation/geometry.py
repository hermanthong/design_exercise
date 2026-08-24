"""Pure 2-D geometry helpers for the grout-line simulation.

No ROS dependencies live here so the maths can be unit-tested directly.
"""
import math

import numpy as np


def angle_wrap(a: float) -> float:
    """Wrap an angle to the range [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _project_point_to_segment(p, a, b):
    """Return (nearest_point, t, seg_length) for point ``p`` onto segment a->b.

    ``t`` is the clamped parameter in [0, 1] along the segment.
    """
    ab = b - a
    length2 = float(ab @ ab)
    if length2 == 0.0:
        return a.copy(), 0.0, 0.0
    t = float((p - a) @ ab) / length2
    t_clamped = min(1.0, max(0.0, t))
    nearest = a + t_clamped * ab
    return nearest, t_clamped, math.sqrt(length2)


def project_to_polyline(p, pts):
    """Project point ``p`` onto a polyline given as a list of 2-D np arrays.

    Returns a dict with the nearest point, the perpendicular ``dist``, the
    arc-length ``s`` of the nearest point from the polyline start, the total
    polyline length, the index of the closest segment, and the tangent angle
    ``ang`` of that segment.
    """
    best = None
    cumulative = 0.0
    for i in range(len(pts) - 1):
        a = pts[i]
        b = pts[i + 1]
        nearest, t, seg_len = _project_point_to_segment(p, a, b)
        dist = float(np.hypot(*(p - nearest)))
        s = cumulative + t * seg_len
        if best is None or dist < best["dist"]:
            tangent = b - a
            best = {
                "nearest": nearest,
                "dist": dist,
                "s": s,
                "seg": i,
                "ang": math.atan2(float(tangent[1]), float(tangent[0])),
            }
        cumulative += seg_len
    best["total"] = cumulative
    return best
