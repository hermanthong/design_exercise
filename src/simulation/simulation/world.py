"""The simulated world: robot kinematics + the black-box downward camera.

This is deliberately ROS-free so it can be unit-tested as plain Python. The ROS
node in :mod:`simulation.sim_node` is a thin wrapper around a :class:`World`.
"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import angle_wrap, project_to_polyline


@dataclass
class LineReading:
    """What the downward camera reports on ``/line_detection``."""

    extruder_distance: float
    angle_difference: float
    valid: bool


class World:
    """Differential-drive robot on a flat floor with a single grout line.

    The robot integrates a unicycle model from commanded (v, w). The "camera"
    measures where the extruder tip sits relative to the grout line.
    """

    def __init__(self, scenario, seed: int = 0):
        self.scn = scenario
        self.pts = [np.asarray(p, dtype=float) for p in scenario.waypoints]
        if len(self.pts) < 2:
            raise ValueError("a scenario needs at least two waypoints")
        x, y, theta = scenario.start_pose
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.time = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self._rng = np.random.default_rng(seed)

    # -- kinematics ---------------------------------------------------------
    def step(self, dt: float, v: float, w: float) -> None:
        """Advance the unicycle model by ``dt`` seconds under command (v, w)."""
        self.last_v = float(v)
        self.last_w = float(w)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = angle_wrap(self.theta + w * dt)
        self.time += dt

    def extruder_tip(self) -> np.ndarray:
        """World position of the extruder tip (a point ahead of base_link)."""
        length = self.scn.nozzle_forward
        return np.array(
            [self.x + length * math.cos(self.theta), self.y + length * math.sin(self.theta)]
        )

    # -- sensing ------------------------------------------------------------
    def _in_dropout(self) -> bool:
        return any(t0 <= self.time <= t1 for (t0, t1) in self.scn.faults)

    def sense(self, add_noise: bool = True) -> LineReading:
        """Produce a camera reading of the grout line under the extruder tip."""
        tip = self.extruder_tip()
        pr = project_to_polyline(tip, self.pts)
        ang = pr["ang"]
        left_normal = np.array([-math.sin(ang), math.cos(ang)])
        extruder_distance = float((pr["nearest"] - tip) @ left_normal)
        angle_difference = angle_wrap(ang - self.theta)

        # Validity: the camera loses the line past the end, before the start,
        # if the tip strays outside the camera footprint, or during a dropout.
        first, second = self.pts[0], self.pts[1]
        last, prev = self.pts[-1], self.pts[-2]
        t_start = (second - first) / np.linalg.norm(second - first)
        t_end = (last - prev) / np.linalg.norm(last - prev)
        beyond_end = float((tip - last) @ t_end) > 0.0
        before_start = float((tip - first) @ t_start) < -self.scn.start_margin
        out_of_view = pr["dist"] > self.scn.camera_half_width
        valid = not (beyond_end or before_start or out_of_view or self._in_dropout())

        if not valid:
            return LineReading(0.0, 0.0, False)
        if add_noise:
            extruder_distance += float(self._rng.normal(0.0, self.scn.noise_extruder_sigma))
            angle_difference = angle_wrap(
                angle_difference + float(self._rng.normal(0.0, self.scn.noise_angle_sigma))
            )
        return LineReading(extruder_distance, angle_difference, True)
