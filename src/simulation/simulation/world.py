"""The simulated world: robot kinematics + the black-box downward camera.

This is deliberately ROS-free so it can be unit-tested as plain Python. The ROS
node in :mod:`simulation.sim_node` is a thin wrapper around a :class:`World`.
"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import angle_wrap


@dataclass
class LineReading:
    """What the downward camera reports on ``/line_detection``."""

    extruder_distance: float
    angle_difference: float
    valid: bool


class World:
    """Differential-drive robot on a flat floor with a single grout line.

    The robot integrates a kinematics model from the commanded linear and angular
    velocity. The "camera" measures where the extruder tip sits relative to the
    grout line, the straight segment from ``start`` to ``end``.
    """

    def __init__(self, scenario):
        self.scenario = scenario
        self.start = np.asarray(scenario.start, dtype=float)
        self.end = np.asarray(scenario.end, dtype=float)
        segment = self.end - self.start
        self.length = float(np.linalg.norm(segment))
        if self.length == 0.0:
            raise ValueError("The scenario's start and end points must differ")
        self.tangent = segment / self.length
        self.tangent_angle = math.atan2(float(self.tangent[1]), float(self.tangent[0]))
        self.left_normal = np.array([-self.tangent[1], self.tangent[0]])
        x, y, theta = scenario.start_pose
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.time = 0.0
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self._rng = np.random.default_rng()

    def step(self, dt: float, linear_velocity: float, angular_velocity: float) -> None:
        """Advance the model by ``dt`` seconds under the commanded velocities."""
        self.last_linear_velocity = float(linear_velocity)
        self.last_angular_velocity = float(angular_velocity)
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta = angle_wrap(self.theta + angular_velocity * dt)
        self.time += dt

    def extruder_tip(self) -> np.ndarray:
        """World position of the extruder tip (a point ahead of base_link)."""
        length = self.scenario.extruder_tf
        return np.array(
            [self.x + length * math.cos(self.theta), self.y + length * math.sin(self.theta)]
        )

    def _in_dropout(self) -> bool:
        return any(t0 <= self.time <= t1 for (t0, t1) in self.scenario.faults)

    def sense(self, add_noise: bool = True) -> LineReading:
        """Produce a camera reading of the grout line under the extruder tip."""
        tip = self.extruder_tip()
        offset = tip - self.start
        along = float(offset @ self.tangent)        # distance along the line from start, metres
        lateral = float(offset @ self.left_normal)  # signed lateral offset of the tip, +ve to its left

        # extruder_distance reports where the line sits relative to the tip, so it
        # is the tip's lateral offset negated: +ve means the line is to the left.
        extruder_distance = -lateral
        angle_difference = angle_wrap(self.tangent_angle - self.theta)

        # The camera sees the line while the extruder tip is over it: within the
        # camera's lateral half-width of the line, past the start (minus a small
        # margin), before the end, and not during an injected dropout.
        out_of_view = abs(lateral) > self.scenario.camera_half_width
        before_start = along < 0.0
        beyond_end = along > self.length
        valid = not (out_of_view or before_start or beyond_end or self._in_dropout())

        if not valid:
            return LineReading(0.0, 0.0, False)
        if add_noise:
            extruder_distance += float(self._rng.normal(0.0, self.scenario.noise_extruder_sigma))
            angle_difference = angle_wrap(
                angle_difference + float(self._rng.normal(0.0, self.scenario.noise_angle_sigma))
            )
        return LineReading(extruder_distance, angle_difference, True)
