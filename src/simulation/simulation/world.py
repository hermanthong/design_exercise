"""The simulated world: robot kinematics + the black-box downward camera.

This is deliberately ROS-free so it can be unit-tested as plain Python. The ROS
node in :mod:`simulation.sim_node` is a thin wrapper around a :class:`World`.
"""
import math
from dataclasses import dataclass

import numpy as np

from .geometry import angle_wrap

# Physical limits of the differential-drive base.
MAX_LINEAR_VELOCITY = 0.3       # m/s
MAX_ANGULAR_VELOCITY = 1.0      # rad/s
MAX_LINEAR_ACCELERATION = 1.0   # m/s^2
MAX_ANGULAR_ACCELERATION = 3.0  # rad/s^2


def _clamp(value: float, limit: float) -> float:
    """Clamp ``value`` to the symmetric range [-limit, limit]."""
    return max(-limit, min(limit, value))


@dataclass
class LineReading:
    """What the downward camera reports on ``/line_detection``."""

    lateral_error: float
    heading_error: float
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
        self.v = 0.0  # current linear velocity
        self.w = 0.0  # current angular velocity
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self._rng = np.random.default_rng()

    def step(self, dt: float, linear_velocity: float, angular_velocity: float) -> None:
        """Advance the model by ``dt`` seconds under the commanded velocities.

        The commands are clamped to the base's top speeds and rate-limited by its
        maximum accelerations, so the achieved velocity ramps toward the command.
        """
        v_cmd = _clamp(float(linear_velocity), MAX_LINEAR_VELOCITY)
        w_cmd = _clamp(float(angular_velocity), MAX_ANGULAR_VELOCITY)
        self.v += _clamp(v_cmd - self.v, MAX_LINEAR_ACCELERATION * dt)
        self.w += _clamp(w_cmd - self.w, MAX_ANGULAR_ACCELERATION * dt)
        self.last_linear_velocity = self.v
        self.last_angular_velocity = self.w
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = angle_wrap(self.theta + self.w * dt)
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

        # lateral_error reports where the line sits relative to the tip, so it
        # is the tip's lateral offset negated: +ve means the line is to the left.
        lateral_error = -lateral
        heading_error = angle_wrap(self.tangent_angle - self.theta)

        # calculate if line is in field of view
        half_width = self.scenario.camera_half_width
        out_of_view = abs(lateral) > half_width
        before_start = along < -half_width
        beyond_end = along > self.length + half_width
        valid = not (out_of_view or before_start or beyond_end or self._in_dropout())

        if not valid:
            return LineReading(0.0, 0.0, False)
        if add_noise:
            lateral_error += float(self._rng.normal(0.0, self.scenario.camera_noise_linear_sigma))
            heading_error = angle_wrap(
                heading_error + float(self._rng.normal(0.0, self.scenario.camera_noise_angular_sigma))
            )
        return LineReading(lateral_error, heading_error, True)
