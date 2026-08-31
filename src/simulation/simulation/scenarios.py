"""Scenarios: the grout-line geometry and conditions a run plays out under.

A scenario is just data. The simulation ships exactly one, ``happy_path``.
Adding your own is part of the exercise -- register more below (or from your
own module) and select one with the ``scenario`` parameter / launch argument.
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Scenario:
    """A single run configuration.

    waypoints            : grout-line polyline, list of (x, y) in metres.
    start_pose           : robot (x, y, theta) at launch -- "roughly" on the line.
    extruder_forward     : extruder-tip offset ahead of base_link, metres.
    camera_half_width    : how far off the line the camera can still see it, metres.
    start_margin         : slack behind the line start before the camera loses it.
    noise_extruder_sigma : std-dev of Gaussian noise on extruder_distance, metres.
    noise_angle_sigma    : std-dev of Gaussian noise on angle_difference, radians.
    faults               : detection-dropout windows, list of (t_start, t_end) in secs.
    """

    name: str
    waypoints: List[Tuple[float, float]]
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    extruder_forward: float = 0.2
    camera_half_width: float = 0.15
    start_margin: float = 0.05
    noise_extruder_sigma: float = 0.002
    noise_angle_sigma: float = 0.01
    faults: List[Tuple[float, float]] = field(default_factory=list)


_REGISTRY = {}


def register(scenario: Scenario) -> Scenario:
    """Add a scenario to the registry (keyed by name)."""
    _REGISTRY[scenario.name] = scenario
    return scenario


def get_scenario(name: str) -> Scenario:
    if name not in _REGISTRY:
        raise KeyError(f"unknown scenario '{name}'. available: {list_scenarios()}")
    return _REGISTRY[name]


def list_scenarios() -> List[str]:
    return sorted(_REGISTRY)


# ---------------------------------------------------------------------------
# Shipped scenario: a straight 2 m grout line, robot launched slightly off it.
register(
    Scenario(
        name="happy_path",
        waypoints=[(0.0, 0.0), (2.0, 0.0)],
        start_pose=(0.0, -0.04, math.radians(6.0)),
    )
)

# ---------------------------------------------------------------------------
# Add your own scenarios here (or in your own package, importing `register`).
# A scenario is only data -- geometry, start pose, noise, and injected faults.
#
# register(Scenario(
#     name="curve",
#     waypoints=[(0.0, 0.0), (1.0, 0.0), (1.6, 0.4), (2.2, 0.4)],
#     start_pose=(0.0, -0.03, 0.0),
# ))
#
# register(Scenario(
#     name="dropout",
#     waypoints=[(0.0, 0.0), (2.0, 0.0)],
#     start_pose=(0.0, -0.04, 0.0),
#     faults=[(3.0, 4.5)],          # camera loses the line for 1.5 s mid-run
# ))
