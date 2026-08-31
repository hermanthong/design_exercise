"""
Each Scenario contains the grout-line geometry and conditions a run plays out under.

The simulation ships exactly one Scenario, ``happy_path``.
Feel free to add more scenarios here to test the robustness of your solution.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class Scenario:
    """
    A single run configuration.

    name                       : identifier, must match this scenario's key in SCENARIOS.
    start                      : the grout line's start point (x, y), in metres.
    end                        : the grout line's end point (x, y), in metres. The grout line is
                                 a straight line from start to end.
    start_pose                 : robot pose (x, y, theta) at launch. x, y in metres, theta in radians.
    extruder_tf                : extruder-tip offset ahead of base_link, in metres
    camera_half_width          : how far off the line the camera can still see it, in metres
    camera_noise_linear_sigma  : std-dev of the Gaussian noise on lateral_error, in metres.
    camera_noise_angular_sigma : std-dev of the Gaussian noise on heading_error, in radians.
    camera_delay               : camera latency, in seconds.
    odom_drift_rate            : odometry drift std per unit of motion.
    odom_delay                 : odometry latency, in seconds.
    faults                     : detection-dropout windows [(t_start, t_end), ...], in seconds,
                                 during which the camera reports valid=false.
    """

    name: str
    start: Tuple[float, float]
    end: Tuple[float, float]
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    extruder_tf: float = 0.2
    camera_half_width: float = 0.15
    camera_noise_linear_sigma: float = 0.002
    camera_noise_angular_sigma: float = 0.01
    camera_delay: float = 0.3
    odom_drift_rate: float = 0.05
    odom_delay: float = 0.05
    faults: List[Tuple[float, float]] = field(default_factory=list)


"""
All scenarios, keyed by name. Add your own entries here.
"""
SCENARIOS: Dict[str, Scenario] = {
    "happy_path": Scenario(
        name="happy_path",
        # A straight grout line 2 m long, from the origin to (2, 0) along +x.
        start=(0.0, 0.0),
        end=(2.0, 0.0),
        # Start 4 cm to the right of the line (y = -0.04) and rotated 6 deg to the left
        start_pose=(0.0, -0.04, math.radians(6.0)),
    ),
}


def get_scenario(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario '{name}'. available: {sorted(SCENARIOS)}")
    return SCENARIOS[name]
