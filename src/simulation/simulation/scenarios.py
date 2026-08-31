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

    name                 : identifier, must match this scenario's key in SCENARIOS.
    start                : the grout line's start point (x, y), in metres.
    end                  : the grout line's end point (x, y), in metres. The grout line is
                           the straight segment from start to end.
    start_pose           : robot pose (x, y, theta) at launch.
                           x, y in metres, theta in radians. Offsetting it from the line
                           gives the robot something to converge from.
    extruder_tf          : extruder-tip offset ahead of base_link, in metres
    camera_half_width    : how far off the line the camera can still see it, in metres
    noise_extruder_sigma : std-dev of the Gaussian noise on extruder_distance, in metres.
    noise_angle_sigma    : std-dev of the Gaussian noise on angle_difference, in radians.
    faults               : detection-dropout windows [(t_start, t_end), ...], in seconds,
                           during which the camera reports valid=false (loses the line).
    """

    name: str
    start: Tuple[float, float]
    end: Tuple[float, float]
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    extruder_tf: float = 0.2
    camera_half_width: float = 0.15
    noise_extruder_sigma: float = 0.002
    noise_angle_sigma: float = 0.01
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
        # Start 4 cm to the right of the line (y = -0.04) and yawed 6 deg, so the
        # robot has to align itself before it can follow.
        start_pose=(0.0, -0.04, math.radians(6.0)),
    ),
    # "diagonal": Scenario(
    #     name="diagonal",
    #     start=(0.0, 0.0),
    #     end=(2.0, 1.0),               # a straight line at about 27 deg
    #     start_pose=(0.0, 0.0, 0.0),
    # ),
    # "dropout": Scenario(
    #     name="dropout",
    #     start=(0.0, 0.0),
    #     end=(2.0, 0.0),
    #     faults=[(3.0, 4.5)],          # camera loses the line for 1.5 s mid-run
    # ),
}


def get_scenario(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario '{name}'. available: {sorted(SCENARIOS)}")
    return SCENARIOS[name]
