# `simulation` — running the grout-line simulation

This package plays the world and the four hardware black boxes your nodes talk to.
For the exercise itself, see `DESIGN_EXERCISE.md` at the repo root.

## Build

Requires ROS 2 Humble.

```bash
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch simulation simulation.launch.py
```

While running, the simulation:

- **publishes** `/line_detection` (`msgs/LineDetection`) at ~15 Hz and `/odom` (`nav_msgs/Odometry`)
  at ~50 Hz, and broadcasts TF `odom -> base_link`;
- **subscribes** to `/cmd_vel` (`geometry_msgs/Twist`) and `/extrude` (`std_msgs/Bool`).


## Scenarios

The simulation ships one scenario, `happy_path`. Add your own by putting another entry in the `SCENARIOS`
dict in `simulation/scenarios.py`, then select it by name. For example, once you add a `diagonal` scenario:

```bash
ros2 launch simulation simulation.launch.py scenario:=diagonal
```
