# `simulation` — running the grout-line simulation

This package plays the world and the four hardware black boxes your nodes talk to. It does not score a run; it
only simulates. For the exercise itself, see `DESIGN_EXERCISE.md` at the repo root.

## Build

Requires ROS 2 Humble.

```bash
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch simulation simulation.launch.py
# choose a scenario / seed:
ros2 launch simulation simulation.launch.py scenario:=happy_path seed:=0
```

While running, the simulation:

- **publishes** `/line_detection` (`msgs/LineDetection`) at ~15 Hz and `/odom` (`nav_msgs/Odometry`)
  at ~50 Hz, and broadcasts TF `odom -> base_link`;
- **subscribes** to `/cmd_vel` (`geometry_msgs/Twist`) and `/grout_on` (`std_msgs/Bool`).

## Drive it by hand

With the simulation running, in a second sourced shell:

```bash
# watch what the camera reports
ros2 topic echo /line_detection

# drive forward at 0.3 m/s (held until you change it)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# forward + gentle left turn, streamed at 10 Hz
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"

# turn the extruder on
ros2 topic pub --once /grout_on std_msgs/msg/Bool "{data: true}"

# stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Scenarios

The simulation ships one scenario, `happy_path`. Add your own in `simulation/scenarios.py` (or from your own
package by importing `register`) and select it with `scenario:=<name>`.
