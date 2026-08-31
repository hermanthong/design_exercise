# Design Exercise: Robotics Software Engineer
Fabrica AI

---

## What we're looking for

This exercise asks you to do two things a strong engineer on our team does regularly: architect a real-time robotics system, and write the code at the centre of it. Just as important to us is how you think through the problem, so please show your reasoning, the trade-offs you weigh, and the options you decide against.

After your submission, we will schedule a follow-up interview round where you walk us through your design and defend your decisions. We will focus on the architecture, interfaces, trade-offs, and other design considerations.

Treat this as a design challenge rather than a coding puzzle. This is, on some level, the technical work you would actually do on the team. We suggest budgeting about a day, but spend less or more as you see fit. We care about depth, judgement, and whether you take the work all the way to something that runs and deploys.

## The system: a grout-line follower

Design and build the software that grouts a single grout line. Assume the following workflow:
1. The robot has been positioned by the operator roughly around the start of one grout line.
2. On launch, the robot aligns itself to the grout line.
3. The robot follows the grout line while keeping its extruder aligned over the grout line and dispenses grout continuously as it moves.
4. The robot stops when the grout line ends.

**The system must be designed and implemented in ROS 2.** Our robot stack is ROS 2 based, so use ROS 2 nodes, topics and services, QoS, and executors as your building blocks throughout. The robot has the following hardware for your software to orchestrate. Treat each one as a black box, which the simulation replaces.

- **Downward-facing camera.** Sees the grout line beneath the robot and reports where the extruder sits relative to it: a lateral distance and a heading error. Moderate rate, 15 Hz, noisy.
- **Wheel motors.** Two wheel motors in a differential drive. You command a forward velocity and an angular velocity.
- **Wheel encoders.** Report odometry.
- **Extruder motor.** Dispenses grout while it is switched on.

## Simulation

We provide a lightweight ROS 2 (Humble) simulation, the `simulation` package, that plays the world and all four black boxes: it moves the robot according to your `/cmd_vel` and reports what the downward camera sees. It builds with a plain `colcon build`. Computer vision is solved for you here, and the simulation hands you the grout line position directly (with noise).

**The interface.** This contract is fixed. Design your own nodes and internal messages around it.

The simulation publishes:

- `/line_detection` (`msgs/LineDetection`, ~15 Hz): `extruder_distance` (m, signed lateral distance of the extruder from the grout line), `angle_difference` (rad, heading versus the grout line tangent), and `valid` (bool).
- `/odom` (`nav_msgs/Odometry`, ~50 Hz): noisy odometry from the encoders.

The simulation subscribes:

- `/cmd_vel` (`geometry_msgs/Twist`): `linear.x` and `angular.z`.
- `/grout_on` (`std_msgs/Bool`): extruder on or off.

`LineDetection` is:

```
std_msgs/Header header
float64 extruder_distance    # metres, signed: +ve = grout line is to the robot's left
float64 angle_difference   # radians, signed heading error versus the grout line tangent
bool    valid              # false when the camera has no grout line beneath it
```

### Design considerations
1. `extruder_distance` and `angle_difference` carry noise, but the controller you design should converge the robot to the grout line.
2. The camera is not always able to see the grout line. What your system does then is up to your architecture.

### Scenarios
The simulation ships one scenario, `happy_path`: a straight grout line with nominal noise. It is deliberately easy. Add your own scenarios to test cases with higher noise, different geometries, etc.

### Evaluation
Use these metrics to evaluate the run.

- **Coverage.** How much of the grout line actually received grout. A robot that stops early or skips stretches does poorly here. A stretch of the grout line counts as covered when the extruder passed within 1 mm of it while dispensing.
- **Average distance from the grout line.** On average, how far the extruder sat from the grout line while dispensing.
- **Worst-case distance from the grout line.** The largest single deviation during the run.

We will not hand you a specific target to hit. These metrics are here so you can support your claims with something you measured during a simulation run.

## Part A: System architecture

Produce a design document that covers the following.

- **A ROS 2 node architecture, with a diagram.** Show all the nodes, what each one owns, and the topics between them, including what each message carries. Cover sensing (the camera and odometry), the alignment control that keeps the extruder on the grout line, extruder control, and start and stop management.
- **Topics, services, and actions.** Which interactions are plain topics, which are services or actions, and why you chose each.
- **Failure handling.** Name at least one thing that can go wrong mid-run, and explain how your architecture detects it, handles it, and reaches a safe state.

## Part B: Build it

**Write runnable ROS 2 code that follows and grouts the grout line,** implemented as one or more ROS 2 nodes running against the provided simulation. It should genuinely work: consume `/line_detection`, keep the extruder aligned over the grout line, drive `/cmd_vel`, gate the extruder with `/grout_on`, and stop when the grout line ends. Use C++ or Python (rclcpp or rclpy), whichever lets you do your best work.

Using the scenario framework, write scenarios that exercise the failure modes you described in Part A, and demonstrate that your system handles them.

Finally, make it deployable. Include build, run and launch instructions. We will execute your instructions on a clean machine running Ubuntu 22.04 during evaluation.

## Part C: Your thinking

Throughout, show your work. Tell us the alternatives you considered and set aside, the assumptions you made, which part you judged the hardest or highest risk and how you de-risked it, and what you would do next given more time or real hardware. We read this as closely as the code.

## Deliverables

- A design document covering Parts A and C, in any format. A diagram is expected for the architecture.
- Your Part B code as a git fork of this repo, with run and launch instructions and any tests.

## AI tools policy

Using AI tools is allowed for this exercise. **Declare all AI-generated code in the design document.** Note that we expect you to understand and defend everything you submit.

*If a requirement is ambiguous, state your assumption and carry on. Making sensible assumptions explicit is part of the job. Only ask us if you are genuinely blocked and have no idea how to continue.*
