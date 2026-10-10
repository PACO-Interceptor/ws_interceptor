# ws_interceptor

`ws_interceptor` is a ROS 2 workspace that connects two simulated PX4
vehicles: an **interceptor**, which tries to reach a **target**. Our own code
is in [`src/interceptor`](../../src/interceptor). The workspace also contains
three vendored dependencies: `px4_msgs`, `px4_ros_com` and
`px4-ros2-interface-lib`, which provide the PX4 messages, the coordinate
conversions and the library to register custom flight modes.

## One-page summary

PX4 publishes odometry, the nodes convert it and relate it through tf2, and a
guidance mode computes a setpoint that PX4 tries to follow. If this is your
first time here, this is all you need before going into detail:

- There are two drones simulated with PX4: the **interceptor** (the one that
  chases) and the **target** (the one being chased).
- Each PX4 publishes its position and velocity (`VehicleOdometry`) on a
  different ROS 2 topic: instance `0` for the interceptor, instance `1` for
  the target.
- Two nodes (`interceptor_tf2_odometry`, `target_tf2_odometry`) convert that
  data from PX4 coordinates (NED) to ROS coordinates (ENU) and publish the
  result as tf2 transforms: "where each drone is relative to a common origin
  called `map`".
- The target node also publishes its velocity on a separate topic
  (`target/velocity`), because tf2 only stores position and orientation, not
  velocity.
- Two flight modes (`pursuit_mode` and `PN_mode`) read that position (and, for
  PN, the velocity too) and compute where the interceptor should move. They
  hand it to PX4 as a *setpoint* (a velocity/acceleration reference); PX4 does
  the actual moving.
- `pursuit_mode` is the simplest: it points straight at the target's current
  position. `PN_mode` is more elaborate: it anticipates the target's motion
  using its velocity (proportional navigation).
- The package also includes helper nodes for inspection and diagnostics, which
  only print data to the console and don't take part in flight control.
- None of this starts PX4 for you: the two PX4 simulations are started by
  hand. This project's `launch` does start the Micro XRCE-DDS Agent and our
  nodes, but it needs the agent to be already built in
  `~/Micro-XRCE-DDS-Agent`.

That's the general idea. The rest of the wiki explains each piece in more
detail, with diagrams and the exact commands to install, run and debug.

## Suggested reading order

The documentation follows the system from end to end:

1. [Installation and build](Installation-and-build.md)
2. [Running the simulation](Simulation.md)
3. [Architecture and data flow](Architecture.md)
4. [Nodes, topics and diagnostics](Nodes-and-topics.md)
5. [Guidance modes](Guidance-modes.md)
6. [Code walkthrough](Code-walkthrough.md)
7. [Line by line: project and build files](Line-by-line-project-and-build-files.md)
8. [Line by line: odometry and tf2 nodes](Line-by-line-odometry-nodes.md)
9. [Line by line: guidance modes](Line-by-line-guidance-modes.md)
10. [Workspace map and dependencies](Workspace-file-map.md)
11. [Quick reference and troubleshooting](Quick-reference-and-troubleshooting.md)

## Short glossary

Sixteen words that come up all over the wiki. They don't replace the
explanations on each page, but they help if you land directly on a page in
the middle without reading the summary above:

| Term | Meaning in this project |
| --- | --- |
| Node | A ROS 2 program that does one specific job. |
| (Flight) mode | A guidance strategy registered in PX4 (`pursuit_mode`, `PN_mode`). It is not the same as a node, although each mode lives inside one. |
| Topic | A named channel over which messages are exchanged. |
| Message | A data structure that travels over a topic. |
| Publisher | What a node creates to **send** messages to a topic. |
| Subscription (subscriber) | What a node creates to **receive** messages from a topic; each incoming message triggers a callback. |
| Timer | Triggers a callback repeatedly at fixed intervals (for example, every 50 ms), without waiting for any message. |
| Logging | The mechanism for printing diagnostic messages (`RCLCPP_INFO`, `RCLCPP_WARN`...) to the console or the logs. |
| Callback | A function you don't call yourself: you hand it to ROS 2 (when subscribing to a topic, creating a timer...) and ROS 2 runs it automatically every time a message arrives or the timer fires. |
| DDS | The protocol that delivers messages between nodes without them knowing each other directly. PX4 uses a lightweight version (uXRCE-DDS) that the Micro XRCE-DDS Agent translates into the DDS spoken by ROS 2 nodes. |
| Frame | A named reference system (`map`, `interceptor/base_link`, `target/base_link`) that tf2 uses to place each vehicle. |
| tf2 | The system that relates positions and orientations between frames. |
| Odometry | Estimated position, orientation and velocity. |
| Setpoint | A motion reference handed to PX4. |
| PX4 | Open-source autopilot: the software that, inside the vehicle, estimates its position and drives the motors to follow a setpoint. More detail in [Installation and build](Installation-and-build.md). |
| SITL | PX4 running as software on the computer, without real hardware, with simulated sensors and physics. |

This is only a pocket glossary; terms are explained in more detail where
needed on each page. For commands and troubleshooting steps, use the
[quick reference and troubleshooting](Quick-reference-and-troubleshooting.md)
page, which is about terminal commands rather than concepts.

## Limits and safety

This wiki documents the current behaviour of the repository; it doesn't
replace the official ROS 2 or PX4 documentation, or control theory. The
project must be tested in simulation before any use with real hardware,
following the safety measures and limits configured in PX4.

---

➡️ Next: [Installation and build](Installation-and-build.md)
