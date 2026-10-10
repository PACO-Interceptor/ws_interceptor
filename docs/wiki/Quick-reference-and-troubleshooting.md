# Quick reference and troubleshooting

This page is a checklist for when something doesn't work. An **error** doesn't
always mean the code is wrong: a process, a dependency, a setting or some data
may be missing. The approach is to check one layer at a time and not change
many things at once.

## Common commands

All the `ros2` commands on this page run in a terminal with the environment
loaded: from the workspace folder, `source /opt/ros/jazzy/setup.bash` and
`source install/setup.bash` (see
[Load the environment](Simulation.md#load-the-environment)). Since the launch
and the nodes take up their own terminals, you'll usually open a separate one
just for diagnostics.

```bash
# See active nodes and topics
ros2 node list
ros2 topic list

# See who publishes/subscribes on a node or a topic
ros2 node info /target_tf2_frame_publisher
ros2 topic info /target/velocity

# See the message type of a topic
ros2 topic type /target/velocity

# Measure the publishing rate
ros2 topic hz /target/velocity

# Inspect messages
ros2 topic echo /target/velocity
ros2 topic echo /px4_1/fmu/out/vehicle_odometry

# Run a node of the package
ros2 run interceptor target_tf2_odometry
```

`list` shows what exists, `info` shows who publishes or subscribes on a
specific node or topic, `type` shows the message type, `hz` counts how many
messages arrive per second, `echo` shows the messages going through a topic
and `run` starts an executable.

## Suggested diagnostic sequence

Run the checks in this order, since each step depends on the previous one:

```bash
# 1. Is ROS 2 up and which nodes exist?
ros2 node list

# 2. Is PX4 data arriving?
ros2 topic list
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic hz /px4_1/fmu/out/vehicle_odometry

# 3. Does the message have the expected structure?
ros2 topic type /px4_1/fmu/out/vehicle_odometry
ros2 interface show px4_msgs/msg/VehicleOdometry

# 4. Does the tf2 tree exist?
ros2 run tf2_ros tf2_echo map target/base_link
ros2 run tf2_ros tf2_echo map interceptor/base_link

# 5. Is the transformed velocity arriving?
ros2 topic hz /target/velocity
```

If a command fails, fix that level before carrying on. There's no point
debugging PN while the target's odometry topic is empty.

## Common problems

### `pursuit_mode` / `PN_mode` die with `Registration failed`

If the launch log shows, about 15 s after starting:

```text
[pursuit_mode]: timeout while waiting for FMU publisher discovery
terminate called after throwing an instance of 'px4_ros2::Exception'
  what():  Registration failed
```

or:

```text
[PN_mode]: Mismatch for the following topics, update PX4 or the px4_ros2 library and px4_msgs:
 - fmu/out/manual_control_setpoint
```

the PX4 version doesn't match the vendored `px4_msgs`: the modes look for
topics with a name or format that this PX4 doesn't publish (for example, they
expect `/fmu/out/vehicle_status_v4` and PX4 v1.17.0 publishes
`vehicle_status_v1`). Build PX4 at the commit given in
[Installation and build](Installation-and-build.md#px4). To see which version
your PX4 publishes:

```bash
ros2 topic list | grep vehicle_status
```

### The target publishes no odometry (missing sensors)

If the instance `1` terminal keeps printing `Preflight Fail: Accel Sensor 0
missing`, `barometer 0 missing` or `ekf2 missing data`, and
`ros2 topic hz /px4_1/fmu/out/vehicle_odometry` shows no rate even though the
topic exists, instance `1` was started without `GZ_IP=127.0.0.1`: Gazebo
creates the drone, but PX4 doesn't get its sensors. Stop that instance and
start it with the full command from
[Running the simulation](Simulation.md#2-start-the-targets-px4).

### `Arming denied` / `No connection to the GCS`

PX4 won't arm without a ground station connected. Open QGroundControl (see
[Running the simulation](Simulation.md#4-open-qgroundcontrol)); as soon as it
connects, the warning goes away.

### `map -> target/base_link` doesn't appear and the log mentions the global reference

If the `target_tf2_odometry` log keeps printing, at most every 5 seconds:

```text
Waiting for global reference (ref_lat/ref_lon/ref_alt) of interceptor and target
before publishing map -> target/base_link
```

the node is waiting for the estimator of one of the drones (or both; the
message says which one is missing) to fix its global position. Without that
reference it can't place the target in the interceptor's frame, so it would
rather publish nothing than publish a mixed-up position. It normally sorts
itself out a few seconds after PX4 starts. If not, check that the instance has
simulated GPS:

```bash
ros2 topic echo --once /fmu/out/vehicle_local_position_v1 --field xy_global
```

It should say `true`. Until then, the guidance modes won't arm, because they
get no target data.

### The mode reports the target reached without moving

If, when activating **PN mode** or **Pursuit Intercept**, the log shows
`Target reached. Stopping pursuit.` straight away and the interceptor doesn't
move, even though the target is far away in Gazebo, the two drones are
measuring from different origins. `target_tf2_odometry` corrects this by
itself, so first check that the node is alive and publishing:

```bash
ros2 run tf2_ros tf2_echo map target/base_link
```

The distance it shows should be close to the real separation in Gazebo. As a
manual fix, both PX4s can be given the same origin: read `ref_lat`, `ref_lon`
and `ref_alt` in the interceptor's `pxh>` console with
`listener vehicle_local_position` and apply them in the target's with
`commander set_ekf_origin <ref_lat> <ref_lon> <ref_alt>`. If it answers
`commander not running`, wait a few seconds and try again.

### `VehicleOdometry` doesn't appear

Check that both PX4 instances are running, that the agent is up and that each
terminal has ROS 2 loaded. Instance 1 must publish with the `/px4_1/` prefix.

Tell apart "the topic doesn't exist" from "it exists but nothing is
published". `ros2 topic list` checks the first and `ros2 topic hz` the second.
If it exists but has no rate, check PX4, the agent and the UDP transport.

### The target doesn't appear in tf2

Check that `target_tf2_odometry` is running and receiving
`/px4_1/fmu/out/vehicle_odometry`. If the topic exists but the frame doesn't
appear, look at the node's output and the `vehicle_name` parameter.

Use `tf2_echo` to find which frame is missing. If `map` shows up but
`target/base_link` doesn't, the problem is in `target_tf2_odometry` or its
input, not in PN.

### PN won't arm

PN needs two things: `map -> target/base_link` and `target/velocity`. Start
`target_tf2_odometry` and check the topic:

```bash
ros2 topic echo /target/velocity
```

Also check that the mode gets the position:

```bash
ros2 run tf2_ros tf2_echo map target/base_link
```

PN needs both sources; only one of them working is not enough.

### The launch can't find the Micro XRCE-DDS Agent

The launch file expects the agent at:

```text
~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
```

If it's installed somewhere else, change `MICRO_XRCE_DDS_AGENT_DIR` in
`interceptor.launch.py`.

### C++ changes don't show up

Build again and reload the workspace:

```bash
colcon build --packages-up-to interceptor --symlink-install
source install/setup.bash
```

If an old version still shows up, check:

```bash
ros2 pkg prefix interceptor
which ros2
```

The terminal may be using another overlaid workspace. Each `source` changes
the environment of that terminal, not the others.

### tf2 warnings appear

The modes look up transforms periodically. Getting warnings at the start,
before the converter has received the first odometry, is normal. Continuous
warnings mean a node, a topic or a frame name is missing.

Don't hide the warning by increasing the interval without understanding it.
The names `map`, `target/base_link` and `interceptor/base_link` must match
exactly, including case, slashes and namespace.

## A safe order for changing code

1. Work out whether the change affects a diagnostic node, tf2 or the guidance.
2. Review the topics and frames the node already uses.
3. Keep the NED/ENU conversions in one place and document any change of
   convention.
4. Build with `colcon build --packages-up-to interceptor --symlink-install`.
5. Run the package tests/lint:

```bash
colcon test --packages-select interceptor --event-handlers console_direct+
colcon test-result --verbose
```

Before opening a pull request, also review the diff:

```bash
git status
git diff --check
git diff -- README.md docs/
```

A good contribution explains what changed, why, how it was tested and which
limitations remain.

## Official references

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [PX4 (official site)](https://px4.io/)
- [PX4 ROS 2 guide](https://docs.px4.io/main/en/ros2/)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Micro XRCE-DDS (protocol documentation)](https://micro-xrce-dds.docs.eprosima.com/en/latest/)
- [Micro-XRCE-DDS-Agent (repository)](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

---

🏠 [Home](Home.md) · ⬅️ Previous: [Workspace map and dependencies](Workspace-file-map.md)
