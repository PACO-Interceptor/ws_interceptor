# Running the simulation

This page describes how to get several pieces running at the same time. When
it says "another terminal", it means another command window; each one keeps
its own process running.

The project's launch file connects ROS 2 to two PX4 SITL instances. PX4 SITL
simulates the autopilot and the vehicle; the Micro XRCE-DDS Agent is the
bridge between PX4 and ROS 2.

## Before you start

This page assumes you have already followed
[Installation and build](Installation-and-build.md) and that you have:

- the workspace **built** (`colcon build --symlink-install` without errors);
- **PX4** cloned in `~/PX4-Autopilot`, pinned to the commit given on that
  page and built at least once (if you cloned it somewhere else, change that
  path in the commands of steps 1 and 2);
- the **Micro XRCE-DDS Agent** built in `~/Micro-XRCE-DDS-Agent`;
- **QGroundControl v5.1.4** downloaded.

You'll need four terminals: one for each piece that keeps running, plus a
spare one for checking things.

<a id="load-the-environment"></a>
**Load the environment.** Every new terminal starts knowing nothing about ROS 2
or this workspace, and what you load in one doesn't carry over to the others.
Before using any `ros2` command, in that terminal:

```bash
cd ~/ws_interceptor  # wherever you cloned the workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The first line loads ROS 2 and the second, this workspace (that's why you have
to be inside its folder: `install/setup.bash` is a relative path). The PX4
terminals don't need it: PX4 is not a ROS 2 program.

## Quick summary

The commands, in order, to get everything running. Each block goes in its own
terminal and is explained in detail below.

```bash
# 1. Interceptor (instance 0). Also opens the Gazebo window
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# 2. Target (instance 1), 20 m to the north
cd ~/PX4-Autopilot
GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1

# 3. Project nodes, with the chosen guidance mode
cd ~/ws_interceptor  # wherever you cloned the workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interceptor interceptor.launch.py mode:=pn

# 4. Ground station
~/QGroundControl-x86_64.AppImage
```

Then, from QGroundControl: take off both drones, send the target somewhere
else and select **PN mode** on the interceptor (see [Fly](#fly)).

## What is being simulated

SITL is not just a window with a 3D model. It runs the PX4 software and its
estimators as normal processes, so ROS 2 receives messages very similar to
those of a real vehicle. This lets you test the integration without risking
hardware, although it doesn't reproduce every physical condition.

The instances must have different identities:

- index `0`: interceptor, topics without `/px4_1/`;
- index `1`: target, topics with `/px4_1/`.

`/px4_1/` is a prefix PX4 adds automatically to the topics of an instance
started with `-i` greater than 0 (here, `-i 1`), to tell them apart from those
of instance `0`, which has no prefix. It's needed here because interceptor and
target are two PX4s running at the same time on the same computer: without
the prefix, both would publish on the same topic names and there'd be no way
of telling which vehicle each message comes from. So the index is not a
display name but a real part of the addressing: if the indices are swapped at
start-up, the code still builds the same, but each node ends up consuming the
wrong vehicle's data.

## Recommended order

Open separate terminals and load ROS 2 and the workspace where needed.

```mermaid
%%{init: {"theme": "dark"}}%%
sequenceDiagram
    participant T1 as Terminal 1 (PX4 interceptor)
    participant T2 as Terminal 2 (PX4 target)
    participant T3 as Terminal 3 (project launch)

    T1->>T1: make px4_sitl gz_x500 (instance 0)
    Note over T1: wait until start-up finishes
    T2->>T2: GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" ... px4 -i 1 (instance 1)
    Note over T2: wait until start-up finishes
    T3->>T3: ros2 launch interceptor interceptor.launch.py mode:=pn
    T3->>T3: starts Micro XRCE-DDS Agent (udp4, port 8888)
    T3->>T3: starts tf2 converters, diagnostics and the chosen flight mode
    T1-->>T3: VehicleOdometry (instance 0)
    T2-->>T3: VehicleOdometry (instance 1)
```

Order matters: if the launch starts before the two PX4 instances, its nodes
simply don't receive odometry yet and wait; if the DDS agent isn't running,
neither PX4 instance reaches ROS 2 even though they're "up".

### 1. Start the interceptor's PX4

In the PX4 folder:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This is instance `0` (the interceptor, see above).

Wait for PX4 to finish starting before you carry on. In a real test it's worth
checking that the process is stable and that the vehicle shows up in the
simulator.

### 2. Start the target's PX4

In another terminal:

```bash
cd ~/PX4-Autopilot
GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

This is instance `1` (the target, see above). `make` isn't used here: the
simulator was already started by instance `0`, so the PX4 binary built in the
previous step is run directly. Every part of the command matters:

- `./build/...`: the path is relative to `~/PX4-Autopilot` (it starts with
  `./`). With `/build/...` the system would look for it from the root of the
  disk and not find it.
- `GZ_IP=127.0.0.1`: the address PX4 uses to talk to Gazebo. `make` sets it by
  itself for instance `0`; if instance `1` doesn't use the same one, Gazebo
  creates the drone but PX4 doesn't get its sensors (`Accel Sensor 0 missing`,
  `ekf2 missing data`) and publishes no odometry.
- `PX4_GZ_MODEL_POSE="0,20"`: the target's starting position in Gazebo, in
  metres (`x,y`): here, 20 m north of the interceptor. Without it, the target
  appears at the same point as the interceptor, one inside the other.
- `PX4_SIM_MODEL=gz_x500`: the same drone model as instance `0`.
- `-i 1`: the instance number, which gives its topics the `/px4_1/` prefix.

### 3. Start the project launch

In another terminal, **inside the workspace folder** (the two previous steps
left you in the PX4 folder, and a new terminal starts in your home folder).
That's what [Load the environment](#load-the-environment) explains:

```bash
cd ~/ws_interceptor  # wherever you cloned the workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interceptor interceptor.launch.py mode:=pn
```

If you run those `source` commands from another folder, you'll see
`bash: install/setup.bash: No such file or directory` and then
`Package 'interceptor' not found`: same cause, you're not in the workspace.

The `mode` argument is optional and only accepts `pn` or `pursuit`; if left
out, it is `pn` (the old name, `modo:=`, is accepted for now as an alias). The
launch starts the agent with `udp4` on port `8888`, the odometry converters,
the diagnostic nodes and the single flight mode node chosen with `mode`.
`udp4` means UDP over IPv4: a way of sending packets over the network without
keeping a permanent connection; here it's used inside the computer itself, so
the agent receives the data PX4 sends it.

The launch doesn't start PX4 SITL. The two previous commands are required if
you want to test with simulation.

### 4. Open QGroundControl

In another terminal, run the AppImage you downloaded in
[Installation and build](Installation-and-build.md), with the path where you
saved it:

```bash
~/QGroundControl-x86_64.AppImage
```

You can also open it with a double click from the file manager, if you made it
executable. The first time it asks for the vehicle type and the units: choose
*PX4 Pro*, *Multi-Rotor* and *Metric System*, so distances match this wiki.

It connects by itself to both instances over UDP (port `14550`), with nothing
to configure: the interceptor shows up as vehicle `1` and the target as
vehicle `2`. While QGroundControl is not open, PX4 won't arm
(`Preflight Fail: No connection to the GCS`).

## How to tell if start-up worked

In a new terminal, with the environment loaded (see
[Load the environment](#load-the-environment)):

```bash
ros2 node list
ros2 topic list | grep -E 'vehicle_odometry|target/velocity'
ros2 topic hz /target/velocity
```

You should see the package nodes, the odometry topics of both instances and a
rate of about 10 Hz for `target/velocity`. The exact rate may vary; what
matters at first is that there is data and that it changes.

## Test the guidance

There's nothing else to prepare. Each PX4 measures its position from the point
where it started, so interceptor and target use different origins, but
`target_tf2_odometry` corrects this by itself: it places the target in the
interceptor's frame before publishing it (see
[Architecture and data flow](Architecture.md)). You can check it in a new
terminal, with the environment loaded (see
[Load the environment](#load-the-environment)):

```bash
ros2 run tf2_ros tf2_echo map target/base_link
```

With the target started 20 m to the north, you should get `y` ≈ 20, not `0`.

### Fly

In QGroundControl, picking each vehicle in the vehicle selector of the top bar:

1. **Target (vehicle 2)**: take off (*Takeoff*). Once it's in the air, click
   on the map and use *Go to location* to send it somewhere else; that way the
   chase is against a moving target.
2. **Interceptor (vehicle 1)**: take off (*Takeoff*) and, once in the air,
   select **PN mode** or **Pursuit Intercept** in the flight mode selector.

The interceptor goes after the target. The mode can only be selected once it
receives target data; otherwise PX4 rejects it with
`No target odometry received yet` (see
[Troubleshooting](Quick-reference-and-troubleshooting.md)). When it reaches it
(within 1 m) the log shows `Target reached. Stopping pursuit.` once per
approach, but the interceptor stays in the mode: if the target moves away, it
chases it again and the message will appear again on the next approach. To
finish, switch the interceptor to *Hold* or *Land*.

## Run a single node

Each node goes in its own terminal, with the environment loaded (see
[Load the environment](#load-the-environment)). It's useful to test one piece
without bringing up the whole launch; PX4 still has to be running.

```bash
ros2 run interceptor target_tf2_odometry
ros2 run interceptor interceptor_tf2_odometry
ros2 run interceptor tf2_listener
```

The odometry diagnostic node is a single executable for both drones, so when
running it by hand you have to tell it which one to listen to:

```bash
ros2 run interceptor vehicle_odometry_subscriber --ros-args \
  -p vehicle_name:=target -p odometry_topic:=/px4_1/fmu/out/vehicle_odometry
```

Without those parameters it uses the defaults, which are the interceptor's.

Don't run two copies of the same node without a clear reason: they could
publish the same transform or use duplicate resources. And don't start a
guidance mode (`pursuit_mode` or `PN_mode`) while the launch is running
another: both would try to register in PX4 and you may end up with a duplicate
registration that doesn't respond (see "A note on the modes" below).

## Stop the simulation

Stop the launch first and then the PX4 instances, each with Ctrl-C in its
terminal. If you leave old processes running, the next test may receive
duplicate messages or fail to open ports.

When you press Ctrl-C, the launch takes about **5 seconds** to close. That's on
purpose: the flight mode needs that time to unregister from PX4 before the
agent, which carries that message, shuts down. The agent ignores the first
Ctrl-C and closes when the launch insists, five seconds later. If it closed at
the same time as everything else, PX4 would be left with a registered mode
that no longer exists, and on the next launch you'd get warnings about
unresponsive modes.

The Gazebo window was started by instance `0`, so it closes when you stop that
terminal; if it stays open, close it yourself. QGroundControl is a separate
program: close it from its window when you no longer need it. To check that
nothing is left hanging:

```bash
pgrep -a -f 'px4|gz sim|MicroXRCEAgent'
```

If something still shows up after closing every terminal, close it with
`pkill -f` and the name shown before starting again.

## A note on the modes

The launch registers **a single** guidance mode in PX4, the one you choose
with `mode:=pn` or `mode:=pursuit`. Both register through `px4_ros2_cpp`,
but one at a time: if both try to register at once, PX4 can end up with a
duplicate registration that doesn't respond and that mode can no longer be
activated. To try the other one, stop the launch and start it again with the
other value. To understand their differences and the data they need, see
[Guidance modes](Guidance-modes.md).

---

🏠 [Home](Home.md) · ⬅️ Previous: [Installation and build](Installation-and-build.md) · ➡️ Next: [Architecture and data flow](Architecture.md)
