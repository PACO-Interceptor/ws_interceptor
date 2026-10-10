# ws_interceptor

Project website: https://paco-interceptor.github.io

ROS 2 workspace for tracking and intercepting a vehicle using PX4 odometry data.

## Requirements

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- `colcon`
- `rosdep`
- To run the simulation: PX4 at commit `14b3f44081`, Micro XRCE-DDS
  Agent `v2.4.3` and QGroundControl `v5.1.4` (versions tested together; see
  [Installation and build](docs/wiki/Installation-and-build.md))

## Get the workspace

```bash
git clone https://github.com/PACO-Interceptor/ws_interceptor.git
cd ws_interceptor
```

The repository already includes the `interceptor` package and the PX4 dependencies it needs inside `src/`.

## Install dependencies and build

From the workspace folder (the one from the `cd` above; if you open a new
terminal, go back into it with `cd ~/ws_interceptor`):

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

To build only our package and its dependencies:

```bash
colcon build --packages-up-to interceptor --symlink-install
```

Every new terminal that will use `ros2` has to load the environment again,
from the workspace folder:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Run the simulation

You need four terminals, one per piece. The details, and what you should see
at each step, are in
[Running the simulation](docs/wiki/Simulation.md).

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

In QGroundControl: take off the target (vehicle 2), send it somewhere else with
*Go to location*, take off the interceptor (vehicle 1) and pick **PN mode** or
**Pursuit Intercept** in the flight mode selector.

The `mode` argument is optional and only accepts `pn` or `pursuit`: it chooses
which guidance mode is registered in PX4, since only one is launched. If left
out, it is `pn`. The old name, `modo:=`, still works for now as an alias of `mode`.
PX4 won't arm without QGroundControl connected. Single nodes can also be
started with `ros2 run interceptor <executable>`.

To stop: Ctrl-C in the launch first, which takes about 5 seconds on purpose,
then in the PX4 instances.

## Layout

- `src/interceptor`: this project's nodes and launch file.
- `src/px4_msgs`: PX4 messages.
- `src/px4_ros_com`: PX4-ROS 2 communication.
- `src/px4-ros2-interface-lib`: C++ library for the PX4-ROS 2 interface.

The `build/`, `install/` and `log/` folders are generated locally and are not part of the repository.

## Wiki

The [local introductory wiki](docs/wiki/Home.md) explains step by step how to
set up the workspace, run the simulation and understand what each node does.
It is the only source: the [GitHub wiki](https://github.com/PACO-Interceptor/ws_interceptor/wiki)
is generated from it and is never edited by hand.

```bash
python3 tools/publish_wiki.py --dry-run  # see what would change
python3 tools/publish_wiki.py            # publish
```

The script converts the links to the wiki format (pages without extension and
with their page title, paths to the code as absolute URLs) and pushes the
result. When adding a new page to `docs/wiki/`, give it a title in the
`PAGES` dictionary of that script.

## License

The `interceptor` package and the rest of the code in this repository are
released under the [Apache-2.0](LICENSE) licence.

The PX4 dependencies included in `src/` (`px4_msgs`, `px4_ros_com` and
`px4-ros2-interface-lib`) keep their original BSD-3-Clause licence, found in the
`LICENSE` file of each one.
