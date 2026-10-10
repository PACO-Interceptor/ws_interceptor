# Installation and build

This page has commands to type in a terminal. A terminal is a window where you
type instructions and the operating system answers with text. You don't need
to know Linux beforehand: copy a command, press Enter, read the output and
only carry on if there's no error.

A **workspace** is the folder that holds the source code, the dependencies and
the build results of several ROS 2 packages at once; in this project it is the
`ws_interceptor` folder that you download further down with `git clone`. You'll
see this word all over this page.

## Requirements

- **Ubuntu 24.04 (Noble)**: provides the compilers, Python and the tools that
  the ROS 2 packages of this setup expect.
- **ROS 2 Jazzy**: provides the runtime, the standard messages and `rclcpp`.
  "Jazzy" is the name of this version (or *distribution*) of ROS 2.
- **`colcon`**: coordinates building several packages that depend on each other.
- **`rosdep`**: turns ROS 2 dependencies into installable system packages.
- **PX4**: produces the odometry and receives the flight setpoints.
- **Micro XRCE-DDS Agent**, installed in `~/Micro-XRCE-DDS-Agent`: connects
  PX4's uXRCE-DDS transport with the DDS graph used by the ROS 2 nodes.
- **QGroundControl v5.1.4**: the ground station used to arm the drones, move
  the target and choose the interceptor's guidance mode.

Having the commands installed is not enough: they also have to be compatible
with each other. The PX4 section below says exactly which version to build so
that it matches the `px4_msgs` vendored in this repository.

### What each piece is, in more detail

A **runtime** is the set of processes and libraries that must be running for a
ROS 2 program to work while it runs, not just to build it. **`rclcpp`** is
the ROS 2 library for writing nodes in C++: it provides `Node`, `spin`,
publishers, subscribers and logging. It is explained in more detail in the
[Code walkthrough](Code-walkthrough.md).

**[PX4](https://px4.io/)** is an open-source autopilot for drones and other
uncrewed vehicles: the software that, inside the vehicle, reads the sensors,
estimates its position/velocity/orientation (the odometry) and drives the
motors to follow a flight reference (the setpoint). This project doesn't use
real hardware: each PX4 instance runs as **SITL** (*Software In The Loop*),
the same software running as a normal program on the computer, with simulated
sensors and physics instead of a physical drone.

**DDS** (*Data Distribution Service*) is the protocol ROS 2 uses to deliver
messages between nodes without them knowing each other directly. PX4 doesn't
speak full DDS (it is too heavy for a small flight board) but a lightweight
version called **[Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)**
("XRCE" stands for *eXtremely Resource Constrained Environments*). The
**Micro XRCE-DDS Agent** is the bridge program that translates those
lightweight PX4 messages into the full DDS spoken by the ROS 2 nodes
(`rclcpp`), so they can subscribe to them like any other topic.

### How to install each requirement if you don't have it yet

If you're missing any of them, here is the official installation guide:

- **Ubuntu 24.04**: follow the
  [official installation tutorial](https://ubuntu.com/desktop/docs/en/latest/tutorial/install-ubuntu-desktop/)
  (physical machine, virtual machine or WSL2 on Windows). Check the version you
  download: it has to be **24.04**, not the latest one if a newer release is
  out by then.
- **ROS 2 Jazzy**: follow the
  [official installation guide](https://docs.ros.org/en/jazzy/Installation.html)
  (official `apt` packages). The "Desktop" install (`ros-jazzy-desktop`)
  already includes `rclcpp` and the basic tools.
- **`colcon` and `rosdep`**: if you installed ROS 2 with `apt`, add them with:

  ```bash
  sudo apt install python3-colcon-common-extensions python3-rosdep
  sudo rosdep init  # only the first time on the system
  ```
- **QGroundControl v5.1.4**: download `QGroundControl-x86_64.AppImage` from the
  [v5.1.4 release](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.4)
  and follow the
  [official Linux installation guide](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html),
  which lists the system packages it needs. Use that exact version, not the
  "latest" or the *Daily* build: it's the one tested with the PX4 version on
  this page. One detail from that guide: it asks you to install `libfuse2`,
  but on Ubuntu 24.04 that package is called `libfuse2t64`
  (`sudo apt install libfuse2t64`). Without it, the AppImage doesn't open.

A **ground station** (*GCS*, *Ground Control Station*) is the program a person
uses to monitor the vehicle and send it commands: arm, take off, change mode,
mark a destination on the map. PX4 checks that one is connected before it lets
you arm; without QGroundControl open, arming is rejected with
`No connection to the GCS`.

PX4 and the Micro XRCE-DDS Agent aren't covered with a single link, because
the [official PX4 + ROS 2 guide, Jazzy section](https://docs.px4.io/main/en/ros2/user_guide#jazzy)
assumes things that don't apply here (an unpinned PX4 version, and the Agent
installed as a `colcon` package instead of a standalone binary). Instead of
pointing there and then correcting it, these are the commands this project
needs; the PX4 guide is still the source of the versions used below.

#### PX4

PX4 and the ROS 2 nodes only understand each other if they use **exactly the
same message definitions**. Many PX4 messages carry a version
(`MESSAGE_VERSION`) that is part of the topic name: for example, a version 4
`VehicleStatus` is published on `/fmu/out/vehicle_status_v4`. If PX4
publishes one version and `px4_ros2_cpp` expects another, the modes don't
find the topics and die at start-up with `Registration failed`.

The `px4_msgs` vendored in `src/px4_msgs` matches, message by message, PX4
`main` at commit **`14b3f44081`** (28 July 2026). Its `CHANGELOG.rst` says
"1.17.0", but it is **not** compatible with the `v1.17.0` release (there
`VehicleStatus` is at version 1), nor with current PX4 `main`. That's why
that exact commit has to be built. There's no need to create a ROS 2
workspace with `px4_msgs` (a step the PX4 guide does ask for in general): it's
already vendored here.

PX4-Autopilot doesn't have to be inside `ws_interceptor` or in any fixed
path; nothing in this project looks for it in a specific place. Clone it
wherever you keep your projects, for example your home folder:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout 14b3f44081
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

`git checkout` pins the commit (there's no named tag for it, which is why `-b`
isn't used when cloning). `git submodule update --init --recursive` downloads
the git submodules PX4 uses, at the versions of that commit. The
`Tools/setup/ubuntu.sh` script installs the build and simulation tools
(including Gazebo); the official PX4 guide recommends restarting the computer
when it finishes, before building. The last command builds and starts a test
run with the `gz_x500` model, the same one used in
[Simulation.md](Simulation.md); the first PX4 build takes quite a few minutes
and opens the Gazebo window when it finishes. Close it with Ctrl-C in that same
terminal before carrying on.

#### Micro XRCE-DDS Agent

`interceptor.launch.py` expects a standalone binary in a specific path
(`~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`), so it is built as a
standalone CMake project, not as a `colcon` package inside a ROS 2 workspace
(which is how the PX4 guide installs it in general):

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
```

`v2.4.3` is the version the PX4 guide recommends for Jazzy. That's all: there's
no need for `sudo make install`, because `interceptor.launch.py` runs
`build/MicroXRCEAgent` directly inside this same folder. If you install it
somewhere else, change `MICRO_XRCE_DDS_AGENT_DIR` in that file.

Ubuntu, ROS 2, `colcon`/`rosdep`, PX4, the Agent and QGroundControl are system
installs, independent of this repository: they're done once per machine, not
every time you build `interceptor`.

## Download the workspace

This one you can clone wherever you keep your projects (your home folder, for
example):

```bash
git clone https://github.com/PACO-Interceptor/ws_interceptor.git
cd ws_interceptor
```

From here on, every command on this page runs inside this folder
(`ws_interceptor`), unless stated otherwise.

## Install dependencies

The repository already includes inside `src/` (short for *source*) our package
and the vendored dependencies needed to build:

- `src/interceptor`
- `src/px4_msgs`
- `src/px4_ros_com`
- `src/px4-ros2-interface-lib`

That's why the next command points at `src` (`--from-paths src`): it tells
`rosdep` where to look for what to install.

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` installs system dependencies. The `--ignore-src` option avoids trying
to download again the packages that are already inside `src/`.

If `rosdep` reports an unresolved dependency, don't hide the error with `-r`:
read which package is missing, install it and run the command again. The `-y`
option only answers yes automatically to the installs rosdep confirms.

If the system asks for a password, it's your normal Ubuntu user password, not
a project password. If you see red text or a command ends with an error code,
stop and keep the full message to diagnose it.

## Build

To build the whole workspace:

```bash
colcon build --symlink-install
```

To build our package and the dependencies it needs:

```bash
colcon build --packages-up-to interceptor --symlink-install
```

After building, every terminal that will use ROS 2 has to load the workspace:

```bash
source install/setup.bash
```

`build/`, `install/` and `log/` are folders generated locally. They don't hold
source code and can be regenerated if the workspace is cleaned.

## What happens during `colcon build`

1. `colcon` finds each package's `package.xml` (the file that gives its name
   and what it depends on).
2. It builds a dependency graph: a map of which package needs which (for
   example, `interceptor` needs `px4_msgs`), to know the build order.
3. It configures each package with its build tool (`ament_cmake` in this
   project).
4. It builds the libraries and binaries: the compiled executables you later
   start with `ros2 run`, such as `pursuit_mode`.
5. It installs into `install/` those binaries, the headers (the `.h`/`.hpp`
   files that other `.cpp` files can pull in with `#include` without seeing how
   they're implemented), the contents of `launch/` (so `ros2 launch` can find
   them) and the package metadata (data about it, such as the `package.xml`
   itself, not code).
6. It keeps results and logs so failures can be diagnosed.

`--symlink-install` makes some development files available through symbolic
links. This avoids copies and makes iterating easier, but you still have to
rebuild when C++ code changes.

A **symbolic link** is a file that points to another file or folder. It isn't
a second copy; that's why it saves space and reflects changes to some files
faster.

## Check the installation

```bash
ros2 pkg list | grep interceptor
ros2 pkg executables interceptor
```

The second command should list the package executables, such as
`pursuit_mode`, `PN_mode` and the odometry nodes.

You can also check that ROS 2 finds the installed resources:

```bash
ros2 pkg prefix interceptor
ros2 interface show px4_msgs/msg/VehicleOdometry
```

The first command shows which install the terminal is using. The second shows
which fields the message has before you write a callback for it.

---

🏠 [Home](Home.md) · ⬅️ Previous: [Home](Home.md) · ➡️ Next: [Running the simulation](Simulation.md)
