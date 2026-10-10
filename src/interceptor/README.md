# interceptor

ROS 2 package (ament_cmake) for tracking and intercepting a target vehicle using PX4 odometry data.

## Requirements

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- To run the simulation: PX4 at commit `14b3f44081`, Micro XRCE-DDS Agent `v2.4.3` and
  QGroundControl `v5.1.4` (versions tested together; see the
  [installation guide](../../docs/wiki/Installation-and-build.md))

## Dependencies

This package is built inside the `ws_interceptor` workspace, which already includes in `src/` the three
PX4 packages it needs, vendored (no need to clone them separately):

- [px4_msgs](https://github.com/PX4/px4_msgs.git)
- [px4_ros_com](https://github.com/PX4/px4_ros_com.git)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) (provides `px4_ros2_cpp`)

Standard ROS 2 dependencies (installed with `rosdep`): `rclcpp`, `tf2_ros`, `tf2`, `geometry_msgs`, `sensor_msgs`, `launch`, `launch_ros`.

## Get the workspace

```bash
git clone https://github.com/PACO-Interceptor/ws_interceptor.git
cd ws_interceptor
```

## Install dependencies

From the workspace folder:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

To build only this package (and its dependencies):

```bash
colcon build --packages-up-to interceptor --symlink-install
```

## Run

The launch file doesn't start PX4: the two SITL instances are started by hand first, each in its own terminal
(see [Running the simulation](../../docs/wiki/Simulation.md)). With the environment loaded
(`source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` from the workspace folder):

```bash
ros2 launch interceptor interceptor.launch.py mode:=pn
```

The `mode` argument is optional (`pn` by default, or `pursuit`): only the chosen guidance mode is launched. `modo:=` is accepted for now as an alias.

Or run a single node, for example:

```bash
ros2 run interceptor pursuit_mode
```

## Questions

If you have any questions, ask the team or check the PX4/ROS 2 documentation before touching the code:

- [PX4 ROS 2 Interface](https://docs.px4.io/main/en/ros2/)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
