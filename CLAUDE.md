# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (Jazzy) workspace for guiding one PX4 drone ("interceptor") to track and intercept another
("target") using PX4 vehicle odometry, tf2 frames, and PX4's offboard-mode ROS 2 API. Only
`src/interceptor` is this project's own code (Spanish comments/docs, English code identifiers);
`src/px4_msgs`, `src/px4_ros_com`, and `src/px4-ros2-interface-lib` are vendored upstream
dependencies (PX4, Auterion) checked into `src/` rather than pulled via rosdep from git — don't
"fix" or refactor code inside those three unless the task is explicitly about updating a vendored
dep.

**The user writes interceptor code themselves.** Default to explaining/orienting rather than
auto-editing `src/interceptor` unless asked to make the change.

## Build / test / run

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y      # install deps first time / after adding deps

colcon build --packages-up-to interceptor --symlink-install   # build (only what's needed)
source install/setup.bash

colcon test --packages-select interceptor --event-handlers console_direct+
colcon test-result --verbose
```

Tests are `ament_lint_auto`/`ament_lint_common` only (uncrustify, cppcheck, lint_cmake and xmllint
for C++/CMake/XML, flake8 + pep257 for the Python launch file; `copyright` and `cpplint` are
explicitly skipped in `CMakeLists.txt`) — there are no unit tests, and no custom lint config files,
so linting follows ROS 2 ament defaults. CI (`.github/workflows` at the repo root) runs this same
build+test sequence on `ros:jazzy-ros-base` for pushes/PRs to `main`/`master`; a release workflow
tags and publishes GitHub releases after CI succeeds on `main`.

Run everything:
```bash
ros2 launch interceptor interceptor.launch.py modo:=pn
```
`modo` is an optional launch argument, `pn` (the default) or `pursuit`: both mode nodes are in the
`LaunchDescription`, each behind an `IfCondition`, so only the chosen one is started. Or a single
node:
`ros2 run interceptor <executable>` (executable names = source file names, e.g.
`pursuit_mode`, `PN_mode`).

The launch file starts the Micro XRCE-DDS Agent (expected at `~/Micro-XRCE-DDS-Agent`), the
interceptor package's odometry/diagnostic nodes, and the one guidance mode picked with `modo`. It
does **not** start PX4 SITL itself — that's run manually from
`~/PX4-Autopilot`, checked out at commit `14b3f44081` (the one the vendored `px4_msgs` matches;
see `docs/wiki/Installation-and-build.md`):
- drone 0 (interceptor): `make px4_sitl gz_x500`
- drone 1 (target): `GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1`

Without `GZ_IP` the target gets no sensor data; without `PX4_GZ_MODEL_POSE` it spawns inside the
interceptor. Arming requires QGroundControl v5.1.4 connected (see `docs/wiki/Simulation.md`).

## Architecture

Two PX4 SITL instances publish `VehicleOdometry` on uXRCE-DDS: instance 0 (no topic prefix) is the
**interceptor**, instance 1 (`/px4_1/...` prefix) is the **target**. Everything downstream is wired
through **tf2**, not direct topic-to-topic subscriptions — this is the key thing to understand
before touching any node:

1. **`target_tf2_odometry`** / **`interceptor_tf2_odometry`** (`FramePublisher`) — each subscribes
   to its drone's `.../fmu/out/vehicle_odometry`, converts PX4's NED/aircraft frame to ROS's
   ENU/base_link frame (`px4_ros_com::frame_transforms`), and broadcasts a tf2 transform
   `map -> {target,interceptor}/base_link`. The target's node additionally republishes its ENU
   linear velocity on `target/velocity` (`TwistStamped`, 10 Hz) since tf2 transforms carry no
   velocity. Because each PX4 instance measures its local position from its own startup origin,
   `target_tf2_odometry` also subscribes to `vehicle_local_position_v1` of both instances for
   `ref_lat`/`ref_lon`/`ref_alt` (the `_v1` suffix comes from the message's `MESSAGE_VERSION` and
   is appended at runtime by `px4_ros2::getMessageNameVersion`), and adds the NED offset between
   the target's and the interceptor's origin to the target's position before publishing, so `map`
   consistently means "the interceptor's origin" for both vehicles; until both references are
   valid it withholds the transform and logs a throttled warning instead.
2. **`vehicle_odometry_subscriber`** — a single parametrized debug listener (`vehicle_name`,
   `odometry_topic`) that dumps raw `VehicleOdometry` fields to stdout; the launch file starts it
   twice, once per vehicle, as `target_vehicle_odometry_subscriber` and
   `interceptor_vehicle_odometry_subscriber`. Not part of the control loop.
3. **`tf2_listener`** — standalone debug node, logs the `interceptor -> target` tf2 transform plus
   the target's last known NED velocity once a second. Commented out of the default launch file.
4. **`pursuit_mode`** / **`PN_mode`** — the actual guidance logic, each a
   `px4_ros2::ModeBase` subclass wrapped in `px4_ros2::NodeWithMode<T>` (registers as a custom PX4
   flight mode via `px4_ros2_cpp`). Both:
   - look up the `map -> target/base_link` tf2 transform on a 50 ms timer to get target position
     (converted back to NED for control math),
   - read own position via `px4_ros2::OdometryLocalPosition`,
   - gate arming/running on `checkArmingAndRunConditions` until target data is valid,
   - compute a velocity/acceleration setpoint from the line-of-sight (LOS) vector each
     `updateSetpoint(dt_s)` tick, sent through a shared `TrajectorySetpointType`,
   - call `completed(px4_ros2::Result::Success)` once per approach, the first `updateSetpoint` tick
     LOS norm drops below 1 m (an internal `_target_reached` flag, reset in `onActivate()` and
     whenever LOS norm climbs back above 1 m, keeps this from re-firing every tick while it stays
     under 1 m).
   `PursuitMode` does straight pure-pursuit (velocity toward target, capped horizontal/vertical
   speed). `PN_Mode` additionally subscribes to `target/velocity` (converting ENU->NED) to run true
   Proportional Navigation (LOS rotation rate × relative velocity, scaled by a navigation constant,
   capped acceleration below `kPnMinRange`), so it depends on `target_tf2_odometry` running to
   supply that velocity topic — `pursuit_mode` does not.

Frame/topic naming convention to preserve when adding nodes: vehicle frames are always
`{vehicle_name}/base_link` under the common `map` frame; PX4 topics for a given instance are always
under `[/px4_<N>]/fmu/out/...` (empty prefix = instance 0).

Guidance tuning constants (speed caps, navigation constant, min ranges) are `static constexpr`
inside each mode class — no external param files or dynamic reconfiguration.
</content>
