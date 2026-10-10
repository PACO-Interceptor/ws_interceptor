# Line by line: project and build files

This is the first of three pages that cover **every file of our own in
`src/interceptor`**. This page deals with the package's build, configuration
and start-up files. The other two are:

- [Odometry and tf2 nodes, line by line](Line-by-line-odometry-nodes.md): the
  diagnostic subscribers, the two tf2 converters and `tf2_listener`.
- [Guidance modes, line by line](Line-by-line-guidance-modes.md):
  `pursuit_mode.cpp` and `PN_mode.cpp`.

Most files on this page are analysed line by line because they are code or
configuration that the machine runs (`.gitignore`, `package.xml`,
`CMakeLists.txt`, the launch file). `LICENSE` and `README.md` are the
exception: they're text for people, not instructions that ROS 2 or CMake
interpret, so their sections only explain what they are and what they're for.

Third-party libraries are not analysed line by line: those folders are covered
in the [workspace map](Workspace-file-map.md), but their code belongs to their
upstream projects. The `.github` workflows (CI, issue summaries, releases) are
also explained in the [workspace map](Workspace-file-map.md), together with
the rest of the repository automation.

## How to read a technical explanation

A line of code can look like a strange sentence because it mixes words,
symbols and types. Read it in this order:

1. Find the main action, usually the name before `(`.
2. Look at the values in brackets: they're the data that action receives.
3. Check the type before the name: it says what kind of data is stored or
   returned.
4. Look for `=` to tell "create or compute something" from "use something that
   already exists".
5. `->` or `.` mean "access something that belongs to an object".

For example, `msg->velocity[0]` reads as: "from the message called `msg`, go
into the `velocity` field and take its first position". Indices start at zero
in C++, so `[0]`, `[1]` and `[2]` are the three components.

You don't need to master every symbol at once. The explanation under each
block says what data goes in, what transformation happens and what comes out.

## C++ syntax you'll see all the time

The [Code walkthrough](Code-walkthrough.md) already explains the basics (`{`,
`;`, `//`, classes, functions, variables, lambdas,
`shared_ptr`/`unique_ptr`). The tables on these pages also use more specific
C++ constructs that aren't explained anywhere else in the wiki. If you find one
and don't know what it means, come back to this table:

| Syntax | What it means | Example in this package |
| --- | --- | --- |
| `override` | Written after a method, it means "this replaces a method that already existed in the class I inherit from". The compiler warns if the name or the parameters don't match the original exactly. | `void updateSetpoint(float dt_s) override` replaces the empty `updateSetpoint` declared by `px4_ros2::ModeBase`. |
| `explicit` | In front of a constructor, it stops C++ from using it "behind your back" to turn a value into an object of that class without the programmer asking for it. It's a safety measure; it doesn't change what the constructor does. | `explicit PursuitMode(rclcpp::Node & node)`. |
| `static constexpr` | A constant value computed at compile time (before the program runs) and shared by every instance of the class; it never changes while the program runs. | `static constexpr float kMaxHorizontalSpeed = 5.0f;`. |
| `static` outside a class | A file-level `static` variable or function is only visible inside that `.cpp`; no other file can use it, even with the same name. | `static const std::string kName = "Pursuit Intercept";`. |
| `Template<Type>` (for example `shared_ptr<Buffer>`) | The `< >` after a class name mean a template: a generic mould filled in with a specific type. `shared_ptr<Buffer>` is "a shared pointer, specifically to a `Buffer` object", not to anything. | `std::unique_ptr<tf2_ros::Buffer> _tf_buffer;`. |
| `Type & name` (reference) | Receives the original object, not a copy. If the function changes that parameter, the change is visible outside the function too. | `explicit PursuitMode(rclcpp::Node & node)`: the mode gets the real node, not a copy. |
| `const Type & name` | Like the previous one, but it also promises the function won't change that object: it only reads it. | `catch (const tf2::TransformException & ex)`. |
| `const` in front of a variable | The value can't be changed after it's created. | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();`. |
| `Space::Name` (`::`) | Read it as "inside". `std::string` is `string` inside the `std` namespace; `px4_ros2::Result::Success` is the value `Success`, inside `Result`, inside `px4_ros2`. | `px4_ros2::Result::Success`, `std::chrono::milliseconds(100)`. |
| `.cross(v)` | Cross product of two 3D vectors: the result is another vector, perpendicular to both, related to how much and in which direction one "turns" relative to the other. | `los.cross(v_rel)` in `PN_mode.cpp`. |
| `.norm()` | The length (magnitude) of a vector. | `los.norm() < 1.0f`. |
| `.squaredNorm()` | The squared length. Used instead of `.norm()` when the exact square root isn't needed, because it's faster to compute. | `los.squaredNorm() + 1e-6f`. |
| `.normalized()` | Returns the same vector with length 1, keeping its direction. It keeps only "where it points", without the size. | `los_horizontal.normalized() * kMaxHorizontalSpeed`. |
| `.cast<float>()` | Converts the numbers of a vector/quaternion from one type to another (here, from `double` to `float`). The `f` at the end of `Eigen::Vector3f` means its three components are `float`; the `d` in `Eigen::Vector3d`, that they're `double`. Frame conversion functions usually work in `double`, while the modes keep their internal state in `float`, which is why this `.cast<float>()` shows up so often. | `position_ned.cast<float>()`. |
| `std::bind(&Class::method, this)` | Packs a method of a specific object so it can be passed around as if it were a free function (for example, to a timer). | `std::bind(&FrameListener::on_timer, this)` in `tf2_listener.cpp`. |
| `std::clamp(value, min, max)` | If `value` is below `min`, returns `min`; if it's above `max`, returns `max`; otherwise returns `value` as it is. It limits a number to a range. | `std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed)`. |
| `std::min(a, b)` | Returns the smaller of the two values. | `std::min(a_cmd_norm, kMaxAcceleration)`. |
| `(void)name;` | Tells the compiler "I know I don't use this variable, it's on purpose", so it doesn't warn about an unused parameter. | `(void)dt_s;` in `updateSetpoint`. |

## How to use this analysis

- Line numbers match the current state of each file.
- Blank lines separate blocks and don't do anything.
- When several lines make up a single CMake, XML, Python or C++ instruction,
  they're explained together, saying what each one does.
- Comments don't change the program; they explain intent to the reader.
- To check the exact text, open the link to the source file in each section.

## Suggested order for these three pages

The sections of this page and the other two already follow this order; there's
no need to jump around, just read the three pages top to bottom, in this order:

1. **On this page**: `.gitignore`, `LICENSE` and `README.md`, to get to know the
   package layout; then `package.xml` and `CMakeLists.txt`, to link each
   executable to its `.cpp`; and finally `interceptor.launch.py`, to see what
   starts and in which order.
2. **In [odometry and tf2 nodes](Line-by-line-odometry-nodes.md)**: first the
   diagnostic subscriber (the simplest callbacks in the package), then the tf2
   converters and `tf2_listener`.
3. **In [guidance modes](Line-by-line-guidance-modes.md)**: `pursuit_mode.cpp`
   before `PN_mode.cpp` (Pursuit is the base PN builds on).

After the three pages, use the [workspace map](Workspace-file-map.md) to tell
our code apart from the vendored dependencies and to see the `.github`
workflows.

## 1. `src/interceptor/.gitignore`

File: [`src/interceptor/.gitignore`](../../src/interceptor/.gitignore)

Each pattern tells Git which local files it must not include:

| Line | Explanation |
| --- | --- |
| `build/` | Ignores CMake/colcon build artefacts. |
| `install/` | Ignores the install generated by colcon. |
| `log/` | Ignores logs generated by ROS 2. |
| `*.o` | Ignores compiled C/C++ objects. |
| `*.so` | Ignores compiled shared libraries. |
| `__pycache__/` | Ignores bytecode generated by Python. |

The file keeps reproducible build results out of the repository. It doesn't
stop source files from being committed and doesn't change how the package
behaves.

## 2. `src/interceptor/LICENSE`

File: [`src/interceptor/LICENSE`](../../src/interceptor/LICENSE)

The package's legal text, not executable code: the full text of the
[Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0), the same
licence as the rest of the repository (root `LICENSE`). `package.xml` declares
the same licence, separately, in its own `<license>` tag (section 4, below).

## 3. `src/interceptor/README.md`

File: [`src/interceptor/README.md`](../../src/interceptor/README.md)

The entry point to the package for someone cloning it for the first time. It
sums up what it is (a ROS 2 `ament_cmake` package to track and intercept a
target using PX4 odometry data), which operating system and versions it needs
(Ubuntu 24.04, ROS 2 Jazzy and, to simulate, the pinned versions of PX4, the
Agent and QGroundControl), and which three external packages it uses
(`px4_msgs`, `px4_ros_com`, `px4-ros2-interface-lib`, with links to their
official repositories), making clear they're already vendored inside `src/`
and don't need to be cloned separately. From there it gives the commands to
clone the workspace, install dependencies, build and run the package; those
same commands are explained in detail, in order and with context, in
[Installation and build](Installation-and-build.md) and
[Running the simulation](Simulation.md), so they aren't repeated one by one
here. It ends with links to the official ROS 2 and PX4 documentation for
anyone who wants to dig deeper without mixing it up with this code.

## 4. `src/interceptor/package.xml`

File: [`src/interceptor/package.xml`](../../src/interceptor/package.xml)

| Line | Explanation |
| --- | --- |
| `<?xml version="1.0"?>` | Declares that the document uses XML 1.0. |
| `<?xml-model ... package_format3.xsd ...?>` | Lets the document be validated against the ROS 2 package schema. |
| `<package format="3">` | Opens a format 3 package. |
| `<name>interceptor</name>` | Name used by `ros2 run`, `ros2 launch` and colcon. |
| `<version>0.1.1</version>` | Declared package version. |
| `<description>...</description>` | Short description for ROS 2 tools. |
| `<maintainer email=...>Deireb</maintainer>` | Maintainer, with their GitHub no-reply address. |
| `<license>Apache-2.0</license>` | Name of the declared licence. |
| `<url type="website">...</url>` | Maintainer's web page. |
| `<buildtool_depend>ament_cmake</buildtool_depend>` | Says that CMake/ament builds the package. |

The `<depend>` tags for `rclcpp`, `tf2_ros`, `tf2`, `geometry_msgs`,
`px4_msgs`, `px4_ros_com` and `px4_ros2_cpp` declare dependencies that the code
really uses to build and run. Each name must match a ROS 2 package that
`find_package` can find. `sensor_msgs` is different: it's declared here and in
`CMakeLists.txt`, but no `.cpp` in the package includes it or uses any of its
types; it's a declared dependency that the current source doesn't use.

`<exec_depend>launch</exec_depend>` and `<exec_depend>launch_ros</exec_depend>`
are needed to run the Python launch file. The two `<test_depend>` tags are
only used for lint/tests. `<export>` and `<build_type>ament_cmake</build_type>`
tell ROS 2 how to build the package. `</package>` closes the document.

## 5. `src/interceptor/CMakeLists.txt`

File: [`src/interceptor/CMakeLists.txt`](../../src/interceptor/CMakeLists.txt)

### Project and compiler, lines 1-6

| Code | Explanation |
| --- | --- |
| `cmake_minimum_required(VERSION 3.8)` | Rejects CMake versions that are too old. |
| `project(interceptor)` | Sets the CMake project name. |
| `if(CMAKE_COMPILER_IS_GNUCXX OR ... Clang)` | Enters if the compiler is GCC or Clang. |
| `add_compile_options(-Wall -Wextra -Wpedantic)` | Turns on warnings about errors and non-portable constructs. |
| `endif()` | Closes the condition. |

### Dependencies, lines 8-17

Each `find_package(NAME REQUIRED)` looks for a package and makes the
configuration fail if it doesn't exist:

- `ament_cmake`: CMake/ROS 2 integration.
- `rclcpp`: the ROS 2 C++ API.
- `tf2_ros` and `tf2`: transforms and exceptions.
- `geometry_msgs`: transforms, twists and geometric messages.
- `px4_msgs`: `VehicleOdometry`.
- `px4_ros_com`: frame conversions.
- `px4_ros2_cpp`: PX4 modes and setpoints.
- `sensor_msgs`: declared but unused in the current code (see the note in the
  `package.xml` section, above).

### Building each executable

Each `add_executable` + `ament_target_dependencies` pair does two things:

1. Links a command name to a `.cpp`.
2. Links the libraries that binary needs.

| Executable | Source | Specific dependencies |
| --- | --- | --- |
| `vehicle_odometry_subscriber` | `src/vehicle_odometry_subscriber.cpp` | ROS 2, tf2, geometry, PX4 messages and sensors. The launch starts it twice (once per vehicle) with different `name=`/`parameters=`. |
| `interceptor_tf2_odometry` | `src/interceptor_tf2_odometry.cpp` | Adds `px4_ros_com` to convert frames. |
| `target_tf2_odometry` | `src/target_tf2_odometry.cpp` | Adds `px4_ros_com` to convert frames and `px4_ros2_cpp` to compute the origin offset (`vectorToGlobalPosition`); it also publishes velocity. |
| `tf2_listener` | `src/tf2_listener.cpp` | tf2 buffer/listener and PX4 odometry. |
| `pursuit_mode` | `src/pursuit_mode.cpp` | `px4_ros2_cpp` and transforms. |
| `PN_mode` | `src/PN_mode.cpp` | Same as `pursuit_mode`. |

The names repeated inside `ament_target_dependencies` aren't accidental
duplication: they tell the linker which libraries each target uses.

### Tests and install

Inside `if(BUILD_TESTING)`:

- `find_package(ament_lint_auto REQUIRED)` finds the lint system.
- The two `ament_cmake_*_FOUND = TRUE` assignments turn off the copyright and
  cpplint checks, as the file's own comments say.
- `ament_lint_auto_find_test_dependencies()` registers the automatic tests.

`install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` installs the six
binaries in the package's standard folder. `install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})` installs the launch file. Finally,
`ament_package()` generates the metadata ROS 2 needs to find the package.

## 6. `src/interceptor/launch/interceptor.launch.py`

File: [`interceptor.launch.py`](../../src/interceptor/launch/interceptor.launch.py)

This table covers **every** line of the file, in order. Blank lines are left
out because they do nothing; check in the file that the line numbers match.

| Line | Code | Explanation |
| ---: | --- | --- |
| 1 | `#!/usr/bin/env python` | *Shebang*: lets the file run directly as a Python script on Unix systems. |
| 3 | `"""` | Opens a docstring: a multi-line comment that documents the file. |
| 4 | `Launch file for the interceptor/target scenario.` | Descriptive text, not run. |
| 6 | `Starts the Micro XRCE-DDS agent, the 4 odometry/diagnostic nodes` | Sums up what this launch starts: the DDS agent and the 4 odometry/diagnostic nodes. |
| 7 | `(two instances of vehicle_odometry_subscriber, one per vehicle, plus` | Names those nodes: two instances of the diagnostic subscriber, one per vehicle. |
| 8 | `target_tf2_odometry and interceptor_tf2_odometry) and a single guidance` | Completes the list with the two tf2 converters and adds that only one guidance mode is launched. |
| 9 | `mode, chosen with the argument mode:=pn\|pursuit (pn by default).` | Says the mode is chosen with the `mode` argument, which accepts `pn` or `pursuit`, and is `pn` if not given. |
| 10 | `The old name, modo:=, is accepted for now as an alias of mode.` | Notes that the old argument name, `modo`, still works for now as an alias of `mode`. |
| 12 | `Usage example:` | Introduces the example invocation. |
| 13 | `ros2 launch interceptor interceptor.launch.py  # same as mode:=pn` | Shortest invocation: without the argument the default, `pn`, is used. |
| 14 | `ros2 launch interceptor interceptor.launch.py mode:=pursuit` | Same invocation choosing the other guidance mode. |
| 16 | `PX4 is started by hand, each instance in its own terminal (from ~/PX4-Autopilot):` | Makes clear that PX4 isn't started from this file, and from which folder the following commands run. |
| 17 | `interceptor (instance 0): make px4_sitl gz_x500` | Example command to start instance 0 (interceptor). |
| 18 | `target (instance 1):` | Introduces the command for instance 1 (target). |
| 19 | `GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 \` | Example command to start instance 1 (target), 20 m north of the interceptor (`PX4_GZ_MODEL_POSE="0,20"`); the backslash continues on the next line. |
| 20 | `./build/px4_sitl_default/bin/px4 -i 1` | Rest of the instance 1 command. |
| 22 | `On Ctrl-C, the launch takes about 5 s to close: on purpose, to give the` | Warns that closing with Ctrl-C takes about 5 s, and that it's intentional. |
| 23 | `guidance mode time to unregister from PX4 before the agent dies.` | Explains why: to give the guidance mode time to unregister from PX4. |
| 24 | `"""` | Closes the docstring. |
| 26 | `import os` | Imports operating system utilities; only used here to expand `~`. |
| 28 | `from launch import LaunchDescription` | Imports the class that represents "the list of actions to run". |
| 29 | `from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, SetLaunchConfiguration` | Imports the action that declares a launch argument (`mode`, `modo`), the one that runs an arbitrary shell command (the DDS agent), the one that writes a message to the log (`LogInfo`) and the one that changes the value of an already declared argument (`SetLaunchConfiguration`). |
| 30 | `from launch.conditions import IfCondition, UnlessCondition` | Imports the conditions that decide whether an action runs: `IfCondition` runs it if the expression is true and `UnlessCondition` if it's false. |
| 31 | `from launch.substitutions import EqualsSubstitution, LaunchConfiguration` | Imports the substitution that compares two values at launch time (`EqualsSubstitution`) and the one that reads the value of a declared argument (`LaunchConfiguration`). |
| 32 | `from launch_ros.actions import Node` | Imports the action that starts a ROS 2 node from a package. |
| 34 | `MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')` | Expands `~` to the current user's absolute path and stores it in a constant. |
| 37 | `def generate_launch_description():` | Defines the argument-less function that ROS 2 looks for and runs on `ros2 launch`. |
| 39 | `mode_arg = DeclareLaunchArgument(` | Starts declaring the `mode` launch argument. |
| 40 | `'mode',` | Argument name: passed as `mode:=<value>` on the command line. |
| 41 | `default_value='pn',` | Value the argument takes if not given on the command line: `PN_mode` is registered. |
| 42 | `choices=['pn', 'pursuit'],` | Restricts the valid values to `pn` and `pursuit`; with `default_value`, any value outside that list is an error. |
| 43 | `description=(` | Starts the argument description (shown with `ros2 launch interceptor interceptor.launch.py --show-args`). |
| 44 | `'Guidance mode registered in PX4 (pn = PN mode, pursuit = Pursuit '` | First part of the description text. |
| 45 | `'Intercept); only one is launched. Default: pn.'` | Second part: makes clear only one mode is launched and which one is the default. |
| 46 | `),` | Closes the tuple of concatenated `description` strings. |
| 47 | `)` | Closes the `DeclareLaunchArgument` call. |
| 49 | `# Temporary alias for the old argument name. If modo:= is given, its value` | Comment: what follows is a temporary alias for the old name, `modo`, and if it's given, its value replaces `mode`. |
| 50 | `# replaces mode. Remove once modo:= is no longer used anywhere.` | Continues the comment: the alias will be removed once nobody uses `modo:=` any more. |
| 51 | `modo_arg = DeclareLaunchArgument(` | Starts declaring the `modo` argument, which only exists for compatibility. |
| 52 | `'modo',` | Name of the old argument: `modo:=<value>`. |
| 53 | `default_value='',` | Defaults to the empty string, which here means "not given". |
| 54 | `choices=['', 'pn', 'pursuit'],` | Valid values: empty, `pn` or `pursuit`; any other value is an error, just like with `mode`. |
| 55 | `description='Deprecated: temporary alias of mode. Use mode:=pn\|pursuit.',` | Description shown with `--show-args`: warns that it's deprecated and what to use instead. |
| 56 | `)` | Closes the `DeclareLaunchArgument` call. |
| 57 | `modo_given = UnlessCondition(EqualsSubstitution(LaunchConfiguration('modo'), ''))` | Stores the condition "`modo` is not empty" (the user passed `modo:=`) in a variable; it's reused by the next two actions. |
| 58 | `modo_alias = SetLaunchConfiguration('mode', LaunchConfiguration('modo'), condition=modo_given)` | If `modo:=` was given, copies its value into `mode`. Since this action comes before the nodes in the list, the node conditions already see the copied value. |
| 59 | `modo_warning = LogInfo(` | Starts building an action that writes a message to the launch log. |
| 60 | `msg='The modo:= argument is deprecated; use mode:= instead.',` | Warning text: `modo:=` is deprecated and `mode:=` should be used. |
| 61 | `condition=modo_given,` | The warning is only shown if `modo:=` was given. |
| 62 | `)` | Closes the `LogInfo` call. |
| 64 | `# The agent ignores SIGINT to stay alive after Ctrl-C until the launch escalates to` | Comment: explains why the agent ignores SIGINT. |
| 65 | ``# SIGTERM: that gives the guidance mode time to send `Unregistering` to PX4.`` | Continues the comment: it gives the guidance mode time to unregister from PX4. |
| 66 | `micro_xrce_agent = ExecuteProcess(` | Starts building the action that will start the DDS agent. |
| 67 | `cmd=[["trap '' INT; exec ./build/MicroXRCEAgent udp4 -p 8888"]],` | Command to run: `trap '' INT` makes the process ignore SIGINT (Ctrl-C doesn't kill it); `exec` replaces the shell with the agent binary, UDP/IPv4 transport, port 8888, so the shutdown signal (SIGTERM) reaches it directly. |
| 68 | `shell=True,` | Runs that command through a shell, as if typed in a terminal. |
| 69 | `cwd=MICRO_XRCE_DDS_AGENT_DIR,` | Working folder where the command runs (`build/MicroXRCEAgent` must exist inside it). |
| 70 | `output='log',` | The process output goes to the ROS 2 logs, not straight to the terminal. |
| 71 | `)` | Closes the `ExecuteProcess` call. |
| 73 | `target_vehicle_odometry_subscriber_node = Node(` | Starts building the block of the first node. |
| 74 | `package='interceptor',` | ROS 2 package where the executable is found. |
| 75 | `executable='vehicle_odometry_subscriber',` | Name of the binary to run: the parametrised diagnostic subscriber (see [Nodes and topics](Nodes-and-topics.md)). |
| 76 | `name='target_vehicle_odometry_subscriber',` | Visible node name in `ros2 node list`, different from the executable. |
| 77 | `parameters=[{` | Starts the parameter list of this instance. |
| 78 | `'vehicle_name': 'target',` | Sets `vehicle_name` to `target`. |
| 79 | `'odometry_topic': '/px4_1/fmu/out/vehicle_odometry',` | Sets `odometry_topic` to instance 1's topic (target): this way the node name and the topic it listens to match. |
| 80 | `}],` | Closes the parameter list. |
| 81 | `output='log',` | Its output goes to the logs, not the screen. |
| 82 | `)` | Closes this node's block. |
| 84 | `target_tf2_odometry_node = Node(` | Starts the block of the target's tf2 converter. |
| 85 | `package='interceptor',` | Same as line 74. |
| 86 | `executable='target_tf2_odometry',` | Executable that converts the target's odometry and publishes `target/base_link` and `target/velocity`. |
| 87 | `output='screen',` | Here the output does go straight to the terminal (`screen`), unlike the diagnostics. |
| 88 | `)` | Closes the block. |
| 90 | `interceptor_vehicle_odometry_subscriber_node = Node(` | Starts the block of the second diagnostic subscriber. |
| 91 | `package='interceptor',` | Same as above. |
| 92 | `executable='vehicle_odometry_subscriber',` | Same executable as the first instance, with other parameters. |
| 93 | `name='interceptor_vehicle_odometry_subscriber',` | Visible name of this instance. |
| 94 | `parameters=[{` | Starts the parameter list. |
| 95 | `'vehicle_name': 'interceptor',` | Sets `vehicle_name` to `interceptor`. |
| 96 | `'odometry_topic': '/fmu/out/vehicle_odometry',` | Sets `odometry_topic` to instance 0's topic (interceptor). |
| 97 | `}],` | Closes the parameter list. |
| 98 | `output='log',` | Output to the logs. |
| 99 | `)` | Closes the block. |
| 101 | `interceptor_tf2_odometry_node = Node(` | Starts the block of the interceptor's tf2 converter. |
| 102 | `package='interceptor',` | Same as above. |
| 103 | `executable='interceptor_tf2_odometry',` | Publishes `map -> interceptor/base_link`. |
| 104 | `output='screen',` | Output to the screen. |
| 105 | `)` | Closes the block. |
| 107 | `# tf2_listener_node = Node(` | Commented-out line: it starts with `#`, so Python ignores it completely. |
| 108 | `# package='interceptor',` | Commented out, ignored. |
| 109 | `# executable='tf2_listener',` | Commented out, ignored. |
| 110 | `# output='screen',` | Commented out, ignored. |
| 111 | `# )` | Commented out, ignored. This whole 5-line block is disabled: that's why `tf2_listener` doesn't start with the default launch. |
| 113 | `pursuit_mode_node = Node(` | Starts the block of the pure pursuit mode. |
| 114 | `package='interceptor',` | Same as above. |
| 115 | `executable='pursuit_mode',` | Executable of the `pursuit_mode` mode. |
| 116 | `output='screen',` | Output to the screen. |
| 117 | `condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'pursuit')),` | This node only starts if `mode` is `pursuit`: it compares the argument value with the string `'pursuit'`. |
| 118 | `)` | Closes the block. |
| 120 | `PN_mode_node = Node(` | Starts the block of the proportional navigation mode. |
| 121 | `package='interceptor',` | Same as above. |
| 122 | `executable='PN_mode',` | Executable of the `PN_mode` mode. |
| 123 | `output='screen',` | Output to the screen. |
| 124 | `condition=IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'pn')),` | This node only starts if `mode` is `pn`. |
| 125 | `)` | Closes the block. |
| 127 | `return LaunchDescription([` | Builds and returns the list of actions ROS 2 will run; the order of this list is the start-up order. |
| 128 | `mode_arg,` | First the `mode` argument is declared (it must come before it's used in the node conditions). |
| 129 | `modo_arg,` | Then the `modo` alias. |
| 130 | `modo_alias,` | If `modo:=` was given, its value is copied into `mode` here, before the node conditions are evaluated. |
| 131 | `modo_warning,` | And, in that case, the warning that `modo` is deprecated is shown. |
| 132 | `micro_xrce_agent,` | Then the DDS agent is started. |
| 133 | `target_vehicle_odometry_subscriber_node,` | Then this node. |
| 134 | `target_tf2_odometry_node,` | Then this one. |
| 135 | `interceptor_vehicle_odometry_subscriber_node,` | Then this one. |
| 136 | `interceptor_tf2_odometry_node,` | Then this one. |
| 137 | `pursuit_mode_node,` | This one only really starts if `mode:=pursuit`. |
| 138 | `PN_mode_node,` | And this one only if `mode:=pn`. |
| 139 | `])` | Closes the list and the `LaunchDescription` call. |

`tf2_listener_node` isn't in this final list because its variable was never
even created (it's commented out above): commenting the block isn't the only
step; if it were uncommented, it would also have to be added here to run.
`pursuit_mode_node` and `PN_mode_node` are always in the list, but their
`condition` decides at launch time whether the process actually starts; only
one of them does, depending on `mode`. PX4 SITL doesn't appear anywhere in this
file: it's started by hand in other terminals, as the docstring at the top
reminds you.

That's all for the build and start-up files. Carry on with
[Odometry and tf2 nodes, line by line](Line-by-line-odometry-nodes.md).

---

🏠 [Home](Home.md) · ⬅️ Previous: [Code walkthrough](Code-walkthrough.md) · ➡️ Next: [Line by line: odometry and tf2 nodes](Line-by-line-odometry-nodes.md)
