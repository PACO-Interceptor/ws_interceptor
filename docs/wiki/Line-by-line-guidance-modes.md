# Line by line: guidance modes

This is the third of three pages that go line by line through every file of
our own in `src/interceptor`. Start with
[Project and build files](Line-by-line-project-and-build-files.md) if you
haven't read it yet: it has the C++ syntax glossary (`override`, `explicit`,
`constexpr`, templates, references, `.cross()`/`.normalized()`...) that you'll
also need here. The second page is
[Odometry and tf2 nodes, line by line](Line-by-line-odometry-nodes.md).

This page covers `pursuit_mode.cpp` and `PN_mode.cpp`: the two flight modes
that compute a trajectory setpoint from the target's position (and, in PN, its
velocity).

## 1. `pursuit_mode.cpp`

File: [`pursuit_mode.cpp`](../../src/interceptor/src/pursuit_mode.cpp)

| Line | Code | Explanation |
| ---: | --- | --- |
| 1 | `#include <Eigen/Eigen>` | Brings vectors and linear algebra operations (`Eigen::Vector3f`, `.norm()`, `.normalized()`...). |
| 2 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Brings `TransformStamped`, the result of a tf2 lookup. |
| 3 | `#include <px4_ros2/components/mode.hpp>` | Brings `ModeBase`, the base class of a custom flight mode. |
| 4 | `#include <px4_ros2/components/node_with_mode.hpp>` | Brings `NodeWithMode`, the wrapper that registers the mode as a ROS 2 node. |
| 5 | `#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>` | Brings `TrajectorySetpointType`, used to send velocity/acceleration/yaw to PX4. |
| 6 | `#include <px4_ros2/odometry/local_position.hpp>` | Brings `OdometryLocalPosition`, to read our own position/velocity. |
| 7 | `#include <px4_ros2/utils/geometry.hpp>` | Geometry utilities from the `px4_ros2` library. |
| 8 | `#include <px4_ros_com/frame_transforms.h>` | Brings the NED↔ENU conversion functions. |
| 9 | `#include <rclcpp/rclcpp.hpp>` | Brings `Node`, `init`, `spin`, logging. |
| 10 | `#include <tf2/exceptions.h>` | Brings `tf2::TransformException`. |
| 11 | `#include <tf2_ros/buffer.h>` | Brings `Buffer`, where tf2 stores the transforms. |
| 12 | `#include <tf2_ros/transform_listener.h>` | Brings `TransformListener`, which fills that buffer. |
| 14 | `using namespace std::chrono_literals; // NOLINT` | Lets you write `50ms` as a duration; the `NOLINT` comment asks the linter not to complain about this `using namespace`. |
| 16 | `static const std::string kName = "Pursuit Intercept";` | Name of the mode as PX4 will show it. |
| 17 | `static const std::string kMapFrame = "map";` | Name of the parent frame used in tf2 lookups. |
| 19 | `class PursuitMode : public px4_ros2::ModeBase` | Declares the class, inheriting from `ModeBase`, so PX4 can register it and call its methods. |
| 20 | `{` | Opens the class body. |
| 21 | `public:` | What follows is accessible from outside. |
| 22 | `explicit PursuitMode(rclcpp::Node & node)` | Constructor that takes a reference to the ROS 2 node that wraps it. |
| 23 | `: ModeBase(node, Settings{kName})` | Initialises the base class with the node and the mode name. |
| 24 | `{` | Opens the constructor body. |
| 25 | `_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);` | Creates the object used to send trajectory setpoints. |
| 26 | `_own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);` | Creates the reader of the interceptor's own position/velocity. |
| 28 | `_target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");` | Declares the `target_frame` parameter (default `target/base_link`) and stores its value. |
| 30 | `_tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());` | Creates the tf2 buffer, tied to the node's clock. |
| 31 | `_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);` | Creates the listener that fills that buffer by listening to tf2. |
| 33 | `_target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});` | Creates a timer that calls `updateTargetPosition()` every 50 ms. |
| 34 | `}` | Closes the constructor. |
| 36 | `void onActivate() override {_target_reached = false;}` | Method PX4 calls when the mode is activated; resets `_target_reached` to `false` so that, if the mode is activated again after reaching the target, it can report and call `completed()` again next time. |
| 38 | `void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override` | Method PX4 calls to decide whether the mode can arm; `override` confirms it replaces the one in `ModeBase`. |
| 39 | `{` | Opens the method body. |
| 40 | `if (!_target_valid) {` | If no valid target position has been received yet... |
| 41-44 | `reporter.armingCheckFailureExt(px4_ros2::events::ID("pursuit_no_target"), px4_ros2::events::Log::Error, "No target odometry received yet");` | ...reports an arming failure to PX4, with an identifier and an error message. |
| 45 | `}` | Closes the `if`. |
| 46 | `}` | Closes `checkArmingAndRunConditions`. |
| 48 | `void updateSetpoint(float dt_s) override` | Method PX4 calls on every cycle of the mode to ask for a new setpoint. |
| 49 | `{` | Opens the method body. |
| 50 | `(void)dt_s;` | Marks the `dt_s` parameter as deliberately unused (it's part of the `ModeBase` interface, but this mode doesn't need it). |
| 52 | `if (!_target_valid) {` | If there's no valid target position yet... |
| 53 | `RCLCPP_WARN(node().get_logger(), "Target position not valid yet. Skipping setpoint update.");` | ...logs a warning... |
| 54 | `return;` | ...and returns without computing any setpoint. |
| 55 | `}` | Closes the `if`. |
| 57 | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();` | Computes the line of sight (LOS): target position minus own position, both in NED. |
| 59 | `if (los.norm() < 1.0f) {` | If the distance to the target is less than 1 metre... |
| 60 | `if (!_target_reached) {` | ...and the reach hasn't been recorded yet during this stretch within the metre (avoids repeating the message and `completed()` while the interceptor stays there)... |
| 61 | `RCLCPP_INFO(node().get_logger(), "Target reached. Stopping pursuit.");` | ...logs that the target was reached... |
| 62 | `completed(px4_ros2::Result::Success);` | ...tells PX4/the library that the mode finished successfully... |
| 63 | `_target_reached = true;` | ...and records that it has reported it, so it isn't repeated while still within the metre. |
| 64 | `}` | Closes the `if (!_target_reached)`. |
| 65 | `return;` | Returns without computing more setpoints, whether or not it reported in this cycle. |
| 66 | `}` | Closes the `if (los.norm() < 1.0f)`. |
| 67 | `_target_reached = false;` | If the distance is no longer below 1 m, clears the flag: if the target moves away and the interceptor closes in again, the message and `completed()` can happen again on the next reach. |
| 69 | `const Eigen::Vector2f los_horizontal(los.x(), los.y());` | Takes only the X and Y components of `los` (drops the vertical axis). |
| 70 | `Eigen::Vector2f velocity_horizontal = Eigen::Vector2f::Zero();` | Starts the horizontal velocity at zero in case it isn't recomputed. |
| 71 | `if (los_horizontal.norm() > kMinHorizontalDistance) {` | If the horizontal distance is above the minimum (`0.1 m`), to avoid normalising an almost zero vector... |
| 72 | `velocity_horizontal = los_horizontal.normalized() * kMaxHorizontalSpeed;` | ...the horizontal velocity points at the target at the maximum speed (`5 m/s`). |
| 73 | `_last_yaw = atan2f(los_horizontal.y(), los_horizontal.x());` | Computes the heading angle (yaw) towards the target and stores it as the "last valid yaw". |
| 74 | `}` | Closes the `if`. |
| 76 | `const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);` | Limits the vertical component of `los` to between `-2` and `2 m/s`: that's the requested vertical speed. |
| 78 | `const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};` | Puts the three components together in a single 3D velocity vector. |
| 80 | `_trajectory_setpoint->update(velocity, {}, _last_yaw);` | Sends the setpoint to PX4: the computed velocity, no acceleration (`{}`) and the stored yaw. |
| 81 | `}` | Closes `updateSetpoint`. |
| 83 | `private:` | What follows is only accessible inside the class. |
| 84 | `void updateTargetPosition()` | Declares the method the timer of line 33 calls every 50 ms. |
| 85 | `{` | Opens the method body. |
| 86 | `geometry_msgs::msg::TransformStamped t;` | Variable that will hold the tf2 lookup result. |
| 88 | `try {` | Starts the block that may fail if the target frame doesn't exist yet. |
| 89 | `t = _tf_buffer->lookupTransform(kMapFrame, _target_frame, tf2::TimePointZero);` | Asks for the `map -> target/base_link` transform, using the latest available data. |
| 90 | `} catch (const tf2::TransformException & ex) {` | If it fails (the frame doesn't exist yet), it's caught here. |
| 91-93 | `RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000, "Could not transform %s to %s: %s", ...);` | Logs a warning, but at most once every 5000 ms (5 s), so the log isn't flooded. |
| 94 | `return;` | Returns without updating the target position. |
| 95 | `}` | Closes the `catch` block. |
| 97-98 | `const Eigen::Vector3d position_enu(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);` | Builds a `double` vector with the received ENU translation. |
| 99-100 | `const Eigen::Vector3d position_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(position_enu);` | Converts that position from ENU to NED, so it can be compared with our own odometry (also in NED). |
| 102 | `_target_position_ned = position_ned.cast<float>();` | Stores the converted position as `float` (that's how the member is declared). |
| 103 | `_target_valid = true;` | Records that there is a valid target position; from here on `updateSetpoint` can compute. |
| 104 | `}` | Closes `updateTargetPosition`. |
| 106 | `static constexpr float kMaxHorizontalSpeed = 5.0f; // [m/s]` | Constant: maximum horizontal speed, 5 m/s. |
| 107 | `static constexpr float kMaxVerticalSpeed = 2.0f; // [m/s]` | Constant: maximum vertical speed, 2 m/s. |
| 108 | `static constexpr float kMinHorizontalDistance = 0.1f; // [m]` | Constant: minimum horizontal distance to normalise `los_horizontal`, 0.1 m. |
| 110 | `std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;` | Stores the setpoint sender. |
| 111 | `std::shared_ptr<px4_ros2::OdometryLocalPosition> _own_position;` | Stores the reader of our own position/velocity. |
| 113 | `std::string _target_frame;` | Stores the name of the target's tf2 frame. |
| 114 | `std::unique_ptr<tf2_ros::Buffer> _tf_buffer;` | Stores the tf2 buffer. |
| 115 | `std::shared_ptr<tf2_ros::TransformListener> _tf_listener;` | Stores the tf2 listener. |
| 116 | `rclcpp::TimerBase::SharedPtr _target_lookup_timer;` | Stores the 50 ms timer. |
| 118 | `Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};` | Last known NED position of the target, initialised to zero. |
| 119 | `bool _target_valid{false};` | Whether at least one valid target position has been received. |
| 120 | `bool _target_reached{false};` | Whether the reach (LOS < 1 m) has already been recorded in the current stretch; reset in `onActivate()` and whenever the distance is ≥ 1 m again. |
| 121 | `float _last_yaw{0.f};` | Last computed yaw, kept when the target is almost directly above. |
| 122 | `};` | Closes the `PursuitMode` class. |
| 124 | `using PursuitModeNode = px4_ros2::NodeWithMode<PursuitMode>;` | Defines an alias: `PursuitModeNode` is "a `PursuitMode` wrapped as a ROS 2/PX4 node". |
| 126 | `static const std::string kNodeName = "pursuit_mode";` | Name of the executable/node ROS 2 will see. |
| 127 | `static const bool kEnableDebugOutput = true;` | Turns on extra debug output from the `px4_ros2` library. |
| 129 | `int main(int argc, char * argv[])` | Program entry point. |
| 130 | `{` | Opens `main`. |
| 131 | `rclcpp::init(argc, argv);` | Initialises ROS 2. |
| 132 | `rclcpp::spin(std::make_shared<PursuitModeNode>(kNodeName, kEnableDebugOutput));` | Creates the node with that name and debug option, and keeps it alive processing events (timer, `checkArmingAndRunConditions`, `updateSetpoint`...). |
| 133 | `rclcpp::shutdown();` | Releases the ROS 2 resources when `spin` returns. |
| 134 | `return 0;` | Ends without errors. |
| 135 | `}` | Closes `main`. |

## 2. `PN_mode.cpp`

File: [`PN_mode.cpp`](../../src/interceptor/src/PN_mode.cpp)

`PN_mode.cpp` shares almost all of `pursuit_mode.cpp`'s structure (same
includes, same constructor pattern, same life cycle). The table marks with
**★** the lines that have no equivalent in `pursuit_mode.cpp`.

| Line | Code | Explanation |
| ---: | --- | --- |
| 1-10 | Doxygen comment `/** ... */` | Documents that this file implements Proportional Navigation (PN). |
| 12-23 | Same `#include` lines as `pursuit_mode.cpp` (lines 1-12) | Identical, same purpose. |
| 24 | ★ `#include <geometry_msgs/msg/twist_stamped.hpp>` | Extra: brings `TwistStamped`, needed because PN does need the target's velocity. |
| 26 | `using namespace std::chrono_literals; // NOLINT` | Same as in Pursuit. |
| 28 | `static const std::string kName = "PN mode";` | Name of the mode shown by PX4 (different text from Pursuit). |
| 29 | `static const std::string kMapFrame = "map";` | Same as in Pursuit. |
| 31 | `class PN_Mode : public px4_ros2::ModeBase` | Declares the `PN_Mode` class (a different name from `PursuitMode`), same base class. |
| 32 | `{` | Opens the body. |
| 33 | `public:` | Same. |
| 34 | `explicit PN_Mode(rclcpp::Node & node)` | Constructor, same pattern as Pursuit. |
| 35 | `: ModeBase(node, Settings{kName})` | Same. |
| 36 | `{` | Opens the constructor. |
| 37 | `_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);` | Same as in Pursuit. |
| 38 | `_own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);` | Same. |
| 40 | `_target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");` | Same. |
| 41 | `_target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});` | Same, although here it comes earlier in the file than in Pursuit; the order of these lines doesn't change the behaviour. |
| 42-43 | ★ `_target_velocity_topic = node.declare_parameter<std::string>("target_velocity_topic", "target/velocity");` | Extra: declares the velocity topic parameter, default `target/velocity`. |
| 45 | ★ `// Subscribe to the target's velocity topic. ENU to NED conversion is done in the callback.` | Explanatory comment. |
| 46-47 | ★ `_target_velocity_sub = node.create_subscription<geometry_msgs::msg::TwistStamped>(_target_velocity_topic, 10,` | Extra: creates the subscription to `target/velocity`, with a queue of 10 messages. |
| 48 | ★ `[this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {` | Lambda run for every received velocity message. |
| 49 | ★ `using px4_ros_com::frame_transforms::enu_to_ned_local_frame;` | Shortens the name of the conversion function. |
| 51 | ★ `// Convert the received ENU velocity to NED frame` | Explanatory comment. |
| 52-53 | ★ `Eigen::Vector3d velocity_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);` | Builds a `double` vector with the ENU linear velocity of the `TwistStamped` message. |
| 54 | ★ `Eigen::Vector3d velocity_ned = enu_to_ned_local_frame(velocity_enu);` | Converts that velocity to NED. |
| 56 | ★ `_target_velocity_ned = velocity_ned.cast<float>(); // .cast to convert from double to float` | Stores the converted velocity as `float`. |
| 58 | ★ `_target_velocity_valid = true;` | Records that at least one valid target velocity has been received. |
| 59 | ★ `});` | Closes the lambda and `create_subscription`. |
| 61 | `_tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());` | Same as in Pursuit. |
| 62 | `_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);` | Same. |
| 64 | `}` | Closes the constructor. |
| 66 | `void onActivate() override {_target_reached = false;}` | Same method as in Pursuit (line 36 of that table): resets `_target_reached` when the mode is activated. |
| 68 | ★ `// Safety checks` | Comment introducing `checkArmingAndRunConditions`; it has no textual equivalent in Pursuit. |
| 69 | `void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override` | Same method as in Pursuit. |
| 70 | `{` | Opens the body. |
| 71 | ★ `if (!_target_valid \|\| !_target_velocity_valid) {` | **Key difference**: here both a valid position *and* a valid velocity are required (`\|\|` is C++'s "or" between the two negations); Pursuit only requires the position. |
| 72-75 | `reporter.armingCheckFailureExt(...)` | Same as in Pursuit, same error message (the text doesn't mention the velocity even though the condition checks it). |
| 76 | `}` | Closes the `if`. |
| 77 | `}` | Closes the method. |
| 79 | ★ `// PN main logic` | Comment introducing `updateSetpoint`; no equivalent in Pursuit. |
| 80 | `void updateSetpoint(float dt_s) override` | Same signature as in Pursuit. |
| 81 | `{` | Opens the body. |
| 82 | `(void)dt_s;` | Same: unused parameter. |
| 84-87 | `if (!_target_valid) { RCLCPP_WARN(...); return; }` | Same as in Pursuit. |
| 89 | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();` | Same as in Pursuit: line of sight. |
| 90 | ★ `const Eigen::Vector3f v_rel = _target_velocity_ned - _own_position->velocityNed();` | Extra: relative velocity = target velocity minus own velocity. |
| 92 | `if (los.norm() < 1.0f) {` | Same as in Pursuit (line 59 of that table): if the distance to the target is less than 1 metre... |
| 93 | `if (!_target_reached) {` | Same as in Pursuit (line 60): avoids repeating the message and `completed()` on every cycle while still within the metre. |
| 94 | `RCLCPP_INFO(node().get_logger(), "Target reached. Stopping pursuit.");` | Same as in Pursuit (line 61). |
| 95 | `completed(px4_ros2::Result::Success);` | Same as in Pursuit (line 62). |
| 96 | `_target_reached = true;` | Same as in Pursuit (line 63). |
| 97 | `}` | Closes the `if (!_target_reached)`. |
| 98 | `return;` | Same as in Pursuit (line 65). |
| 99 | `}` | Closes the `if (los.norm() < 1.0f)`. |
| 100 | `_target_reached = false;` | Same as in Pursuit (line 67): clears the flag if the distance is ≥ 1 m again. |
| 102 | ★ `Eigen::Vector3f a_cmd = Eigen::Vector3f::Zero();` | Extra: the navigation acceleration starts at zero. |
| 105 | ★ `if (los.norm() < kPnMinRange) {` | If the distance is below `kPnMinRange` (7 m)... |
| 106 | ★ `a_cmd = Eigen::Vector3f::Zero();` | ...no PN acceleration is applied (too close for the formula to be reliable). |
| 107 | ★ `} else {` | If the distance is greater than or equal to `kPnMinRange`... |
| 108 | ★ `// LOS rotation rate ω = (los × v_rel) / (los · los)` | Comment explaining the formula on the next line. |
| 109 | ★ `const Eigen::Vector3f los_rotation_rate = los.cross(v_rel) / (los.squaredNorm() + 1e-6f);` | Computes ω, the rotation rate of the line of sight; the `1e-6f` avoids dividing by exactly zero. |
| 110 | ★ `a_cmd = kNavigationConstant * los_rotation_rate.cross(v_rel);` | Applies the proportional navigation formula: acceleration proportional to ω × relative velocity. |
| 111 | ★ `const float a_cmd_norm = a_cmd.norm();` | Computes the size of that acceleration. |
| 113 | ★ `if (a_cmd_norm > 1e-6f) {` | If the computed acceleration isn't practically zero... |
| 114 | ★ `a_cmd = a_cmd.normalized() * std::min(a_cmd_norm, kMaxAcceleration);` | ...it's normalised and capped at `kMaxAcceleration` (3 m/s²). |
| 115 | ★ `} else {` | If it is practically zero... |
| 116 | ★ `a_cmd = Eigen::Vector3f::Zero();` | ...it's set to exactly zero (avoids normalising an almost zero vector). |
| 117 | ★ `}` | Closes the inner `if`/`else`. |
| 118 | ★ `}` | Closes the `kPnMinRange` `if`/`else`. |
| 120-125 | `los_horizontal`, `velocity_horizontal`, `_last_yaw = atan2f(...)` | Same pattern as in Pursuit for the horizontal pursuit velocity. |
| 127 | `const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);` | Same as in Pursuit. |
| 129 | `const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};` | Same as in Pursuit. |
| 131 | ★ `_trajectory_setpoint->update(velocity, a_cmd, _last_yaw);` | **Key difference**: here `a_cmd` is sent (Pursuit sends `{}`, no acceleration). |
| 132 | `}` | Closes `updateSetpoint`. |
| 134 | `private:` | Same as in Pursuit (line 83 of that table): what follows is only accessible inside the class. |
| 135-155 | The whole `updateTargetPosition()` | Identical line by line to the one in `pursuit_mode.cpp` (lines 84-104 of that table): same `lookupTransform`, same `catch`, same ENU→NED conversion. |
| 157 | `static constexpr float kMaxHorizontalSpeed = 7.0f; // [m/s]` | **Different value**: 7 m/s instead of 5 m/s. |
| 158 | `static constexpr float kMaxVerticalSpeed = 2.0f; // [m/s]` | Same value as in Pursuit. |
| 159 | `static constexpr float kMinHorizontalDistance = 0.1f; // [m]` | Same value as in Pursuit. |
| 161 | ★ `static constexpr float kNavigationConstant = 3.5f; // Proportional navigation constant (N/lambda)` | Extra: proportional navigation gain. |
| 162 | ★ `static constexpr float kMaxAcceleration = 3.0f; // [m/s^2]` | Extra: maximum allowed acceleration. |
| 163 | ★ `static constexpr float kPnMinRange = 7.0f; // [m] Minimum range for PN to be active` | Extra: minimum distance for the PN acceleration to be active. |
| 165-171 | Members `_trajectory_setpoint`, `_own_position`, `_target_frame`, `_tf_buffer`, `_tf_listener`, `_target_lookup_timer` | Identical to Pursuit's. |
| 173 | `Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};` | Same as in Pursuit. |
| 174 | ★ `Eigen::Vector3f _target_velocity_ned{Eigen::Vector3f::Zero()};` | Extra: last known NED velocity of the target. |
| 175 | ★ `std::string _target_velocity_topic;` | Extra: name of the velocity topic. |
| 176 | ★ `rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _target_velocity_sub;` | Extra: stores the velocity subscription. |
| 178 | ★ `bool _target_velocity_valid{false};` | Extra: whether a valid target velocity has arrived. |
| 179 | `bool _target_valid{false};` | Same as in Pursuit. |
| 180 | `bool _target_reached{false};` | Same as in Pursuit (line 120 of that table): whether the reach has already been recorded in the current stretch. |
| 181 | `float _last_yaw{0.f};` | Same as in Pursuit. |
| 182 | `};` | Closes the `PN_Mode` class. |
| 184 | `using PN_ModeNode = px4_ros2::NodeWithMode<PN_Mode>;` | Same pattern as `PursuitModeNode`, for this class. |
| 186 | `static const std::string kNodeName = "PN_mode";` | Different executable/node name. |
| 187 | `static const bool kEnableDebugOutput = true;` | Same as in Pursuit. |
| 189-195 | The whole `main` | Same pattern as `pursuit_mode.cpp` (lines 129-135 of that table), using `PN_ModeNode` instead of `PursuitModeNode`. |

Comparing these two tables line by line is the most direct way to see exactly
what PN adds on top of Pursuit: the velocity subscription (lines 42-59), the
arming condition that also requires a valid velocity (line 71), `v_rel` (line
90), the `a_cmd` calculation (lines 102-118) and the fact that `a_cmd` is sent
in the final `update()` (line 131). The reach block (`onActivate`, line 66, and
lines 92-100) is identical to Pursuit's. Everything else (class structure,
`updateTargetPosition`, `main`) is the same pattern with different names.

## Relating lines to observable behaviour

Understanding a line is more useful when you can check it at run time. Use this
table as a bridge between code and tools:

| Code you're studying | What you can observe |
| --- | --- |
| `create_subscription` | `ros2 node info` shows the subscription. |
| `create_publisher` | `ros2 topic info` shows the publisher. |
| `create_wall_timer` | The rate shows up in `ros2 topic hz` if the timer publishes. |
| `sendTransform` | `tf2_echo` can look up the generated frame. |
| `lookupTransform` | Warnings appear if the frame doesn't exist yet. |
| `TrajectorySetpointType::update` | The mode produces references that PX4 consumes. |
| `armingCheckFailureExt` | The mode reports why it isn't ready to arm yet. |
| `completed(Success)` | The mode reports that it has met its completion criterion. |

This avoids reading the code as isolated text: each block has a consequence in
the ROS 2 graph, in the tf2 tree or in the state of the PX4 mode.

## Example: reading a callback

To read an odometry callback, split its operations in your head:

1. **Input**: which message type does it receive and who publishes it?
2. **Extraction**: which fields are read (`position`, `velocity`, `q`)?
3. **Conversion**: does it change frame or numeric type?
4. **Storage**: is it stored for another timer or method?
5. **Output**: does it publish a topic, a transform or a log line?

In the `target/velocity` subscription of `PN_mode.cpp` (lines 46-59), for
example: the input is a `TwistStamped` published by `target_tf2_odometry`; the
extraction is the three fields of `twist.linear`; the conversion takes the
velocity from ENU to NED and from `double` to `float`; the storage is the
`_target_velocity_ned` and `_target_velocity_valid` members; and the output
isn't immediate: it publishes nothing and updates no transform, it just leaves
the data ready for `updateSetpoint` to use later when computing `v_rel`. That's
why this callback runs at its own pace (once per `target/velocity` message)
while the guidance is recomputed at the pace of `updateSetpoint`.

## What not to infer from a single line

A line that creates a subscription doesn't prove there are messages. A line
that declares a publisher doesn't guarantee there's a subscriber. A correct
mathematical conversion doesn't guarantee the data has valid timestamps either.
Always combine:

- what the code declares;
- what `ros2 node/topic` shows;
- the actual rate;
- the logs;
- and, for tf2, the frame tree.

You get the full picture by comparing those five sources.

---

🏠 [Home](Home.md) · ⬅️ Previous: [Line by line: odometry and tf2 nodes](Line-by-line-odometry-nodes.md) · ➡️ Next: [Workspace map and dependencies](Workspace-file-map.md)
