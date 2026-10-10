# Line by line: odometry and tf2 nodes

This is the second of three pages that go line by line through every file of
our own in `src/interceptor`. Start with
[Project and build files](Line-by-line-project-and-build-files.md) if you
haven't read it yet: it has the C++ syntax glossary (`override`, `explicit`,
`constexpr`, templates, references, `.cross()`/`.normalized()`...) and the
"how to use this analysis" notes that also apply here. The third page is
[Guidance modes, line by line](Line-by-line-guidance-modes.md).

This page covers the odometry diagnostic subscriber
(`vehicle_odometry_subscriber`, a single executable the launch starts twice
with different parameters), the two tf2 converters
(`interceptor_tf2_odometry`, `target_tf2_odometry`) and `tf2_listener`: the
nodes that receive PX4 odometry, convert it and relate it through tf2, without
computing any flight setpoint.

## 1. `vehicle_odometry_subscriber.cpp`

File: [`vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/vehicle_odometry_subscriber.cpp)

A single executable is used to diagnose both vehicles: the launch starts it
twice, each time with its own `name=` and its own `vehicle_name`/`odometry_topic`
parameters (see [Project and build files](Line-by-line-project-and-build-files.md)).

| Line | Code | Explanation |
| ---: | --- | --- |
| 1-8 | Doxygen comment `/** ... */` | Documents the file, its purpose (`Vehicle Odometry uORB topic listener example`) and author, and explains that one executable serves both vehicles depending on its parameters. |
| 10 | `#include <rclcpp/rclcpp.hpp>` | Brings `Node`, `init`, `spin`, QoS and logging. |
| 11 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Brings the `VehicleOdometry` type. |
| 13-15 | Class Doxygen comment | Documents the class that follows. |
| 16 | `class VehicleOdometrySubscriber : public rclcpp::Node` | Declares the class, inheriting from `rclcpp::Node`. |
| 17 | `{` | Opens the class body. |
| 18 | `public:` | What follows is accessible from outside the class. |
| 19 | `explicit VehicleOdometrySubscriber()` | Declares the constructor without parameters; `explicit` prevents implicit conversions. |
| 20 | `: Node("vehicle_odometry_subscriber")` | Calls the `Node` constructor with this default visible name; the launch gives each instance a different one with `name=`. |
| 21 | `{` | Opens the constructor body. |
| 22 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");` | Declares the `vehicle_name` parameter (default `"interceptor"`) and stores its value. |
| 23-24 | `std::string odometry_topic = this->declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");` | Declares the `odometry_topic` parameter (default `/fmu/out/vehicle_odometry`) and stores it in a local variable. |
| 26 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copies a QoS profile meant for sensor data. |
| 27 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Turns that profile into a ROS 2 `QoS` with a queue of 5 messages. |
| 29 | `subscription_ =` | Starts assigning the newly created subscription to the `subscription_` member. |
| 30 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>(odometry_topic, qos,` | Creates the subscription to the topic given by the `odometry_topic` parameter, with the `qos` quality of service. |
| 31 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda run for every received message; `msg` is that message. |
| 32 | `std::cout << "\n\n\n...";` | Prints many blank lines to visually "clear" the console before each sample. |
| 33 | `std::cout << "RECEIVED VEHICLE ODOMETRY DATA (" << vehicle_name_ << ")" << std::endl;` | Prints a header with the vehicle name read from the parameter, e.g. `RECEIVED VEHICLE ODOMETRY DATA (target)`. |
| 34 | `std::cout << "Timestamp: " << msg->timestamp << std::endl;` | Prints the PX4 message timestamp. |
| 35 | `std::cout << "Pose frame: " << msg->pose_frame << std::endl;` | Prints the frame identifier PX4 gives the pose. |
| 36 | `std::cout << "X Position: " << msg->position[0] << std::endl;` | Prints the X position (NED) exactly as PX4 sends it, unconverted. |
| 37 | `std::cout << "Y Position: " << msg->position[1] << std::endl;` | Y position (NED). |
| 38 | `std::cout << "Z Position: " << msg->position[2] << std::endl;` | Z position (NED). |
| 39 | `std::cout << "q[0]: " << msg->q[0] << std::endl;` | First component of the orientation quaternion (`w`). |
| 40 | `std::cout << "q[1]: " << msg->q[1] << std::endl;` | Second component (`x`). |
| 41 | `std::cout << "q[2]: " << msg->q[2] << std::endl;` | Third component (`y`). |
| 42 | `std::cout << "q[3]: " << msg->q[3] << std::endl;` | Fourth component (`z`). |
| 43 | `std::cout << "X Velocity: " << msg->velocity[0] << std::endl;` | X velocity (NED). |
| 44 | `std::cout << "Y Velocity: " << msg->velocity[1] << std::endl;` | Y velocity (NED). |
| 45 | `std::cout << "Z Velocity: " << msg->velocity[2] << std::endl;` | Z velocity (NED). |
| 46-48 | `std::cout << "Velocity magnitude: " << sqrt(msg->velocity[0] * msg->velocity[0] + msg->velocity[1] * msg->velocity[1] + msg->velocity[2] * msg->velocity[2]) << std::endl;` | Computes and prints the length of the velocity vector (square root of the sum of squares). |
| 49 | `});` | Closes the lambda and the `create_subscription` call. |
| 50 | `}` | Closes the constructor. |
| 52 | `private:` | What follows is only accessible inside the class. |
| 53 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Stores the subscription; if it weren't stored here, it would be destroyed when the constructor ends and no more messages would arrive. |
| 54 | `std::string vehicle_name_;` | Stores the vehicle name read from the parameter, used in the printed header. |
| 56 | `};` | Closes the class. |
| 58 | `int main(int argc, char *argv[])` | Program entry point. |
| 59 | `{` | Opens `main`. |
| 60 | `std::cout << "Starting vehicle_odometry_subscriber node..." << std::endl;` | Prints a start-up message. |
| 61 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Turns off standard output buffering so `std::cout` output appears immediately. |
| 62 | `rclcpp::init(argc, argv);` | Initialises ROS 2. |
| 63 | `rclcpp::spin(std::make_shared<VehicleOdometrySubscriber>());` | Creates the node and keeps it alive, running the callback of line 31 while the process runs. |
| 65 | `rclcpp::shutdown();` | Releases the ROS 2 resources when `spin` returns. |
| 66 | `return 0;` | Tells the operating system the program ended without errors. |
| 67 | `}` | Closes `main`. |

### Why this matters

There used to be two almost identical files, one per vehicle, that only
differed in the node name and the topic. That duplicate ended up causing a bug
that stayed in the code for a while: the executable called `target_...`
actually listened to the interceptor's odometry, and vice versa.

Now there's a single file and the vehicle is chosen with parameters, so the
launch sets each node's name next to the topic it passes: both pieces of data
are in the same place and can't get out of step. It's an example of why
parametrising is better than duplicating: the bug wasn't a one-off slip, it was
something the duplication made easy.

## 2. `interceptor_tf2_odometry.cpp`

File: [`interceptor_tf2_odometry.cpp`](../../src/interceptor/src/interceptor_tf2_odometry.cpp)

| Line | Code | Explanation |
| ---: | --- | --- |
| 1-6 | Doxygen comment `/** ... */` | Documents the file, its purpose and author. |
| 8 | `#include <memory>` | Brings `std::unique_ptr` / `std::make_unique`. |
| 9 | `#include <sstream>` | Brings `std::ostringstream`, used to build the topic name. |
| 10 | `#include <string>` | Brings `std::string`. |
| 12 | `#include <rclcpp/rclcpp.hpp>` | Brings `Node`, `init`, `spin`, logging. |
| 13 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Brings `TransformStamped`, the message that represents a tf2 transform. |
| 14 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Brings `VehicleOdometry`, the message that comes from PX4. |
| 15 | `#include <px4_ros_com/frame_transforms.h>` | Brings the NED↔ENU conversion functions. |
| 16 | `#include <tf2_ros/transform_broadcaster.h>` | Brings `TransformBroadcaster`, the class that publishes tf2 transforms. |
| 18-21 | Class Doxygen comment | Documents what `FramePublisher` does. |
| 22 | `class FramePublisher : public rclcpp::Node` | Declares the class, inheriting from `rclcpp::Node`. |
| 23 | `{` | Opens the class body. |
| 24 | `public:` | What follows is accessible from outside. |
| 25 | `explicit FramePublisher()` | Declares the constructor without parameters. |
| 26 | `: Node("interceptor_tf2_frame_publisher")` | Calls the `Node` constructor and sets the node's visible name. |
| 27 | `{` | Opens the constructor body. |
| 28 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");` | Declares the `vehicle_name` parameter (default `"interceptor"`) and stores its value. |
| 30 | `tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);` | Creates the object that will publish tf2 transforms, tied to this node. |
| 32 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copies a QoS profile meant for sensor data. |
| 33 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Turns it into a ROS 2 `QoS` with a queue of 5 messages. |
| 35 | `std::ostringstream stream;` | Creates a "text builder" to put the topic name together. |
| 36 | `stream << "/fmu/out/vehicle_odometry";` | Writes the topic of instance 0 (no prefix, the interceptor's). |
| 37 | `std::string topic_name = stream.str();` | Turns what was written into `stream` into a normal `std::string`. |
| 39 | `subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,` | Starts creating the subscription: type `VehicleOdometry`, topic `topic_name`, quality of service `qos`. |
| 40 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda run for every received message; `[this]` lets it use the node's members; `msg` is the message. |
| 41 | `using px4_ros_com::frame_transforms::ned_to_enu_local_frame;` | Shortens the name of the position conversion function. |
| 42 | `using px4_ros_com::frame_transforms::px4_to_ros_orientation;` | Shortens the name of the orientation conversion function. |
| 44 | `// PX4 position is NED, ROS/tf2 expects ENU` | Explanatory comment. |
| 45 | `Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);` | Builds a 3D `double` vector with the message's NED position. |
| 46 | `Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);` | Converts that vector to ENU. |
| 48 | `// PX4 quaternion is (w, x, y, z), aircraft frame relative to NED` | Comment about the component order. |
| 49 | `Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);` | Builds the received quaternion, in (w, x, y, z) order. |
| 50 | `Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);` | Converts that orientation to the ROS convention. |
| 52 | `geometry_msgs::msg::TransformStamped t;` | Creates the transform message to fill in and publish. |
| 53 | `t.header.stamp = this->get_clock()->now();` | Stores the current time as the timestamp. |
| 54 | `t.header.frame_id = "map";` | Sets the parent frame: `map`. |
| 55 | `t.child_frame_id = vehicle_name_ + "/base_link";` | Sets the child frame, `interceptor/base_link` by default. |
| 57 | `t.transform.translation.x = position_enu.x();` | Copies the X (east) component of the ENU position. |
| 58 | `t.transform.translation.y = position_enu.y();` | Y component (north). |
| 59 | `t.transform.translation.z = position_enu.z();` | Z component (up). |
| 61 | `t.transform.rotation.w = q_enu.w();` | Copies the `w` component of the converted quaternion. |
| 62 | `t.transform.rotation.x = q_enu.x();` | `x` component. |
| 63 | `t.transform.rotation.y = q_enu.y();` | `y` component. |
| 64 | `t.transform.rotation.z = q_enu.z();` | `z` component. |
| 66 | `tf_broadcaster_->sendTransform(t);` | Publishes the transform to the tf2 tree; from here on `map -> interceptor/base_link` can be looked up. |
| 67 | `});` | Closes the lambda and the `create_subscription` call. |
| 68 | `}` | Closes the constructor. |
| 70 | `private:` | What follows is only accessible inside the class. |
| 71 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Stores the subscription so it stays alive. |
| 72 | `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;` | Stores the broadcaster to use it in the lambda. |
| 73 | `std::string vehicle_name_;` | Stores the vehicle name read from the parameter. |
| 74 | `};` | Closes the class. |
| 76 | `int main(int argc, char *argv[])` | Program entry point. |
| 77 | `{` | Opens `main`. |
| 78 | `std::cout << "Starting interceptor_tf2_odometry frame publisher..." << std::endl;` | Prints a start-up message. |
| 79 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Turns off standard output buffering. |
| 80 | `rclcpp::init(argc, argv);` | Initialises ROS 2. |
| 81 | `rclcpp::spin(std::make_shared<FramePublisher>());` | Creates the node and keeps it alive, running the callback of line 40. |
| 83 | `rclcpp::shutdown();` | Releases the ROS 2 resources when `spin` returns. |
| 84 | `return 0;` | Ends without errors. |
| 85 | `}` | Closes `main`. |

## 3. `target_tf2_odometry.cpp`

File: [`target_tf2_odometry.cpp`](../../src/interceptor/src/target_tf2_odometry.cpp)

It shares almost every line with `interceptor_tf2_odometry.cpp`. The table
covers the whole file; lines with no equivalent in the interceptor converter
are marked with **★**.

| Line | Code | Explanation |
| ---: | --- | --- |
| 1-13 | Doxygen comment `/** ... */` | Documents the file and its purpose; mentions that it also publishes the target's velocity and explains (lines 9-12) that the target's position is shifted by the NED offset between the target's origin and the interceptor's, so both end up expressed in the same origin. |
| 15 | `#include <memory>` | Same as in the interceptor converter. |
| 16 | `#include <sstream>` | Same. |
| 17 | `#include <string>` | Same. |
| 19 | `#include <rclcpp/rclcpp.hpp>` | Same. |
| 20 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Same. |
| 21 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Same. |
| 22 | ★ `#include <px4_msgs/msg/vehicle_local_position.hpp>` | New: brings `VehicleLocalPosition`, the message with `ref_lat`/`ref_lon`/`ref_alt`. |
| 23 | `#include <px4_ros_com/frame_transforms.h>` | Same. |
| 24 | ★ `#include <px4_ros2/utils/geodesic.hpp>` | New: brings `px4_ros2::vectorToGlobalPosition`, used to compute the NED offset between origins. |
| 25 | ★ `#include <px4_ros2/utils/message_version.hpp>` | New: brings `getMessageNameVersion`, to build the version suffix of the local position topic. |
| 26 | `#include <tf2_ros/transform_broadcaster.h>` | Same. |
| 27 | ★ `#include <geometry_msgs/msg/twist_stamped.hpp>` | Extra: brings `TwistStamped`, the type of the `target/velocity` message. |
| 29-32 | Class Doxygen comment | Same as in the other file. |
| 33 | `class FramePublisher : public rclcpp::Node` | Same class declaration (same class name, different file). |
| 34 | `{` | Opens the body. |
| 35 | `public:` | Same. |
| 36 | `explicit FramePublisher()` | Same. |
| 37 | `: Node("target_tf2_frame_publisher")` | Different node name. |
| 38 | `{` | Opens the constructor. |
| 39 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "target");` | Same parameter, default `"target"`. |
| 41 | `tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);` | Same as in the other file. |
| 43 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Same. |
| 44 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Same. |
| 46 | ★ `const std::string local_position_version_suffix =` | New: starts storing the version suffix of the `VehicleLocalPosition` message. |
| 47 | ★ `px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>();` | New: computes that suffix (something like `_v1` with the vendored `px4_msgs`) from the message type. |
| 49 | ★ `local_position_sub_interceptor_ =` | New: starts creating the subscription to the interceptor's local position. |
| 50 | ★ `this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(` | New: type of the subscribed message. |
| 51 | ★ `"/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,` | New: topic of instance 0 (interceptor), with the same QoS as the odometry. |
| 52 | ★ `[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {` | New: callback lambda. |
| 53 | ★ `ref_lat_interceptor_ = msg->ref_lat;` | New: stores the reference latitude of the interceptor's origin. |
| 54 | ★ `ref_lon_interceptor_ = msg->ref_lon;` | New: reference longitude. |
| 55 | ★ `ref_alt_interceptor_ = msg->ref_alt;` | New: reference altitude. |
| 56 | ★ `ref_valid_interceptor_ = msg->xy_global && msg->z_global;` | New: the reference only counts as valid once the EKF has a horizontal and vertical global origin. |
| 57 | ★ `});` | New: closes the lambda and `create_subscription`. |
| 59 | ★ `local_position_sub_target_ =` | New: starts creating the subscription to the target's local position. |
| 60 | ★ `this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(` | New. |
| 61 | ★ `"/px4_1/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,` | New: topic of instance 1 (target). |
| 62 | ★ `[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {` | New. |
| 63 | ★ `ref_lat_target_ = msg->ref_lat;` | New. |
| 64 | ★ `ref_lon_target_ = msg->ref_lon;` | New. |
| 65 | ★ `ref_alt_target_ = msg->ref_alt;` | New. |
| 66 | ★ `ref_valid_target_ = msg->xy_global && msg->z_global;` | New. |
| 67 | ★ `});` | New: closes the lambda and `create_subscription`. |
| 69 | `std::ostringstream stream;` | Same. |
| 70 | `stream << "/px4_1/fmu/out/vehicle_odometry";` | **Different topic**: instance 1, the target's. |
| 71 | `std::string topic_name = stream.str();` | Same. |
| 74 | ★ `velocity_pub_ =` | Starts assigning the newly created publisher to the `velocity_pub_` member. |
| 75 | ★ `this->create_publisher<geometry_msgs::msg::TwistStamped>("target/velocity", 10);` | Creates the velocity publisher on the `target/velocity` topic, with a queue of 10 messages. |
| 76 | ★ `auto timer_callback =` | Starts defining the function the timer will run. |
| 77 | ★ `[this]()->void {` | Lambda without parameters that returns `void`. |
| 78 | ★ `geometry_msgs::msg::TwistStamped msg;` | Creates the velocity message to publish. |
| 79 | ★ `msg.header.stamp = this->get_clock()->now();` | Current timestamp. |
| 80 | ★ `msg.header.frame_id = vehicle_name_ + "/base_link";` | Frame in which the velocity is expressed. |
| 81 | ★ `msg.twist.linear.x = _target_velocity_enu.x();` | Copies the X component of the last stored ENU velocity. |
| 82 | ★ `msg.twist.linear.y = _target_velocity_enu.y();` | Y component. |
| 83 | ★ `msg.twist.linear.z = _target_velocity_enu.z();` | Z component. |
| 84 | ★ `velocity_pub_->publish(msg);` | Publishes the message on `target/velocity`. |
| 85 | ★ `};` | Closes the `timer_callback` lambda. |
| 86 | ★ `timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);` | Creates a timer that runs `timer_callback` every 100 ms (10 Hz), regardless of when PX4 messages arrive. |
| 88 | `subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,` | Same pattern as in the other converter, but with instance 1's `topic_name`. |
| 89 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Same. |
| 90 | `using px4_ros_com::frame_transforms::ned_to_enu_local_frame;` | Same. |
| 91 | `using px4_ros_com::frame_transforms::px4_to_ros_orientation;` | Same. |
| 93 | ★ `// PX4 velocity is NED, ROS/tf2 expects ENU` | Comment: here the velocity is converted too (this doesn't exist in the interceptor converter). |
| 94 | ★ `Eigen::Vector3d velocity_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);` | Builds a 3D `double` vector with the message's NED velocity. |
| 95 | ★ `Eigen::Vector3d velocity_enu = ned_to_enu_local_frame(velocity_ned);` | Converts that velocity to ENU (reusing the same function as for the position). |
| 97 | ★ `// Vector3d not Vector3f, so use cast. TF2 only has translation and rotation` | Explains why the `.cast<float>()` on the next line is needed. |
| 98 | ★ `_target_velocity_enu = velocity_enu.cast<float>();` | Stores the ENU velocity as `float` (converted from `double`) in the member the timer reads; it's done before checking whether to publish, so the timer always has the latest value even if the transform isn't published yet. |
| 100 | ★ `if (!ref_valid_interceptor_ \|\| !ref_valid_target_) {` | New: if either vehicle's global reference is missing, the offset between origins can't be computed yet. |
| 101 | ★ `// Missing global reference of interceptor and/or target: the NED offset` | Comment explaining the `return`. |
| 102 | ★ `// between origins can't be computed yet, so don't publish a wrong transform` | Continues the comment. |
| 103 | ★ `std::string missing_refs;` | New: declares an empty string that collects which references are missing. |
| 104 | ★ `if (!ref_valid_interceptor_) {` | If the interceptor's reference is missing, enters this block. |
| 105 | ★ `missing_refs += " interceptor";` | Appends `" interceptor"` to the string. |
| 106 | ★ `}` | Closes the previous `if`. |
| 107 | ★ `if (!ref_valid_target_) {` | If the target's reference is missing, enters this block. |
| 108 | ★ `missing_refs += missing_refs.empty() ? " target" : " and target";` | If `missing_refs` was still empty (only the target is missing) it appends `" target"`; if it already had `" interceptor"` (both missing) it appends `" and target"`, so the message reads `of interceptor and target` and not `of interceptor target`. |
| 109 | ★ `}` | Closes the previous `if`. |
| 110 | ★ `RCLCPP_WARN_THROTTLE(` | Logs a warning, rate-limited. |
| 111 | ★ `this->get_logger(), *this->get_clock(), 5000,` | At most one warning every 5000 ms. |
| 112 | ★ `"Waiting for global reference (ref_lat/ref_lon/ref_alt) of%s before publishing "` | First part of the warning text: a single `%s`, filled with `missing_refs`. |
| 113 | ★ `"map -> %s/base_link",` | Second part of the warning text. |
| 114 | ★ `missing_refs.c_str(),` | Fills the `%s` with the string built above (`" interceptor"`, `" target"` or `" interceptor and target"`). |
| 115 | ★ `vehicle_name_.c_str());` | Vehicle name (`target`) shown at the end of the warning. |
| 116 | ★ `return;` | Doesn't publish `map -> target/base_link` while any global reference is missing. |
| 117 | ★ `}` | Closes the `if` block. |
| 119 | `// PX4 position is NED, ROS/tf2 expects ENU` | Same as in the other converter. |
| 120 | `Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);` | Same. |
| 122 | ★ `// NED offset of the target's origin with respect to the interceptor's,` | New comment. |
| 123 | ★ `// so both vehicles end up expressed in the same origin (map = interceptor's)` | Continues the comment. |
| 124 | ★ `Eigen::Vector3d global_position_interceptor(` | New: builds the interceptor's reference lat/lon/alt vector. |
| 125 | ★ `ref_lat_interceptor_, ref_lon_interceptor_, ref_alt_interceptor_);` | Continues building the vector. |
| 126 | ★ `Eigen::Vector3d global_position_target(` | New: builds the target's reference lat/lon/alt vector. |
| 127 | ★ `ref_lat_target_, ref_lon_target_, ref_alt_target_);` | Continues building the vector. |
| 128 | ★ `Eigen::Vector3f origin_offset_ned = px4_ros2::vectorToGlobalPosition(` | New: computes the NED offset of the target's origin relative to the interceptor's. |
| 129 | ★ `global_position_interceptor, global_position_target);` | Arguments: the "current" global position (interceptor) and the "next" one (target). |
| 130 | ★ `position_ned += origin_offset_ned.cast<double>();` | New: adds that offset to the target's NED position, before converting it to ENU. |
| 132 | `Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);` | Same as in the other converter, but now with the position already shifted to the interceptor's origin. |
| 134 | `// PX4 quaternion is (w, x, y, z), aircraft frame relative to NED` | Same. |
| 135 | `Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);` | Same. |
| 136 | `Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);` | Same. |
| 138 | `geometry_msgs::msg::TransformStamped t;` | Same. |
| 139 | `t.header.stamp = this->get_clock()->now();` | Same. |
| 140 | `t.header.frame_id = "map";` | Same. |
| 141 | `t.child_frame_id = vehicle_name_ + "/base_link";` | Here it resolves to `target/base_link`. |
| 143 | `t.transform.translation.x = position_enu.x();` | Same as in the other converter. |
| 144 | `t.transform.translation.y = position_enu.y();` | Same. |
| 145 | `t.transform.translation.z = position_enu.z();` | Same. |
| 147 | `t.transform.rotation.w = q_enu.w();` | Same. |
| 148 | `t.transform.rotation.x = q_enu.x();` | Same. |
| 149 | `t.transform.rotation.y = q_enu.y();` | Same. |
| 150 | `t.transform.rotation.z = q_enu.z();` | Same. |
| 152 | `tf_broadcaster_->sendTransform(t);` | Publishes `map -> target/base_link`, with the target's position already expressed in the interceptor's origin. |
| 153 | `});` | Closes the lambda and `create_subscription`. |
| 155 | `}` | Closes the constructor. |
| 157 | `private:` | Same. |
| 158 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Same. |
| 159-160 | ★ `rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr` / `local_position_sub_interceptor_;` | New: stores the subscription to the interceptor's local position. |
| 161 | ★ `rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_target_;` | New: stores the subscription to the target's local position. |
| 162 | `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;` | Same. |
| 163 | ★ `rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;` | Stores the velocity publisher. |
| 164 | ★ `Eigen::Vector3f _target_velocity_enu{Eigen::Vector3f::Zero()};` | Stores the last known ENU velocity, initialised to zero so the timer doesn't read garbage before the first PX4 message. |
| 165 | ★ `rclcpp::TimerBase::SharedPtr timer_;` | Stores the 100 ms timer. |
| 166 | `std::string vehicle_name_;` | Same. |
| 167 | ★ `double ref_lat_interceptor_{0.0};` | New: last received reference latitude of the interceptor. |
| 168 | ★ `double ref_lon_interceptor_{0.0};` | New: reference longitude of the interceptor. |
| 169 | ★ `float ref_alt_interceptor_{0.0F};` | New: reference altitude of the interceptor. |
| 170 | ★ `bool ref_valid_interceptor_{false};` | New: whether that reference is already valid. |
| 171 | ★ `double ref_lat_target_{0.0};` | New: the same for the target. |
| 172 | ★ `double ref_lon_target_{0.0};` | New. |
| 173 | ★ `float ref_alt_target_{0.0F};` | New. |
| 174 | ★ `bool ref_valid_target_{false};` | New. |
| 175 | `};` | Closes the class. |
| 177 | `int main(int argc, char *argv[])` | Same structure as the other converter. |
| 178 | `{` | Opens `main`. |
| 179 | `std::cout << "Starting target_tf2_odometry frame publisher..." << std::endl;` | Start-up message, matching the binary name (`target_tf2_odometry`). |
| 180 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Same. |
| 181 | `rclcpp::init(argc, argv);` | Same. |
| 182 | `rclcpp::spin(std::make_shared<FramePublisher>());` | Same. |
| 184 | `rclcpp::shutdown();` | Same. |
| 185 | `return 0;` | Same. |
| 186 | `}` | Closes `main`. |

The odometry callback (lines 88-153) does three things with a single input: it
stores the velocity so the timer, at a different rate (a fixed 100 ms),
publishes it separately (lines 76-86); if the global reference of the
interceptor or the target is missing, it warns and returns without publishing
(lines 100-117); and once both references are valid, it shifts the target's
position to the interceptor's origin before publishing the transform (lines
122-152). That's why there are two different "clocks" running in this same
file.

## 4. `tf2_listener.cpp`

File: [`tf2_listener.cpp`](../../src/interceptor/src/tf2_listener.cpp)

| Line | Code | Explanation |
| ---: | --- | --- |
| 1 | `#include <chrono>` | Brings durations and literals such as `1s`. |
| 2 | `#include <functional>` | Brings `std::bind`, used to tie the timer to a method. |
| 3 | `#include <memory>` | Brings `std::unique_ptr`/`std::make_unique`. |
| 4 | `#include <string>` | Brings `std::string`. |
| 6 | `#include "geometry_msgs/msg/transform_stamped.hpp"` | Brings `TransformStamped`, the type of a tf2 lookup result. |
| 7 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Brings `VehicleOdometry`. |
| 8 | `#include "rclcpp/rclcpp.hpp"` | Brings `Node`, `init`, `spin`, logging. |
| 9 | `#include "tf2/exceptions.h"` | Brings `tf2::TransformException`, the exception thrown when a frame is missing. |
| 10 | `#include "tf2_ros/transform_listener.h"` | Brings `TransformListener`, which fills the buffer by listening to tf2. |
| 11 | `#include "tf2_ros/buffer.h"` | Brings `Buffer`, where the received transforms are stored. |
| 13 | `using namespace std::chrono_literals;` | Lets you write durations like `1s` instead of building them by hand. |
| 15 | `class FrameListener : public rclcpp::Node` | Declares the class, inheriting from `rclcpp::Node`. |
| 16 | `{` | Opens the class body. |
| 17 | `public:` | What follows is accessible from outside. |
| 18 | `FrameListener()` | Declares the constructor without parameters (here without `explicit`, unlike other nodes in the package). |
| 19 | `: Node("tf2_frame_listener")` | Calls the `Node` constructor and sets the visible name. |
| 20 | `{` | Opens the constructor body. |
| 21 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copies a sensor QoS profile. |
| 22 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Turns it into a `QoS` with a queue of 5 messages. |
| 24 | `// Declare and acquire \`target_frame\` parameter` | Explanatory comment. |
| 25 | `target_frame_ = this->declare_parameter<std::string>("target_frame", "target/base_link");` | Declares the `target_frame` parameter (default `target/base_link`) and stores its value. |
| 27-28 | `tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());` | Creates the tf2 buffer, which needs the node's clock to interpret timestamps. |
| 29-30 | `tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);` | Creates the listener, which listens to tf2 in the background and fills `tf_buffer_`. |
| 32 | `// Subscribe to target odometry to keep the latest velocity` | Explanatory comment. |
| 33 | `subscription_ =` | Starts assigning the subscription. |
| 34 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry",` | Subscribes to the target's odometry (instance 1) directly, not through tf2, because tf2 carries no velocity. |
| 35 | `qos,` | Uses the quality of service created above. |
| 36 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda run for every target odometry message. |
| 37 | `target_velocity_[0] = msg->velocity[0];` | Stores the X component of the velocity, in NED, unconverted. |
| 38 | `target_velocity_[1] = msg->velocity[1];` | Y component. |
| 39 | `target_velocity_[2] = msg->velocity[2];` | Z component. |
| 40 | `});` | Closes the lambda and `create_subscription`. |
| 42 | `// Call on_timer function every second` | Explanatory comment. |
| 43-44 | `timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer, this));` | Creates a timer that calls `on_timer()` every second; `std::bind` ties the method to `this` as if it were a free function. |
| 45 | `}` | Closes the constructor. |
| 47 | `private:` | What follows is only accessible inside the class. |
| 48 | `void on_timer()` | Declares the method that runs every second. |
| 49 | `{` | Opens the method body. |
| 50-51 | `// Store frame names in variables that will be used to` + `// compute transformations` | Explanatory comment, split over two lines. |
| 52 | `std::string fromFrameRel = target_frame_.c_str();` | Source frame of the lookup: the target's. |
| 53 | `std::string toFrameRel = "interceptor/base_link";` | Destination frame: the interceptor's (fixed, not a parameter). |
| 55 | `geometry_msgs::msg::TransformStamped t;` | Variable that will hold the lookup result. |
| 57 | `try {` | Starts the block that may fail if a frame is missing. |
| 58-60 | `t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);` | Asks for the transform "target seen from the interceptor", using the latest available data (`TimePointZero`). |
| 61 | `} catch (const tf2::TransformException & ex) {` | If `lookupTransform` throws (a frame is missing), it's caught here. |
| 62-64 | `RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ...);` | Logs which transform failed and why. |
| 65 | `return;` | Leaves the method without using invalid data. |
| 66 | `}` | Closes the `catch` block. |
| 68-73 | `RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: translation (...), velocity (...)", ...);` | If the lookup worked, prints the translation and the target's last stored velocity. |
| 75 | `}` | Closes `on_timer`. |
| 77 | `rclcpp::TimerBase::SharedPtr timer_{nullptr};` | Stores the timer, explicitly initialised to null. |
| 78 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Stores the subscription to the target's odometry. |
| 79 | `std::array<float, 3> target_velocity_;` | Stores the target's last known NED velocity. |
| 80 | `std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};` | Stores the tf2 listener. |
| 81 | `std::unique_ptr<tf2_ros::Buffer> tf_buffer_;` | Stores the tf2 buffer. |
| 82 | `std::string target_frame_;` | Stores the target frame name read from the parameter. |
| 83 | `};` | Closes the class. |
| 85 | `int main(int argc, char * argv[])` | Program entry point. |
| 86 | `{` | Opens `main`. |
| 87 | `rclcpp::init(argc, argv);` | Initialises ROS 2. |
| 88 | `rclcpp::spin(std::make_shared<FrameListener>());` | Creates the node and keeps it alive: the odometry lambda and `on_timer` run here. |
| 89 | `rclcpp::shutdown();` | Releases the ROS 2 resources. |
| 90 | `return 0;` | Ends without errors. |
| 91 | `}` | Closes `main`. |

This node never writes a setpoint or calls anything from `px4_ros2`: it only
reads and displays. That's why it's safe to run alongside the guidance modes
without any risk of interfering with flight control; its only effect is
printing text to the console/logs.

That's all for the odometry and tf2 nodes. Carry on with
[Guidance modes, line by line](Line-by-line-guidance-modes.md).

---

🏠 [Home](Home.md) · ⬅️ Previous: [Line by line: project and build files](Line-by-line-project-and-build-files.md) · ➡️ Next: [Line by line: guidance modes](Line-by-line-guidance-modes.md)
