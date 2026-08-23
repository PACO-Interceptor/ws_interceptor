/**
 * @brief Proportional Navigation (PN) pursuit mode for PX4
 * @file PN_mode.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 * @details This mode implements a Proportional Navigation (PN) pursuit algorithm for a PX4-based vehicle.
 * The mode subscribes to the target's position and velocity, calculates the line-of-sight (LOS) vector,
 * and generates velocity setpoints to pursue the target while maintaining a safe distance.
 * The mode also handles arming and run conditions, ensuring that the vehicle only operates when valid target data is available.
 */

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;  // NOLINT

static const std::string kName = "PN mode";
static const std::string kMapFrame = "map";

class PN_Mode : public px4_ros2::ModeBase
{
public:
  explicit PN_Mode(rclcpp::Node & node)
  : ModeBase(node, Settings{kName})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");
    _target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});
    _target_velocity_topic = node.declare_parameter<std::string>("target_velocity_topic",
      "target/velocity");

    // Subscribe to the target's velocity topic. ENU to NED conversion is done in the callback.
    _target_velocity_sub =
      node.create_subscription<geometry_msgs::msg::TwistStamped>(_target_velocity_topic, 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          using px4_ros_com::frame_transforms::enu_to_ned_local_frame;

        // Convert the received ENU velocity to NED frame
          Eigen::Vector3d velocity_enu(msg->twist.linear.x, msg->twist.linear.y,
        msg->twist.linear.z);
          Eigen::Vector3d velocity_ned = enu_to_ned_local_frame(velocity_enu);

          _target_velocity_ned = velocity_ned.cast<float>(); // .cast to convert from double to float

          _target_velocity_valid = true;
      });

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  }

  // Safety checks
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override
  {
    if (!_target_valid || !_target_velocity_valid) {
      reporter.armingCheckFailureExt(
        px4_ros2::events::ID("pursuit_no_target"),
        px4_ros2::events::Log::Error,
        "No target odometry received yet");
    }
  }

  // PN main logic
  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;

    if (!_target_valid) {
      RCLCPP_WARN(node().get_logger(), "Target position not valid yet. Skipping setpoint update.");
      return;
    }

    const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();
    const Eigen::Vector3f v_rel = _target_velocity_ned - _own_position->velocityNed();

    if (los.norm() < 1.0f) {
      RCLCPP_INFO(node().get_logger(), "Target reached. Stopping pursuit.");
      completed(px4_ros2::Result::Success);
      return;
    }

    Eigen::Vector3f a_cmd = Eigen::Vector3f::Zero();


    if (los.norm() < kPnMinRange) {
      a_cmd = Eigen::Vector3f::Zero();
    } else {
      // LOS rotation rate ω = (los × v_rel) / (los · los)
      const Eigen::Vector3f los_rotation_rate = los.cross(v_rel) / (los.squaredNorm() + 1e-6f);
      a_cmd = kNavigationConstant * los_rotation_rate.cross(v_rel);
      const float a_cmd_norm = a_cmd.norm();

      if (a_cmd_norm > 1e-6f) { // avoid division by zero and normalize only if the norm is significant
        a_cmd = a_cmd.normalized() * std::min(a_cmd_norm, kMaxAcceleration);
      } else {
        a_cmd = Eigen::Vector3f::Zero();
      }
    }

    const Eigen::Vector2f los_horizontal(los.x(), los.y());
    Eigen::Vector2f velocity_horizontal = Eigen::Vector2f::Zero();
    if (los_horizontal.norm() > kMinHorizontalDistance) {
      velocity_horizontal = los_horizontal.normalized() * kMaxHorizontalSpeed;
      _last_yaw = atan2f(los_horizontal.y(), los_horizontal.x());
    }

    const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);

    const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};

    _trajectory_setpoint->update(velocity, a_cmd, _last_yaw);
  }

private:
  void updateTargetPosition()
  {
    geometry_msgs::msg::TransformStamped t;

    try {
      t = _tf_buffer->lookupTransform(kMapFrame, _target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 5000,
        "Could not transform %s to %s: %s", kMapFrame.c_str(), _target_frame.c_str(), ex.what());
      return;
    }

    const Eigen::Vector3d position_enu(
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    const Eigen::Vector3d position_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(
      position_enu);

    _target_position_ned = position_ned.cast<float>();
    _target_valid = true;
  }

  static constexpr float kMaxHorizontalSpeed = 7.0f;  // [m/s]
  static constexpr float kMaxVerticalSpeed = 2.0f;    // [m/s]
  static constexpr float kMinHorizontalDistance = 0.1f;  // [m]

  static constexpr float kNavigationConstant = 3.5f;  // Proportional navigation constant (N/lambda)
  static constexpr float kMaxAcceleration = 3.0f;  // [m/s^2]
  static constexpr float kPnMinRange = 7.0f;  // [m] Minimum range for PN to be active

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _own_position;

  std::string _target_frame;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  rclcpp::TimerBase::SharedPtr _target_lookup_timer;

  Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};
  Eigen::Vector3f _target_velocity_ned{Eigen::Vector3f::Zero()};
  std::string _target_velocity_topic;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _target_velocity_sub;

  bool _target_velocity_valid{false};
  bool _target_valid{false};
  float _last_yaw{0.f};
};

using PN_ModeNode = px4_ros2::NodeWithMode<PN_Mode>;

static const std::string kNodeName = "PN_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PN_ModeNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
