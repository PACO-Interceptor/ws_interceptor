#include <Eigen/Eigen>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;  // NOLINT

static const std::string kName = "Pursuit Intercept";

class PursuitMode : public px4_ros2::ModeBase
{
public:
  explicit PursuitMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kName})
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _target_sub = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/px4_1/fmu/out/vehicle_odometry", qos,
      [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
        _target_position_ned = {msg->position[0], msg->position[1], msg->position[2]};
        _target_valid = true;
      });
  }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override
  {
    if (!_target_valid) {
      reporter.armingCheckFailureExt(
        px4_ros2::events::ID("pursuit_no_target"),
        px4_ros2::events::Log::Error,
        "No target odometry received yet");
    }
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;

    if (!_target_valid) {
      RCLCPP_WARN(node().get_logger(), "Target position not valid yet. Skipping setpoint update.");
      return;
    }

    const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();

    if (los.norm() < 1.0f) {
      RCLCPP_INFO(node().get_logger(), "Target reached. Stopping pursuit.");
      completed(px4_ros2::Result::Success);
      return;
    }

    const Eigen::Vector2f los_horizontal(los.x(), los.y());
    Eigen::Vector2f velocity_horizontal = Eigen::Vector2f::Zero();
    if (los_horizontal.norm() > kMinHorizontalDistance) {
      velocity_horizontal = los_horizontal.normalized() * kMaxHorizontalSpeed;
      _last_yaw = atan2f(los_horizontal.y(), los_horizontal.x());
    }

    const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);

    const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};

    _trajectory_setpoint->update(velocity, {}, _last_yaw);
  }

private:
  static constexpr float kMaxHorizontalSpeed = 5.0f;  // [m/s]
  static constexpr float kMaxVerticalSpeed = 2.0f;    // [m/s]
  static constexpr float kMinHorizontalDistance = 0.1f;  // [m]

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _own_position;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _target_sub;

  Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};
  bool _target_valid{false};
  float _last_yaw{0.f};
};

using PursuitModeNode = px4_ros2::NodeWithMode<PursuitMode>;

static const std::string kNodeName = "pursuit_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PursuitModeNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
