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

using namespace std::chrono_literals;  // NOLINT

static const std::string kName = "Pursuit Intercept";
static const std::string kMapFrame = "map";

class PursuitMode : public px4_ros2::ModeBase
{
public:
  explicit PursuitMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kName})
  {
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});
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

  static constexpr float kMaxHorizontalSpeed = 5.0f;  // [m/s]
  static constexpr float kMaxVerticalSpeed = 2.0f;    // [m/s]
  static constexpr float kMinHorizontalDistance = 0.1f;  // [m]

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _own_position;

  std::string _target_frame;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  rclcpp::TimerBase::SharedPtr _target_lookup_timer;

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
