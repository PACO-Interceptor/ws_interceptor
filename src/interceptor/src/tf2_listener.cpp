#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("tf2_frame_listener")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "target/base_link");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to target odometry to keep the latest velocity
    subscription_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry",
      qos,
        [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
          target_velocity_[0] = msg->velocity[0];
          target_velocity_[1] = msg->velocity[1];
          target_velocity_[2] = msg->velocity[2];
        });

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "interceptor/base_link";

    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Transform from %s to %s: translation (%.2f, %.2f, %.2f), velocity (%.2f, %.2f, %.2f)",
        toFrameRel.c_str(), fromFrameRel.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
        target_velocity_[0], target_velocity_[1], target_velocity_[2]);

  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::array<float, 3> target_velocity_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
