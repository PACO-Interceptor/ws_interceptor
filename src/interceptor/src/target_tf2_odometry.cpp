/**
 * @brief Vehicle Odometry to tf2 frame broadcaster
 * @file target_tf2_odometry.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 * @details The process consists of subscribing to a VehicleOdometry message and broadcasting it as a
 *			tf2 transform, converting from PX4's NED/aircraft frame to ROS's ENU/base_link frame.
 *		Additionally, it publishes the target velocity in the ENU frame.
 *		Since each PX4 measures its local position from its own startup origin, the target's
 *		local position is shifted by the NED offset between the target's and the interceptor's
 *		global reference (ref_lat/ref_lon/ref_alt), so that both end up expressed with a common
 *		origin: the interceptor's.
 */

#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <px4_ros2/utils/geodesic.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * @brief Broadcasts a vehicle's VehicleOdometry as a tf2 transform, converting
 * from PX4's NED/aircraft frame to ROS's ENU/base_link frame
 */
class FramePublisher : public rclcpp::Node
{
public:
  explicit FramePublisher()
  : Node("target_tf2_frame_publisher")
  {
    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "target");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    const std::string local_position_version_suffix =
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>();

    local_position_sub_interceptor_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        ref_lat_interceptor_ = msg->ref_lat;
        ref_lon_interceptor_ = msg->ref_lon;
        ref_alt_interceptor_ = msg->ref_alt;
        ref_valid_interceptor_ = msg->xy_global && msg->z_global;
      });

    local_position_sub_target_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/px4_1/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        ref_lat_target_ = msg->ref_lat;
        ref_lon_target_ = msg->ref_lon;
        ref_alt_target_ = msg->ref_alt;
        ref_valid_target_ = msg->xy_global && msg->z_global;
      });

    std::ostringstream stream;
    stream << "/px4_1/fmu/out/vehicle_odometry";
    std::string topic_name = stream.str();


    velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("target/velocity", 10);
    auto timer_callback =
      [this]()->void {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = vehicle_name_ + "/base_link";
        msg.twist.linear.x = _target_velocity_enu.x();
        msg.twist.linear.y = _target_velocity_enu.y();
        msg.twist.linear.z = _target_velocity_enu.z();
        velocity_pub_->publish(msg);
      };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,
        [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
          using px4_ros_com::frame_transforms::ned_to_enu_local_frame;
          using px4_ros_com::frame_transforms::px4_to_ros_orientation;

                        // PX4 velocity is NED, ROS/tf2 expects ENU
          Eigen::Vector3d velocity_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
          Eigen::Vector3d velocity_enu = ned_to_enu_local_frame(velocity_ned);

                        // Vector3d not Vector3f, so use cast. TF2 only has translation and rotation
          _target_velocity_enu = velocity_enu.cast<float>();

          if (!ref_valid_interceptor_ || !ref_valid_target_) {
                        // Missing global reference of interceptor and/or target: the NED offset
                        // between origins can't be computed yet, so don't publish a wrong transform
            std::string missing_refs;
            if (!ref_valid_interceptor_) {
              missing_refs += " interceptor";
            }
            if (!ref_valid_target_) {
              missing_refs += missing_refs.empty() ? " target" : " and target";
            }
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "Waiting for global reference (ref_lat/ref_lon/ref_alt) of%s before publishing "
              "map -> %s/base_link",
              missing_refs.c_str(),
              vehicle_name_.c_str());
            return;
          }

                        // PX4 position is NED, ROS/tf2 expects ENU
          Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);

                        // NED offset of the target's origin with respect to the interceptor's,
                        // so both vehicles end up expressed in the same origin (map = interceptor's)
          Eigen::Vector3d global_position_interceptor(
            ref_lat_interceptor_, ref_lon_interceptor_, ref_alt_interceptor_);
          Eigen::Vector3d global_position_target(
            ref_lat_target_, ref_lon_target_, ref_alt_target_);
          Eigen::Vector3f origin_offset_ned = px4_ros2::vectorToGlobalPosition(
            global_position_interceptor, global_position_target);
          position_ned += origin_offset_ned.cast<double>();

          Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);

                        // PX4 quaternion is (w, x, y, z), aircraft frame relative to NED
          Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
          Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);

          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "map";
          t.child_frame_id = vehicle_name_ + "/base_link";

          t.transform.translation.x = position_enu.x();
          t.transform.translation.y = position_enu.y();
          t.transform.translation.z = position_enu.z();

          t.transform.rotation.w = q_enu.w();
          t.transform.rotation.x = q_enu.x();
          t.transform.rotation.y = q_enu.y();
          t.transform.rotation.z = q_enu.z();

          tf_broadcaster_->sendTransform(t);
                });

  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_position_sub_interceptor_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_target_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  Eigen::Vector3f _target_velocity_enu{Eigen::Vector3f::Zero()};
  rclcpp::TimerBase::SharedPtr timer_;
  std::string vehicle_name_;
  double ref_lat_interceptor_{0.0};
  double ref_lon_interceptor_{0.0};
  float ref_alt_interceptor_{0.0F};
  bool ref_valid_interceptor_{false};
  double ref_lat_target_{0.0};
  double ref_lon_target_{0.0};
  float ref_alt_target_{0.0F};
  bool ref_valid_target_{false};
};

int main(int argc, char *argv[])
{
  std::cout << "Starting target_tf2_odometry frame publisher..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());

  rclcpp::shutdown();
  return 0;
}
