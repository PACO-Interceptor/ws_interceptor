/**
 * @brief Vehicle Odometry to tf2 frame broadcaster
 * @file interceptor_tf2_odometry.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 */

#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @brief Broadcasts a vehicle's VehicleOdometry as a tf2 transform, converting
 * from PX4's NED/aircraft frame to ROS's ENU/base_link frame
 */
class FramePublisher : public rclcpp::Node
{
public:
  explicit FramePublisher()
  : Node("interceptor_tf2_frame_publisher")
  {
    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    std::ostringstream stream;
    stream << "/fmu/out/vehicle_odometry";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,
        [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
          using px4_ros_com::frame_transforms::ned_to_enu_local_frame;
          using px4_ros_com::frame_transforms::px4_to_ros_orientation;

                        // PX4 position is NED, ROS/tf2 expects ENU
          Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);
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
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string vehicle_name_;
};

int main(int argc, char *argv[])
{
  std::cout << "Starting interceptor_tf2_odometry frame publisher..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());

  rclcpp::shutdown();
  return 0;
}
