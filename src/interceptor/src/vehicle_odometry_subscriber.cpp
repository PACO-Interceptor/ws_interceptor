/**
 * @brief Vehicle Odometry uORB topic listener example
 * @file vehicle_odometry_subscriber.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 * @details A single executable serves both vehicles, chosen via the vehicle_name/odometry_topic
 *		parameters at launch.
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

/**
 * @brief Vehicle Odometry uORB topic data callback
 */
class VehicleOdometrySubscriber : public rclcpp::Node
{
public:
  explicit VehicleOdometrySubscriber()
  : Node("vehicle_odometry_subscriber")
  {
    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");
    std::string odometry_topic =
      this->declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    subscription_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>(odometry_topic, qos,
        [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
          std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
          std::cout << "RECEIVED VEHICLE ODOMETRY DATA (" << vehicle_name_ << ")" << std::endl;
          std::cout << "Timestamp: " << msg->timestamp << std::endl;
          std::cout << "Pose frame: " << msg->pose_frame << std::endl;
          std::cout << "X Position: " << msg->position[0] << std::endl;
          std::cout << "Y Position: " << msg->position[1] << std::endl;
          std::cout << "Z Position: " << msg->position[2] << std::endl;
          std::cout << "q[0]: " << msg->q[0] << std::endl;
          std::cout << "q[1]: " << msg->q[1] << std::endl;
          std::cout << "q[2]: " << msg->q[2] << std::endl;
          std::cout << "q[3]: " << msg->q[3] << std::endl;
          std::cout << "X Velocity: " << msg->velocity[0] << std::endl;
          std::cout << "Y Velocity: " << msg->velocity[1] << std::endl;
          std::cout << "Z Velocity: " << msg->velocity[2] << std::endl;
          std::cout << "Velocity magnitude: " <<
            sqrt(msg->velocity[0] * msg->velocity[0] + msg->velocity[1] * msg->velocity[1] +
        msg->velocity[2] * msg->velocity[2]) << std::endl;
                });
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  std::string vehicle_name_;

};

int main(int argc, char *argv[])
{
  std::cout << "Starting vehicle_odometry_subscriber node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleOdometrySubscriber>());

  rclcpp::shutdown();
  return 0;
}
