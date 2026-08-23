/**
 * @brief Vehicle Odometry uORB topic listener example
 * @file interceptor_vehicle_odometry_subscriber.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

/**
 * @brief Vehicle Odometry uORB topic data callback
 */
class VehicleOdometrySubscriber : public rclcpp::Node
{
public:
	explicit VehicleOdometrySubscriber() : Node("interceptor_vehicle_odometry_subscriber")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("px4_1/fmu/out/vehicle_odometry", qos,
		[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE ODOMETRY DATA"   << std::endl;
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
            std::cout << "Velocity magnitude: " << sqrt(msg->velocity[0]*msg->velocity[0] + msg->velocity[1]*msg->velocity[1] + msg->velocity[2]*msg->velocity[2]) << std::endl;
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;

};

int main(int argc, char *argv[])
{
	std::cout << "Starting interceptor_vehicle_odometry subscriber node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleOdometrySubscriber>());

	rclcpp::shutdown();
	return 0;
}