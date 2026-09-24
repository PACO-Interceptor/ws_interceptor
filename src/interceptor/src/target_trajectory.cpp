/**
 * @brief Genera y publica una trayectoria (circulo o recta) para el objetivo en SITL.
 * @file target_trajectory.cpp
 * @addtogroup interceptor
 * @author David Rodriguez <david.rodriguez.elbahri@uvigo.gal>
 * @details Publica OffboardControlMode y TrajectorySetpoint en los topics de PX4,
 *          y envia comandos para armar y poner en modo offboard al dron objetivo.
 *          Con trajectory_type = "circle" (defecto) da vueltas a un circulo; con
 *          "line" recorre una recta a velocidad constante y se para al final, que es
 *          el movimiento que supone el estimador visual (velocidad constante).
 *
 * AVISO DE DISENO: mientras este nodo este en ejecucion, no es posible tomar
 * el control del objetivo desde QGC ni desde ninguna otra fuente externa. Si
 * el objetivo sale del modo offboard o se desarma por cualquier motivo, el nodo
 * lo volvera a llevar a offboard+armado en un maximo de 2 s. Para pilotarlo a
 * mano hay que parar este nodo.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * @brief Nodo que controla la trayectoria del objetivo mediante OffboardControlMode
 *        y TrajectorySetpoint.
 */
class TargetTrajectory : public rclcpp::Node
{
public:
  explicit TargetTrajectory()
  : Node("target_trajectory")
  {
    px4_namespace_ = this->declare_parameter<std::string>("px4_namespace", "/px4_1");
    target_system_ = this->declare_parameter<int>("target_system", 2);
    altitude_m_ = this->declare_parameter<double>("altitude_m", 15.0);
    circle_radius_m_ = this->declare_parameter<double>("circle_radius_m", 25.0);
    circle_speed_mps_ = this->declare_parameter<double>("circle_speed_mps", 3.0);
    center_north_m_ = this->declare_parameter<double>("center_north_m", 0.0);
    center_east_m_ = this->declare_parameter<double>("center_east_m", 0.0);
    trajectory_type_ = this->declare_parameter<std::string>("trajectory_type", "circle");
    line_heading_deg_ = this->declare_parameter<double>("line_heading_deg", 90.0);
    line_speed_mps_ = this->declare_parameter<double>("line_speed_mps", 1.0);
    line_length_m_ = this->declare_parameter<double>("line_length_m", 30.0);
    line_start_delay_s_ = this->declare_parameter<double>("line_start_delay_s", 5.0);
    // Perfil "maneuver": como "line" (mismo inicio, espera y longitud recorrida), pero la
    // velocidad oscila +-maneuver_speed_amp (relativa) con periodo maneuver_speed_period_s y
    // el rumbo +-maneuver_turn_deg con periodo maneuver_turn_period_s. Sirve para probar el
    // checkpoint con aceleraciones y cambios de direccion, que el filtro no modela.
    maneuver_speed_amp_ = this->declare_parameter<double>("maneuver_speed_amp", 0.5);
    maneuver_speed_period_s_ = this->declare_parameter<double>("maneuver_speed_period_s", 8.0);
    maneuver_turn_deg_ = this->declare_parameter<double>("maneuver_turn_deg", 45.0);
    maneuver_turn_period_s_ = this->declare_parameter<double>("maneuver_turn_period_s", 10.0);
    // Si maneuver_max_accel_mps2 > 0, los periodos se calculan a partir de ella en lugar de
    // usar los de arriba: cada componente (tangencial por el cambio de velocidad y normal por
    // el giro) llega como mucho a max_accel/sqrt(2), asi que la aceleracion total no pasa de
    // max_accel. Los periodos no bajan de maneuver_min_period_s para que a poca velocidad no
    // se convierta en una vibracion.
    maneuver_max_accel_mps2_ = this->declare_parameter<double>("maneuver_max_accel_mps2", 0.0);
    maneuver_min_period_s_ = this->declare_parameter<double>("maneuver_min_period_s", 2.0);
    if (maneuver_max_accel_mps2_ > 0.0) {
      const double component = maneuver_max_accel_mps2_ / std::sqrt(2.0);
      const double two_pi = 2.0 * M_PI;
      // Tangencial: v0 * amp * 2pi/T. Normal: v_max * giro * 2pi/T, con v_max = v0 (1 + amp).
      maneuver_speed_period_s_ = std::max(
        maneuver_min_period_s_,
        two_pi * line_speed_mps_ * maneuver_speed_amp_ / component);
      maneuver_turn_period_s_ = std::max(
        maneuver_min_period_s_,
        two_pi * line_speed_mps_ * (1.0 + maneuver_speed_amp_) *
        maneuver_turn_deg_ * M_PI / 180.0 / component);
    }
    if (trajectory_type_ != "circle" && trajectory_type_ != "line" &&
      trajectory_type_ != "maneuver")
    {
      throw std::invalid_argument(
              "trajectory_type debe ser \"circle\", \"line\" o \"maneuver\"");
    }

    std::string ns = px4_namespace_;
    if (!ns.empty() && ns.front() != '/') {
      ns = "/" + ns;
    }
    while (!ns.empty() && ns.back() == '/') {
      ns.pop_back();
    }

    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      ns + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      ns + "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      ns + "/fmu/in/vehicle_command", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      ns + "/fmu/out/vehicle_odometry", qos,
      [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
        altitude_now_m_ = -msg->position[2];
        if (!takeoff_xy_valid_) {
          takeoff_north_m_ = msg->position[0];
          takeoff_east_m_ = msg->position[1];
          takeoff_xy_valid_ = true;
        }
        if (phase_ == Phase::CLIMB) {
          // En modo line tambien tiene que haber llegado al inicio de la recta: si ya
          // estaba en el aire en otro sitio, la recta no empieza hasta llegar.
          const double dn = msg->position[0] - center_north_m_;
          const double de = msg->position[1] - center_east_m_;
          const bool at_start = trajectory_type_ == "circle" || std::hypot(dn, de) < 1.0;
          if (at_start && std::fabs(msg->position[2] - static_cast<float>(-altitude_m_)) < 1.0f) {
            phase_ = (trajectory_type_ == "circle") ? Phase::CIRCLE : Phase::LINE;
            t0_ = this->get_clock()->now();
            RCLCPP_INFO(
              this->get_logger(),
              "Cambio de fase a %s a una altura de %.2f m",
              phase_ == Phase::LINE ? "LINE" : "CIRCLE",
              -msg->position[2]);
          }
        }
      });

    // Suscripcion al estado de control del vehiculo objetivo para saber si ya
    // esta armado y en offboard. El reintento se basa en estas dos banderas.
    vehicle_control_mode_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        ns + "/fmu/out/vehicle_control_mode", qos,
      [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
        flag_armed_ = msg->flag_armed;
        flag_control_offboard_enabled_ = msg->flag_control_offboard_enabled;
        });

    tick_count_ = 0;
    arm_attempt_count_ = 0;
    phase_ = Phase::CLIMB;
    t0_ = this->get_clock()->now();
    last_arm_attempt_ = this->get_clock()->now() - rclcpp::Duration(10, 0);  // fuerza primer intento

    timer_ = this->create_wall_timer(
      100ms,
      [this]() -> void {
        this->timer_callback();
      });
  }

private:
  enum class Phase
  {
    CLIMB,
    CIRCLE,
    LINE
  };

  void timer_callback()
  {
    // 1. Publicar siempre OffboardControlMode con position = true y todo lo demas en false
    publish_offboard_control_mode();

    // 2. Publicar siempre TrajectorySetpoint dictado por la fase
    publish_trajectory_setpoint();

    // 3. Mandar offboard+arm si no estamos ya en ese estado.
    //    El primer intento no va antes del tick 10: PX4 exige recibir
    //    setpoints antes de aceptar el modo offboard.
    //    Los reintentos van cada 2.0 s a partir del primer intento.
    if (tick_count_ >= 10) {
      if (flag_armed_ && flag_control_offboard_enabled_) {
        // Ya en offboard y armado; registrar exito la primera vez que se llega
        if (arm_attempt_count_ > 0 && !arm_success_logged_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Objetivo en offboard y armado tras %u intento(s).",
            arm_attempt_count_);
          arm_success_logged_ = true;
        }
      } else {
        // Todavia no en offboard o no armado: reintentar si han pasado >= 2.0 s
        const double elapsed =
          (this->get_clock()->now() - last_arm_attempt_).seconds();
        if (elapsed >= 2.0) {
          arm_attempt_count_++;
          RCLCPP_INFO(
            this->get_logger(),
            "Intento %u: enviando DO_SET_MODE (offboard) y ARM.",
            arm_attempt_count_);
          publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          last_arm_attempt_ = this->get_clock()->now();
        }
      }
    }

    if (tick_count_ < std::numeric_limits<uint64_t>::max()) {
      tick_count_++;
    }
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (phase_ == Phase::CLIMB) {
      // Hasta media altitud sube en vertical sobre el punto de despegue: ir en diagonal
      // desde el suelo hacia un inicio lejano puede volcar el dron.
      const bool low = takeoff_xy_valid_ && altitude_now_m_ < 0.5 * altitude_m_;
      msg.position[0] = static_cast<float>(low ? takeoff_north_m_ : center_north_m_);
      msg.position[1] = static_cast<float>(low ? takeoff_east_m_ : center_east_m_);
      msg.position[2] = static_cast<float>(-altitude_m_);
      msg.yaw = 0.0f;
    } else if (phase_ == Phase::LINE && trajectory_type_ == "maneuver") {
      publish_maneuver_setpoint(msg);
    } else if (phase_ == Phase::LINE) {
      // Espera line_start_delay_s en el inicio y luego avanza a velocidad constante hasta
      // recorrer line_length_m. La velocidad va tambien como feedforward para que PX4
      // la siga sin el retraso de perseguir solo la posicion.
      const double t = (this->get_clock()->now() - t0_).seconds() - line_start_delay_s_;
      const double travel_time = (line_speed_mps_ > 1e-6) ? line_length_m_ / line_speed_mps_ : 0.0;
      const bool moving = t > 0.0 && t < travel_time;
      const double s = std::clamp(t, 0.0, travel_time) * line_speed_mps_;
      const double heading = line_heading_deg_ * M_PI / 180.0;
      msg.position[0] = static_cast<float>(center_north_m_ + s * std::cos(heading));
      msg.position[1] = static_cast<float>(center_east_m_ + s * std::sin(heading));
      msg.position[2] = static_cast<float>(-altitude_m_);
      const double v = moving ? line_speed_mps_ : 0.0;
      msg.velocity[0] = static_cast<float>(v * std::cos(heading));
      msg.velocity[1] = static_cast<float>(v * std::sin(heading));
      msg.velocity[2] = 0.0f;
      msg.yaw = static_cast<float>(std::atan2(std::sin(heading), std::cos(heading)));
    } else {
      double t = (this->get_clock()->now() - t0_).seconds();
      double omega = (circle_radius_m_ > 1e-6) ? (circle_speed_mps_ / circle_radius_m_) : 0.0;
      double angle = omega * t;
      msg.position[0] = static_cast<float>(center_north_m_ + circle_radius_m_ * std::cos(angle));
      msg.position[1] = static_cast<float>(center_east_m_ + circle_radius_m_ * std::sin(angle));
      msg.position[2] = static_cast<float>(-altitude_m_);
      double raw_yaw = angle + M_PI / 2.0;
      // Normalizar al rango [-PI, +PI]
      msg.yaw = static_cast<float>(std::atan2(std::sin(raw_yaw), std::cos(raw_yaw)));
    }

    trajectory_setpoint_pub_->publish(msg);
  }

  // Integra el perfil "maneuver" desde la ultima llamada y rellena posicion, velocidad y yaw.
  void publish_maneuver_setpoint(px4_msgs::msg::TrajectorySetpoint & msg)
  {
    const double t = (this->get_clock()->now() - t0_).seconds() - line_start_delay_s_;
    const double base_heading = line_heading_deg_ * M_PI / 180.0;
    double speed = 0.0;
    double heading = base_heading;
    if (t > 0.0 && maneuver_travelled_m_ < line_length_m_) {
      const double two_pi = 2.0 * M_PI;
      speed = line_speed_mps_ *
        (1.0 + maneuver_speed_amp_ * std::sin(two_pi * t / maneuver_speed_period_s_));
      heading = base_heading + maneuver_turn_deg_ * M_PI / 180.0 *
        std::sin(two_pi * t / maneuver_turn_period_s_);
      const double dt = maneuver_last_t_ < 0.0 ? 0.0 : std::max(t - maneuver_last_t_, 0.0);
      maneuver_north_m_ += speed * std::cos(heading) * dt;
      maneuver_east_m_ += speed * std::sin(heading) * dt;
      maneuver_travelled_m_ += speed * dt;
      maneuver_last_t_ = t;
    }
    msg.position[0] = static_cast<float>(center_north_m_ + maneuver_north_m_);
    msg.position[1] = static_cast<float>(center_east_m_ + maneuver_east_m_);
    msg.position[2] = static_cast<float>(-altitude_m_);
    msg.velocity[0] = static_cast<float>(speed * std::cos(heading));
    msg.velocity[1] = static_cast<float>(speed * std::sin(heading));
    msg.velocity[2] = 0.0f;
    msg.yaw = static_cast<float>(std::atan2(std::sin(heading), std::cos(heading)));
  }

  void publish_vehicle_command(
    uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  std::string px4_namespace_;
  int target_system_;
  double altitude_m_;
  double circle_radius_m_;
  double circle_speed_mps_;
  double center_north_m_;
  double center_east_m_;
  std::string trajectory_type_;
  double line_heading_deg_;
  double line_speed_mps_;
  double line_length_m_;
  double line_start_delay_s_;
  double maneuver_speed_amp_;
  double maneuver_speed_period_s_;
  double maneuver_turn_deg_;
  double maneuver_turn_period_s_;
  double maneuver_max_accel_mps2_;
  double maneuver_min_period_s_;
  double maneuver_north_m_{0.0};
  double maneuver_east_m_{0.0};
  double maneuver_travelled_m_{0.0};
  double maneuver_last_t_{-1.0};

  uint64_t tick_count_{0};
  Phase phase_{Phase::CLIMB};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};

  // Estado de control del vehiculo objetivo
  bool flag_armed_{false};

  // Punto de despegue y altitud actual, para subir en vertical al principio.
  bool takeoff_xy_valid_{false};
  double takeoff_north_m_{0.0};
  double takeoff_east_m_{0.0};
  double altitude_now_m_{0.0};
  bool flag_control_offboard_enabled_{false};

  // Control de reintentos de offboard+arm
  unsigned int arm_attempt_count_{0};
  bool arm_success_logged_{false};
  rclcpp::Time last_arm_attempt_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetTrajectory>());
  rclcpp::shutdown();
  return 0;
}
