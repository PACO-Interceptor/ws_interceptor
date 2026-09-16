# Análisis línea por línea: nodos de odometría y tf2

Esta es la segunda de tres páginas que cubren, línea por línea, todos los
archivos propios de `src/interceptor`. Empieza por
[Archivos de proyecto y construcción](Line-by-line-project-and-build-files.md) si aún no
la has leído: ahí están el glosario de sintaxis de C++ (`override`, `explicit`,
`constexpr`, plantillas, referencias, `.cross()`/`.normalized()`...) y las
notas de "cómo usar este análisis" que también aplican aquí. La tercera página
es [Modos de guiado, línea por línea](Line-by-line-guidance-modes.md).

Esta página cubre los dos suscriptores de diagnóstico, los dos conversores
tf2 (`interceptor_tf2_odometry`, `target_tf2_odometry`) y `tf2_listener` — los
nodos que reciben la odometría de PX4, la convierten y la relacionan mediante
tf2, sin calcular ningún setpoint de vuelo.

## 1. Suscriptores de odometría para diagnóstico

Archivos:

- [`target_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/target_vehicle_odometry_subscriber.cpp)
- [`interceptor_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/interceptor_vehicle_odometry_subscriber.cpp)

Son casi idénticos: solo cambian el tópico al que se suscriben, el nombre del
nodo, los textos impresos y cómo se reparte esa suscripción entre líneas (ver
más abajo). La tabla siguiente cubre línea por línea
`target_vehicle_odometry_subscriber.cpp` (60 líneas); justo después, una
segunda tabla cubre las líneas de `interceptor_vehicle_odometry_subscriber.cpp`
(61 líneas) que son distintas.

### 1.1. `target_vehicle_odometry_subscriber.cpp`, línea por línea

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1-6 | Comentario Doxygen `/** ... */` | Documenta archivo, propósito (`Vehicle Odometry uORB topic listener example`) y autor; no se ejecuta. |
| 8 | `#include <rclcpp/rclcpp.hpp>` | Trae `Node`, `init`, `spin`, QoS y logging. |
| 9 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Trae el tipo `VehicleOdometry`. |
| 11-13 | Comentario `/** @brief ... */` | Documenta la clase que sigue. |
| 14 | `class VehicleOdometrySubscriber : public rclcpp::Node` | Declara la clase, heredando de `rclcpp::Node`. |
| 15 | `{` | Abre el cuerpo de la clase. |
| 16 | `public:` | Lo siguiente es accesible desde fuera de la clase. |
| 17 | `explicit VehicleOdometrySubscriber()` | Declara el constructor sin parámetros; `explicit` evita conversiones implícitas. |
| 18 | `: Node("target_vehicle_odometry_subscriber")` | Llama al constructor de `Node`, fijando este nombre visible (aunque, como se explica más abajo, escucha la odometría del interceptor). |
| 19 | `{` | Abre el cuerpo del constructor. |
| 20 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copia un perfil de QoS pensado para datos de sensores. |
| 21 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Convierte ese perfil en un `QoS` de ROS 2 con cola de 5 mensajes. |
| 23 | `subscription_ =` | Empieza a asignar el resultado de crear la suscripción al atributo `subscription_`. |
| 24 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,` | Crea la suscripción al tópico `/fmu/out/vehicle_odometry` (instancia 0, la del interceptor) con la calidad de servicio `qos`. |
| 25 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda que se ejecuta con cada mensaje recibido; `msg` es ese mensaje. |
| 26 | `std::cout << "\n\n\n...";` | Imprime muchas líneas en blanco para "limpiar" visualmente la consola antes de cada muestra. |
| 27 | `std::cout << "RECEIVED VEHICLE ODOMETRY DATA" << std::endl;` | Imprime un encabezado. |
| 28 | `std::cout << "Timestamp: " << msg->timestamp << std::endl;` | Imprime la marca de tiempo del mensaje PX4. |
| 29 | `std::cout << "Pose frame: " << msg->pose_frame << std::endl;` | Imprime el identificador de frame que PX4 asigna a la pose. |
| 30 | `std::cout << "X Position: " << msg->position[0] << std::endl;` | Imprime la posición X (NED) tal cual la envía PX4, sin convertir. |
| 31 | `std::cout << "Y Position: " << msg->position[1] << std::endl;` | Posición Y (NED). |
| 32 | `std::cout << "Z Position: " << msg->position[2] << std::endl;` | Posición Z (NED). |
| 33 | `std::cout << "q[0]: " << msg->q[0] << std::endl;` | Primer componente del cuaternión de orientación (`w`). |
| 34 | `std::cout << "q[1]: " << msg->q[1] << std::endl;` | Segundo componente (`x`). |
| 35 | `std::cout << "q[2]: " << msg->q[2] << std::endl;` | Tercer componente (`y`). |
| 36 | `std::cout << "q[3]: " << msg->q[3] << std::endl;` | Cuarto componente (`z`). |
| 37 | `std::cout << "X Velocity: " << msg->velocity[0] << std::endl;` | Velocidad X (NED). |
| 38 | `std::cout << "Y Velocity: " << msg->velocity[1] << std::endl;` | Velocidad Y (NED). |
| 39 | `std::cout << "Z Velocity: " << msg->velocity[2] << std::endl;` | Velocidad Z (NED). |
| 40-42 | `std::cout << "Velocity magnitude: " << sqrt(vx*vx + vy*vy + vz*vz) << std::endl;` | Calcula e imprime la longitud del vector velocidad (raíz cuadrada de la suma de cuadrados). |
| 43 | `});` | Cierra la lambda y la llamada a `create_subscription`. |
| 44 | `}` | Cierra el constructor. |
| 46 | `private:` | Lo siguiente solo es accesible dentro de la clase. |
| 47 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Guarda la suscripción; si no se guardara aquí, se destruiría al salir del constructor y dejaría de recibir mensajes. |
| 49 | `};` | Cierra la clase. |
| 51 | `int main(int argc, char *argv[])` | Punto de entrada del programa. |
| 52 | `{` | Abre `main`. |
| 53 | `std::cout << "Starting target_vehicle_odometry listener node..." << std::endl;` | Imprime un mensaje de arranque. |
| 54 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Desactiva el buffering de la salida estándar para que los `std::cout` aparezcan de inmediato. |
| 55 | `rclcpp::init(argc, argv);` | Inicializa ROS 2. |
| 56 | `rclcpp::spin(std::make_shared<VehicleOdometrySubscriber>());` | Crea el nodo y lo mantiene vivo procesando la callback de la línea 25 mientras el proceso corre. |
| 58 | `rclcpp::shutdown();` | Libera los recursos de ROS 2 cuando `spin` termina. |
| 59 | `return 0;` | Indica al sistema operativo que el programa terminó sin errores. |
| 60 | `}` | Cierra `main`. |

### 1.2. `interceptor_vehicle_odometry_subscriber.cpp`: solo lo que cambia

El resto de líneas son idénticas letra por letra a la tabla anterior, con
cuatro diferencias: tres de contenido y una de formato que además desplaza en
una línea todo lo que viene después (por eso este archivo tiene 61 líneas en
vez de 60):

| Línea | Código | Explicación |
| ---: | --- | --- |
| 3 | `@file interceptor_vehicle_odometry_subscriber.cpp` | El comentario Doxygen indica este nombre de archivo en vez del otro. |
| 18 | `: Node("interceptor_vehicle_odometry_subscriber")` | El nodo se registra con este nombre visible. |
| 24 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>("px4_1/fmu/out/vehicle_odometry",` | **Este es el detalle clave**: escucha `px4_1/fmu/out/vehicle_odometry`, es decir, la instancia **1**, la del target, no la del interceptor. |
| 25 | `qos,` | Aquí `qos` está en su propia línea; en `target_vehicle_odometry_subscriber.cpp` comparte la línea 24 con el tópico. Es la diferencia de formato: por ella este archivo suma una línea más y todo lo que sigue queda desplazado una posición respecto a la tabla 1.1. |
| 54 | `std::cout << "Starting interceptor_vehicle_odometry subscriber node..." << std::endl;` | Mensaje de arranque con este nombre; es la misma línea que la 53 de la tabla 1.1, desplazada una posición por el motivo anterior. |

### Por qué esto importa

El archivo llamado `target_vehicle_odometry_subscriber` escucha en realidad la
odometría del **interceptor** (instancia 0, línea 24 de la tabla 1.1), y el
archivo llamado `interceptor_vehicle_odometry_subscriber` escucha la del
**target** (instancia 1, línea 24 de la tabla 1.2). Los nombres de archivo
están invertidos respecto al tópico real. Esto se documenta deliberadamente
para que nadie use el nombre del ejecutable como prueba de qué vehículo está
diagnosticando: hay que mirar el tópico, no el nombre.

## 2. `interceptor_tf2_odometry.cpp`

Archivo: [`interceptor_tf2_odometry.cpp`](../../src/interceptor/src/interceptor_tf2_odometry.cpp)

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1-6 | Comentario Doxygen `/** ... */` | Documenta archivo, propósito y autor. |
| 8 | `#include <memory>` | Trae `std::unique_ptr` / `std::make_unique`. |
| 9 | `#include <sstream>` | Trae `std::ostringstream`, usado para construir el nombre del tópico. |
| 10 | `#include <string>` | Trae `std::string`. |
| 12 | `#include <rclcpp/rclcpp.hpp>` | Trae `Node`, `init`, `spin`, logging. |
| 13 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Trae `TransformStamped`, el mensaje que representa un transform tf2. |
| 14 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Trae `VehicleOdometry`, el mensaje que llega de PX4. |
| 15 | `#include <px4_ros_com/frame_transforms.h>` | Trae las funciones de conversión NED↔ENU. |
| 16 | `#include <tf2_ros/transform_broadcaster.h>` | Trae `TransformBroadcaster`, la clase que publica transforms tf2. |
| 18-21 | Comentario Doxygen de la clase | Documenta qué hace `FramePublisher`. |
| 22 | `class FramePublisher : public rclcpp::Node` | Declara la clase, heredando de `rclcpp::Node`. |
| 23 | `{` | Abre el cuerpo de la clase. |
| 24 | `public:` | Lo siguiente es accesible desde fuera. |
| 25 | `explicit FramePublisher()` | Declara el constructor sin parámetros. |
| 26 | `: Node("interceptor_tf2_frame_publisher")` | Llama al constructor de `Node`, fija el nombre visible del nodo. |
| 27 | `{` | Abre el cuerpo del constructor. |
| 28 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");` | Declara el parámetro `vehicle_name` (por defecto `"interceptor"`) y guarda su valor. |
| 30 | `tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);` | Crea el objeto que publicará transforms tf2, ligado a este nodo. |
| 32 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copia un perfil de QoS pensado para datos de sensores. |
| 33 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Lo convierte en un `QoS` de ROS 2 con cola de 5 mensajes. |
| 35 | `std::ostringstream stream;` | Crea un "constructor de texto" para armar el nombre del tópico. |
| 36 | `stream << "/fmu/out/vehicle_odometry";` | Escribe el tópico de la instancia 0 (sin prefijo, la del interceptor). |
| 37 | `std::string topic_name = stream.str();` | Convierte lo escrito en `stream` a un `std::string` normal. |
| 39 | `subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,` | Empieza a crear la suscripción: tipo `VehicleOdometry`, tópico `topic_name`, calidad de servicio `qos`. |
| 40 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda que se ejecuta con cada mensaje recibido; `[this]` permite usar los atributos del nodo; `msg` es el mensaje. |
| 41 | `using px4_ros_com::frame_transforms::ned_to_enu_local_frame;` | Acorta el nombre de la función de conversión de posición. |
| 42 | `using px4_ros_com::frame_transforms::px4_to_ros_orientation;` | Acorta el nombre de la función de conversión de orientación. |
| 44 | `// PX4 position is NED, ROS/tf2 expects ENU` | Comentario explicativo. |
| 45 | `Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);` | Arma un vector 3D `double` con la posición NED del mensaje. |
| 46 | `Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);` | Convierte ese vector a ENU. |
| 48 | `// PX4 quaternion is (w, x, y, z), aircraft frame relative to NED` | Comentario sobre el orden de componentes. |
| 49 | `Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);` | Arma el cuaternión recibido, en orden (w, x, y, z). |
| 50 | `Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);` | Convierte esa orientación al convenio ROS. |
| 52 | `geometry_msgs::msg::TransformStamped t;` | Crea el mensaje de transform que se va a rellenar y publicar. |
| 53 | `t.header.stamp = this->get_clock()->now();` | Guarda el instante actual como marca de tiempo. |
| 54 | `t.header.frame_id = "map";` | Fija el frame padre: `map`. |
| 55 | `t.child_frame_id = vehicle_name_ + "/base_link";` | Fija el frame hijo, por defecto `interceptor/base_link`. |
| 57 | `t.transform.translation.x = position_enu.x();` | Copia la componente X (este) de la posición ENU. |
| 58 | `t.transform.translation.y = position_enu.y();` | Componente Y (norte). |
| 59 | `t.transform.translation.z = position_enu.z();` | Componente Z (arriba). |
| 61 | `t.transform.rotation.w = q_enu.w();` | Copia el componente `w` del cuaternión convertido. |
| 62 | `t.transform.rotation.x = q_enu.x();` | Componente `x`. |
| 63 | `t.transform.rotation.y = q_enu.y();` | Componente `y`. |
| 64 | `t.transform.rotation.z = q_enu.z();` | Componente `z`. |
| 66 | `tf_broadcaster_->sendTransform(t);` | Publica el transform al árbol tf2; desde aquí ya se puede consultar `map -> interceptor/base_link`. |
| 67 | `});` | Cierra la lambda y la llamada a `create_subscription`. |
| 68 | `}` | Cierra el constructor. |
| 70 | `private:` | Lo siguiente solo es accesible dentro de la clase. |
| 71 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Guarda la suscripción para que siga viva. |
| 72 | `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;` | Guarda el broadcaster para usarlo en la lambda. |
| 73 | `std::string vehicle_name_;` | Guarda el nombre del vehículo leído del parámetro. |
| 74 | `};` | Cierra la clase. |
| 76 | `int main(int argc, char *argv[])` | Punto de entrada del programa. |
| 77 | `{` | Abre `main`. |
| 78 | `std::cout << "Starting interceptor_tf2_odometry frame publisher..." << std::endl;` | Imprime un mensaje de arranque. |
| 79 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Desactiva el buffering de la salida estándar. |
| 80 | `rclcpp::init(argc, argv);` | Inicializa ROS 2. |
| 81 | `rclcpp::spin(std::make_shared<FramePublisher>());` | Crea el nodo y lo mantiene vivo procesando la callback de la línea 40. |
| 83 | `rclcpp::shutdown();` | Libera los recursos de ROS 2 al terminar `spin`. |
| 84 | `return 0;` | Fin sin errores. |
| 85 | `}` | Cierra `main`. |

## 3. `target_tf2_odometry.cpp`

Archivo: [`target_tf2_odometry.cpp`](../../src/interceptor/src/target_tf2_odometry.cpp)

Comparte casi todas las líneas con `interceptor_tf2_odometry.cpp`. La tabla
cubre el archivo completo; se marcan con **★** las líneas que no tienen
equivalente en el conversor del interceptor.

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1-9 | Comentario Doxygen `/** ... */` | Documenta archivo, propósito y, a diferencia del otro conversor, menciona explícitamente que también publica la velocidad del target. |
| 11 | `#include <memory>` | Igual que en el conversor del interceptor. |
| 12 | `#include <sstream>` | Igual. |
| 13 | `#include <string>` | Igual. |
| 15 | `#include <rclcpp/rclcpp.hpp>` | Igual. |
| 16 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Igual. |
| 17 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Igual. |
| 18 | `#include <px4_ros_com/frame_transforms.h>` | Igual. |
| 19 | `#include <tf2_ros/transform_broadcaster.h>` | Igual. |
| 20 | ★ `#include <geometry_msgs/msg/twist_stamped.hpp>` | Extra: trae `TwistStamped`, el tipo del mensaje `target/velocity`. |
| 22-25 | Comentario Doxygen de la clase | Igual que en el otro archivo. |
| 26 | `class FramePublisher : public rclcpp::Node` | Misma declaración de clase (mismo nombre de clase, archivo distinto). |
| 27 | `{` | Abre el cuerpo. |
| 28 | `public:` | Igual. |
| 29 | `explicit FramePublisher()` | Igual. |
| 30 | `: Node("target_tf2_frame_publisher")` | Nombre de nodo distinto. |
| 31 | `{` | Abre el constructor. |
| 32 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "target");` | Mismo parámetro, valor por defecto `"target"`. |
| 34 | `tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);` | Igual que en el otro archivo. |
| 36 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Igual. |
| 37 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Igual. |
| 39 | `std::ostringstream stream;` | Igual. |
| 40 | `stream << "/px4_1/fmu/out/vehicle_odometry";` | **Tópico distinto**: instancia 1, la del target. |
| 41 | `std::string topic_name = stream.str();` | Igual. |
| 44 | ★ `velocity_pub_ =` | Empieza a asignar el resultado de crear el publisher al atributo `velocity_pub_`. |
| 45 | ★ `this->create_publisher<geometry_msgs::msg::TwistStamped>("target/velocity", 10);` | Crea el publisher de velocidad en el tópico `target/velocity`, con cola de 10 mensajes. |
| 46 | ★ `auto timer_callback =` | Empieza a definir la función que ejecutará el timer. |
| 47 | ★ `[this]()->void {` | Lambda sin parámetros que devuelve `void`. |
| 48 | ★ `geometry_msgs::msg::TwistStamped msg;` | Crea el mensaje de velocidad a publicar. |
| 49 | ★ `msg.header.stamp = this->get_clock()->now();` | Marca de tiempo actual. |
| 50 | ★ `msg.header.frame_id = vehicle_name_ + "/base_link";` | Frame en el que se interpreta la velocidad. |
| 51 | ★ `msg.twist.linear.x = _target_velocity_enu.x();` | Copia la componente X guardada de la última velocidad ENU. |
| 52 | ★ `msg.twist.linear.y = _target_velocity_enu.y();` | Componente Y. |
| 53 | ★ `msg.twist.linear.z = _target_velocity_enu.z();` | Componente Z. |
| 54 | ★ `velocity_pub_->publish(msg);` | Publica el mensaje en `target/velocity`. |
| 55 | ★ `};` | Cierra la lambda `timer_callback`. |
| 56 | ★ `timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);` | Crea un timer que ejecuta `timer_callback` cada 100 ms (10 Hz), independientemente de cuándo lleguen mensajes de PX4. |
| 58 | `subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,` | Igual patrón que en el otro conversor, pero con el `topic_name` de la instancia 1. |
| 59 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Igual. |
| 60 | `using px4_ros_com::frame_transforms::ned_to_enu_local_frame;` | Igual. |
| 61 | `using px4_ros_com::frame_transforms::px4_to_ros_orientation;` | Igual. |
| 63 | `// PX4 position is NED, ROS/tf2 expects ENU` | Igual. |
| 64 | `Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);` | Igual. |
| 65 | `Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);` | Igual. |
| 67 | ★ `// PX4 velocity is NED, ROS/tf2 expects ENU` | Comentario nuevo: aquí sí se convierte también la velocidad. |
| 68 | ★ `Eigen::Vector3d velocity_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);` | Arma un vector 3D `double` con la velocidad NED del mensaje. |
| 69 | ★ `Eigen::Vector3d velocity_enu = ned_to_enu_local_frame(velocity_ned);` | Convierte esa velocidad a ENU (reutiliza la misma función que la posición). |
| 71 | `// PX4 quaternion is (w, x, y, z), aircraft frame relative to NED` | Igual. |
| 72 | `Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);` | Igual. |
| 73 | `Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);` | Igual. |
| 75 | `geometry_msgs::msg::TransformStamped t;` | Igual. |
| 76 | `t.header.stamp = this->get_clock()->now();` | Igual. |
| 77 | `t.header.frame_id = "map";` | Igual. |
| 78 | `t.child_frame_id = vehicle_name_ + "/base_link";` | Aquí resuelve a `target/base_link`. |
| 80 | ★ `// Vector3d not Vector3f, so use cast. TF2 only has translation and rotation` | Explica por qué hace falta el `.cast<float>()` de la línea siguiente. |
| 81 | ★ `_target_velocity_enu = velocity_enu.cast<float>();` | Guarda la velocidad ENU como `float` (convertida desde `double`) en el atributo que leerá el timer. |
| 83 | `t.transform.translation.x = position_enu.x();` | Igual que en el otro conversor. |
| 84 | `t.transform.translation.y = position_enu.y();` | Igual. |
| 85 | `t.transform.translation.z = position_enu.z();` | Igual. |
| 87 | `t.transform.rotation.w = q_enu.w();` | Igual. |
| 88 | `t.transform.rotation.x = q_enu.x();` | Igual. |
| 89 | `t.transform.rotation.y = q_enu.y();` | Igual. |
| 90 | `t.transform.rotation.z = q_enu.z();` | Igual. |
| 92 | `tf_broadcaster_->sendTransform(t);` | Publica `map -> target/base_link`. |
| 93 | `});` | Cierra la lambda y `create_subscription`. |
| 95 | `}` | Cierra el constructor. |
| 97 | `private:` | Igual. |
| 98 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Igual. |
| 99 | `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;` | Igual. |
| 100 | ★ `rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;` | Guarda el publisher de velocidad. |
| 101 | ★ `Eigen::Vector3f _target_velocity_enu{Eigen::Vector3f::Zero()};` | Guarda la última velocidad ENU conocida, inicializada a cero para que el timer no lea basura antes del primer mensaje PX4. |
| 102 | ★ `rclcpp::TimerBase::SharedPtr timer_;` | Guarda el timer de 100 ms. |
| 103 | `std::string vehicle_name_;` | Igual. |
| 104 | `};` | Cierra la clase. |
| 106 | `int main(int argc, char *argv[])` | Igual estructura que el otro conversor. |
| 107 | `{` | Abre `main`. |
| 108 | `std::cout << "Starting interceptor_tf2_odometry frame publisher..." << std::endl;` | El texto dice "interceptor" aunque este binario es `target_tf2_odometry`: es un mensaje de arranque copiado sin actualizar, otro detalle de nombres a tener en cuenta. |
| 109 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Igual. |
| 110 | `rclcpp::init(argc, argv);` | Igual. |
| 111 | `rclcpp::spin(std::make_shared<FramePublisher>());` | Igual. |
| 113 | `rclcpp::shutdown();` | Igual. |
| 114 | `return 0;` | Igual. |
| 115 | `}` | Cierra `main`. |

La callback de odometría (líneas 58-93) hace dos cosas con una sola entrada:
publica el transform inmediatamente (línea 92) y guarda la velocidad para que
el timer, en un ritmo distinto (100 ms fijos), la publique por separado
(líneas 46-56). Por eso hay dos "relojes" distintos funcionando en este mismo
archivo.

## 4. `tf2_listener.cpp`

Archivo: [`tf2_listener.cpp`](../../src/interceptor/src/tf2_listener.cpp)

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1 | `#include <chrono>` | Trae duraciones y literales como `1s`. |
| 2 | `#include <functional>` | Trae `std::bind`, usado para enlazar el timer a un método. |
| 3 | `#include <memory>` | Trae `std::unique_ptr`/`std::make_unique`. |
| 4 | `#include <string>` | Trae `std::string`. |
| 6 | `#include "geometry_msgs/msg/transform_stamped.hpp"` | Trae `TransformStamped`, el tipo del resultado de una consulta tf2. |
| 7 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Trae `VehicleOdometry`. |
| 8 | `#include "rclcpp/rclcpp.hpp"` | Trae `Node`, `init`, `spin`, logging. |
| 9 | `#include "tf2/exceptions.h"` | Trae `tf2::TransformException`, la excepción que se lanza si falta un frame. |
| 10 | `#include "tf2_ros/transform_listener.h"` | Trae `TransformListener`, que rellena el buffer escuchando tf2. |
| 11 | `#include "tf2_ros/buffer.h"` | Trae `Buffer`, donde se almacenan las transformaciones recibidas. |
| 13 | `using namespace std::chrono_literals;` | Permite escribir duraciones como `1s` en vez de construirlas a mano. |
| 15 | `class FrameListener : public rclcpp::Node` | Declara la clase, heredando de `rclcpp::Node`. |
| 16 | `{` | Abre el cuerpo de la clase. |
| 17 | `public:` | Lo siguiente es accesible desde fuera. |
| 18 | `FrameListener()` | Declara el constructor sin parámetros (aquí sin `explicit`, a diferencia de otros nodos del paquete). |
| 19 | `: Node("tf2_frame_listener")` | Llama al constructor de `Node`, fija el nombre visible. |
| 20 | `{` | Abre el cuerpo del constructor. |
| 21 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copia un perfil de QoS de sensor. |
| 22 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Lo convierte en un `QoS` con cola de 5 mensajes. |
| 24 | `// Declare and acquire \`target_frame\` parameter` | Comentario explicativo. |
| 25 | `target_frame_ = this->declare_parameter<std::string>("target_frame", "target/base_link");` | Declara el parámetro `target_frame` (por defecto `target/base_link`) y guarda su valor. |
| 27-28 | `tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());` | Crea el buffer tf2, que necesita el reloj del nodo para interpretar marcas de tiempo. |
| 29-30 | `tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);` | Crea el listener, que escucha tf2 en segundo plano y llena `tf_buffer_`. |
| 32 | `// Subscribe to target odometry to keep the latest velocity` | Comentario explicativo. |
| 33 | `subscription_ =` | Empieza a asignar la suscripción. |
| 34 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry",` | Se suscribe a la odometría del target (instancia 1) directamente, no vía tf2, porque tf2 no lleva velocidad. |
| 35 | `qos,` | Usa la calidad de servicio creada arriba. |
| 36 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda ejecutada con cada mensaje de odometría del target. |
| 37 | `target_velocity_[0] = msg->velocity[0];` | Guarda la componente X de la velocidad, en NED, sin convertir. |
| 38 | `target_velocity_[1] = msg->velocity[1];` | Componente Y. |
| 39 | `target_velocity_[2] = msg->velocity[2];` | Componente Z. |
| 40 | `});` | Cierra la lambda y `create_subscription`. |
| 42 | `// Call on_timer function every second` | Comentario explicativo. |
| 43-44 | `timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer, this));` | Crea un timer que llama a `on_timer()` cada segundo; `std::bind` conecta el método con `this` como si fuera una función suelta. |
| 45 | `}` | Cierra el constructor. |
| 47 | `private:` | Lo siguiente solo es accesible dentro de la clase. |
| 48 | `void on_timer()` | Declara el método que se ejecuta cada segundo. |
| 49 | `{` | Abre el cuerpo del método. |
| 50-51 | `// Store frame names in variables that will be used to compute transformations` | Comentario explicativo. |
| 52 | `std::string fromFrameRel = target_frame_.c_str();` | Frame de origen de la consulta: el del target. |
| 53 | `std::string toFrameRel = "interceptor/base_link";` | Frame de destino: el del interceptor (fijo, no parametrizable). |
| 55 | `geometry_msgs::msg::TransformStamped t;` | Variable donde se guardará el resultado de la consulta. |
| 57 | `try {` | Empieza el bloque que puede fallar si falta algún frame. |
| 58-60 | `t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);` | Pide la transformación "target visto desde interceptor", usando el último dato disponible (`TimePointZero`). |
| 61 | `} catch (const tf2::TransformException & ex) {` | Si `lookupTransform` lanza una excepción (falta algún frame), se captura aquí. |
| 62-64 | `RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ...);` | Registra un mensaje indicando qué transformación falló y por qué. |
| 65 | `return;` | Sale del método sin usar datos inválidos. |
| 66 | `}` | Cierra el bloque `catch`. |
| 68-73 | `RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: translation (...), velocity (...)", ...);` | Si la consulta funcionó, imprime la traslación obtenida y la última velocidad guardada del target. |
| 75 | `}` | Cierra `on_timer`. |
| 77 | `rclcpp::TimerBase::SharedPtr timer_{nullptr};` | Guarda el timer, inicializado explícitamente a nulo. |
| 78 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Guarda la suscripción a la odometría del target. |
| 79 | `std::array<float, 3> target_velocity_;` | Guarda la última velocidad NED conocida del target. |
| 80 | `std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};` | Guarda el listener tf2. |
| 81 | `std::unique_ptr<tf2_ros::Buffer> tf_buffer_;` | Guarda el buffer tf2. |
| 82 | `std::string target_frame_;` | Guarda el nombre del frame del target leído del parámetro. |
| 83 | `};` | Cierra la clase. |
| 85 | `int main(int argc, char * argv[])` | Punto de entrada del programa. |
| 86 | `{` | Abre `main`. |
| 87 | `rclcpp::init(argc, argv);` | Inicializa ROS 2. |
| 88 | `rclcpp::spin(std::make_shared<FrameListener>());` | Crea el nodo y lo mantiene vivo: aquí se ejecutan la lambda de odometría y `on_timer`. |
| 89 | `rclcpp::shutdown();` | Libera los recursos de ROS 2. |
| 90 | `return 0;` | Fin sin errores. |
| 91 | `}` | Cierra `main`. |

Este nodo nunca escribe un setpoint ni llama a nada de `px4_ros2`: solo lee y
muestra. Por eso es seguro ejecutarlo en paralelo con los modos de guiado sin
riesgo de interferir con el control de vuelo — su único efecto es imprimir
texto en la consola/logs.

Con esto terminan los nodos de odometría y tf2. Continúa con
[Modos de guiado, línea por línea](Line-by-line-guidance-modes.md).

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Archivos de proyecto y construcción, línea por línea](Line-by-line-project-and-build-files.md) · ➡️ Siguiente: [Modos de guiado, línea por línea](Line-by-line-guidance-modes.md)
