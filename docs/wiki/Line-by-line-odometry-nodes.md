# Análisis línea por línea: nodos de odometría y tf2

Esta es la segunda de tres páginas que cubren, línea por línea, todos los
archivos propios de `src/interceptor`. Empieza por
[Archivos de proyecto y construcción](Line-by-line-project-and-build-files.md) si aún no
la has leído: ahí están el glosario de sintaxis de C++ (`override`, `explicit`,
`constexpr`, plantillas, referencias, `.cross()`/`.normalized()`...) y las
notas de "cómo usar este análisis" que también aplican aquí. La tercera página
es [Modos de guiado, línea por línea](Line-by-line-guidance-modes.md).

Esta página cubre el suscriptor de diagnóstico de odometría
(`vehicle_odometry_subscriber`, un único ejecutable que el launch arranca dos
veces con parámetros distintos), los dos conversores tf2
(`interceptor_tf2_odometry`, `target_tf2_odometry`) y `tf2_listener` — los
nodos que reciben la odometría de PX4, la convierten y la relacionan mediante
tf2, sin calcular ningún setpoint de vuelo.

## 1. `vehicle_odometry_subscriber.cpp`

Archivo: [`vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/vehicle_odometry_subscriber.cpp)

Un solo ejecutable sirve para diagnosticar los dos vehículos: el launch lo
arranca dos veces, cada una con su propio `name=` y sus propios parámetros
`vehicle_name`/`odometry_topic` (ver
[Archivos de proyecto y construcción](Line-by-line-project-and-build-files.md)).

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1-8 | Comentario Doxygen `/** ... */` | Documenta archivo, propósito (`Vehicle Odometry uORB topic listener example`), autor, y explica que un solo ejecutable sirve para los dos vehículos según sus parámetros. |
| 10 | `#include <rclcpp/rclcpp.hpp>` | Trae `Node`, `init`, `spin`, QoS y logging. |
| 11 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Trae el tipo `VehicleOdometry`. |
| 13-15 | Comentario Doxygen de la clase | Documenta la clase que sigue. |
| 16 | `class VehicleOdometrySubscriber : public rclcpp::Node` | Declara la clase, heredando de `rclcpp::Node`. |
| 17 | `{` | Abre el cuerpo de la clase. |
| 18 | `public:` | Lo siguiente es accesible desde fuera de la clase. |
| 19 | `explicit VehicleOdometrySubscriber()` | Declara el constructor sin parámetros; `explicit` evita conversiones implícitas. |
| 20 | `: Node("vehicle_odometry_subscriber")` | Llama al constructor de `Node`, fijando este nombre visible por defecto; el launch le asigna otro distinto a cada instancia con `name=`. |
| 21 | `{` | Abre el cuerpo del constructor. |
| 22 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "interceptor");` | Declara el parámetro `vehicle_name` (por defecto `"interceptor"`) y guarda su valor. |
| 23-24 | `std::string odometry_topic = this->declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");` | Declara el parámetro `odometry_topic` (por defecto `/fmu/out/vehicle_odometry`) y lo guarda en una variable local. |
| 26 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Copia un perfil de QoS pensado para datos de sensores. |
| 27 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Convierte ese perfil en un `QoS` de ROS 2 con cola de 5 mensajes. |
| 29 | `subscription_ =` | Empieza a asignar el resultado de crear la suscripción al atributo `subscription_`. |
| 30 | `this->create_subscription<px4_msgs::msg::VehicleOdometry>(odometry_topic, qos,` | Crea la suscripción al tópico indicado por el parámetro `odometry_topic` con la calidad de servicio `qos`. |
| 31 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Lambda que se ejecuta con cada mensaje recibido; `msg` es ese mensaje. |
| 32 | `std::cout << "\n\n\n...";` | Imprime muchas líneas en blanco para "limpiar" visualmente la consola antes de cada muestra. |
| 33 | `std::cout << "RECEIVED VEHICLE ODOMETRY DATA (" << vehicle_name_ << ")" << std::endl;` | Imprime un encabezado que incluye el nombre del vehículo leído del parámetro, p. ej. `RECEIVED VEHICLE ODOMETRY DATA (target)`. |
| 34 | `std::cout << "Timestamp: " << msg->timestamp << std::endl;` | Imprime la marca de tiempo del mensaje PX4. |
| 35 | `std::cout << "Pose frame: " << msg->pose_frame << std::endl;` | Imprime el identificador de frame que PX4 asigna a la pose. |
| 36 | `std::cout << "X Position: " << msg->position[0] << std::endl;` | Imprime la posición X (NED) tal cual la envía PX4, sin convertir. |
| 37 | `std::cout << "Y Position: " << msg->position[1] << std::endl;` | Posición Y (NED). |
| 38 | `std::cout << "Z Position: " << msg->position[2] << std::endl;` | Posición Z (NED). |
| 39 | `std::cout << "q[0]: " << msg->q[0] << std::endl;` | Primer componente del cuaternión de orientación (`w`). |
| 40 | `std::cout << "q[1]: " << msg->q[1] << std::endl;` | Segundo componente (`x`). |
| 41 | `std::cout << "q[2]: " << msg->q[2] << std::endl;` | Tercer componente (`y`). |
| 42 | `std::cout << "q[3]: " << msg->q[3] << std::endl;` | Cuarto componente (`z`). |
| 43 | `std::cout << "X Velocity: " << msg->velocity[0] << std::endl;` | Velocidad X (NED). |
| 44 | `std::cout << "Y Velocity: " << msg->velocity[1] << std::endl;` | Velocidad Y (NED). |
| 45 | `std::cout << "Z Velocity: " << msg->velocity[2] << std::endl;` | Velocidad Z (NED). |
| 46-48 | `std::cout << "Velocity magnitude: " << sqrt(msg->velocity[0] * msg->velocity[0] + msg->velocity[1] * msg->velocity[1] + msg->velocity[2] * msg->velocity[2]) << std::endl;` | Calcula e imprime la longitud del vector velocidad (raíz cuadrada de la suma de cuadrados). |
| 49 | `});` | Cierra la lambda y la llamada a `create_subscription`. |
| 50 | `}` | Cierra el constructor. |
| 52 | `private:` | Lo siguiente solo es accesible dentro de la clase. |
| 53 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Guarda la suscripción; si no se guardara aquí, se destruiría al salir del constructor y dejaría de recibir mensajes. |
| 54 | `std::string vehicle_name_;` | Guarda el nombre del vehículo leído del parámetro, para usarlo en el encabezado impreso. |
| 56 | `};` | Cierra la clase. |
| 58 | `int main(int argc, char *argv[])` | Punto de entrada del programa. |
| 59 | `{` | Abre `main`. |
| 60 | `std::cout << "Starting vehicle_odometry_subscriber node..." << std::endl;` | Imprime un mensaje de arranque. |
| 61 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Desactiva el buffering de la salida estándar para que los `std::cout` aparezcan de inmediato. |
| 62 | `rclcpp::init(argc, argv);` | Inicializa ROS 2. |
| 63 | `rclcpp::spin(std::make_shared<VehicleOdometrySubscriber>());` | Crea el nodo y lo mantiene vivo procesando la callback de la línea 31 mientras el proceso corre. |
| 65 | `rclcpp::shutdown();` | Libera los recursos de ROS 2 cuando `spin` termina. |
| 66 | `return 0;` | Indica al sistema operativo que el programa terminó sin errores. |
| 67 | `}` | Cierra `main`. |

### Por qué esto importa

Antes había dos archivos casi idénticos, uno por vehículo, que solo se
diferenciaban en el nombre del nodo y en el tópico. Esa copia duplicada acabó
provocando un error que estuvo tiempo en el código: el ejecutable llamado
`target_...` escuchaba en realidad la odometría del interceptor, y al revés.

Ahora hay un solo archivo y el vehículo se elige con parámetros, así que el
nombre de cada nodo lo decide el launch junto al tópico que le pasa: los dos
datos van en el mismo sitio y no pueden descuadrarse. Es un ejemplo de por qué
conviene parametrizar en lugar de duplicar: el fallo no fue un descuido
puntual, sino algo que la duplicación hacía fácil.

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
| 1-13 | Comentario Doxygen `/** ... */` | Documenta archivo y propósito; menciona que también publica la velocidad del target y explica (líneas 9-12) que la posición del target se desplaza por el offset NED entre el origen del target y el del interceptor, para que ambos queden expresados en el mismo origen. |
| 15 | `#include <memory>` | Igual que en el conversor del interceptor. |
| 16 | `#include <sstream>` | Igual. |
| 17 | `#include <string>` | Igual. |
| 19 | `#include <rclcpp/rclcpp.hpp>` | Igual. |
| 20 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Igual. |
| 21 | `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Igual. |
| 22 | ★ `#include <px4_msgs/msg/vehicle_local_position.hpp>` | Nuevo: trae `VehicleLocalPosition`, el mensaje con `ref_lat`/`ref_lon`/`ref_alt`. |
| 23 | `#include <px4_ros_com/frame_transforms.h>` | Igual. |
| 24 | ★ `#include <px4_ros2/utils/geodesic.hpp>` | Nuevo: trae `px4_ros2::vectorToGlobalPosition`, usada para calcular el desplazamiento NED entre orígenes. |
| 25 | ★ `#include <px4_ros2/utils/message_version.hpp>` | Nuevo: trae `getMessageNameVersion`, para componer el sufijo de versión del tópico de posición local. |
| 26 | `#include <tf2_ros/transform_broadcaster.h>` | Igual. |
| 27 | ★ `#include <geometry_msgs/msg/twist_stamped.hpp>` | Extra: trae `TwistStamped`, el tipo del mensaje `target/velocity`. |
| 29-32 | Comentario Doxygen de la clase | Igual que en el otro archivo. |
| 33 | `class FramePublisher : public rclcpp::Node` | Misma declaración de clase (mismo nombre de clase, archivo distinto). |
| 34 | `{` | Abre el cuerpo. |
| 35 | `public:` | Igual. |
| 36 | `explicit FramePublisher()` | Igual. |
| 37 | `: Node("target_tf2_frame_publisher")` | Nombre de nodo distinto. |
| 38 | `{` | Abre el constructor. |
| 39 | `vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "target");` | Mismo parámetro, valor por defecto `"target"`. |
| 41 | `tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);` | Igual que en el otro archivo. |
| 43 | `rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;` | Igual. |
| 44 | `auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);` | Igual. |
| 46 | ★ `const std::string local_position_version_suffix =` | Nuevo: empieza a guardar el sufijo de versión del mensaje `VehicleLocalPosition`. |
| 47 | ★ `px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>();` | Nuevo: calcula ese sufijo (algo como `_v1` con el `px4_msgs` vendorizado) a partir del tipo del mensaje. |
| 49 | ★ `local_position_sub_interceptor_ =` | Nuevo: empieza a crear la suscripción a la posición local del interceptor. |
| 50 | ★ `this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(` | Nuevo: tipo del mensaje a suscribir. |
| 51 | ★ `"/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,` | Nuevo: tópico de la instancia 0 (interceptor), con el mismo QoS que la odometría. |
| 52 | ★ `[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {` | Nuevo: lambda de la callback. |
| 53 | ★ `ref_lat_interceptor_ = msg->ref_lat;` | Nuevo: guarda la latitud de referencia del origen del interceptor. |
| 54 | ★ `ref_lon_interceptor_ = msg->ref_lon;` | Nuevo: longitud de referencia. |
| 55 | ★ `ref_alt_interceptor_ = msg->ref_alt;` | Nuevo: altitud de referencia. |
| 56 | ★ `ref_valid_interceptor_ = msg->xy_global && msg->z_global;` | Nuevo: la referencia solo se da por válida si el EKF ya tiene origen global horizontal y vertical. |
| 57 | ★ `});` | Nuevo: cierra la lambda y `create_subscription`. |
| 59 | ★ `local_position_sub_target_ =` | Nuevo: empieza a crear la suscripción a la posición local del target. |
| 60 | ★ `this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(` | Nuevo. |
| 61 | ★ `"/px4_1/fmu/out/vehicle_local_position" + local_position_version_suffix, qos,` | Nuevo: tópico de la instancia 1 (target). |
| 62 | ★ `[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {` | Nuevo. |
| 63 | ★ `ref_lat_target_ = msg->ref_lat;` | Nuevo. |
| 64 | ★ `ref_lon_target_ = msg->ref_lon;` | Nuevo. |
| 65 | ★ `ref_alt_target_ = msg->ref_alt;` | Nuevo. |
| 66 | ★ `ref_valid_target_ = msg->xy_global && msg->z_global;` | Nuevo. |
| 67 | ★ `});` | Nuevo: cierra la lambda y `create_subscription`. |
| 69 | `std::ostringstream stream;` | Igual. |
| 70 | `stream << "/px4_1/fmu/out/vehicle_odometry";` | **Tópico distinto**: instancia 1, la del target. |
| 71 | `std::string topic_name = stream.str();` | Igual. |
| 74 | ★ `velocity_pub_ =` | Empieza a asignar el resultado de crear el publisher al atributo `velocity_pub_`. |
| 75 | ★ `this->create_publisher<geometry_msgs::msg::TwistStamped>("target/velocity", 10);` | Crea el publisher de velocidad en el tópico `target/velocity`, con cola de 10 mensajes. |
| 76 | ★ `auto timer_callback =` | Empieza a definir la función que ejecutará el timer. |
| 77 | ★ `[this]()->void {` | Lambda sin parámetros que devuelve `void`. |
| 78 | ★ `geometry_msgs::msg::TwistStamped msg;` | Crea el mensaje de velocidad a publicar. |
| 79 | ★ `msg.header.stamp = this->get_clock()->now();` | Marca de tiempo actual. |
| 80 | ★ `msg.header.frame_id = vehicle_name_ + "/base_link";` | Frame en el que se interpreta la velocidad. |
| 81 | ★ `msg.twist.linear.x = _target_velocity_enu.x();` | Copia la componente X guardada de la última velocidad ENU. |
| 82 | ★ `msg.twist.linear.y = _target_velocity_enu.y();` | Componente Y. |
| 83 | ★ `msg.twist.linear.z = _target_velocity_enu.z();` | Componente Z. |
| 84 | ★ `velocity_pub_->publish(msg);` | Publica el mensaje en `target/velocity`. |
| 85 | ★ `};` | Cierra la lambda `timer_callback`. |
| 86 | ★ `timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);` | Crea un timer que ejecuta `timer_callback` cada 100 ms (10 Hz), independientemente de cuándo lleguen mensajes de PX4. |
| 88 | `subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(topic_name, qos,` | Igual patrón que en el otro conversor, pero con el `topic_name` de la instancia 1. |
| 89 | `[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {` | Igual. |
| 90 | `using px4_ros_com::frame_transforms::ned_to_enu_local_frame;` | Igual. |
| 91 | `using px4_ros_com::frame_transforms::px4_to_ros_orientation;` | Igual. |
| 93 | ★ `// PX4 velocity is NED, ROS/tf2 expects ENU` | Comentario: aquí también se convierte la velocidad (no existe en el conversor del interceptor). |
| 94 | ★ `Eigen::Vector3d velocity_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);` | Arma un vector 3D `double` con la velocidad NED del mensaje. |
| 95 | ★ `Eigen::Vector3d velocity_enu = ned_to_enu_local_frame(velocity_ned);` | Convierte esa velocidad a ENU (reutiliza la misma función que la posición). |
| 97 | ★ `// Vector3d not Vector3f, so use cast. TF2 only has translation and rotation` | Explica por qué hace falta el `.cast<float>()` de la línea siguiente. |
| 98 | ★ `_target_velocity_enu = velocity_enu.cast<float>();` | Guarda la velocidad ENU como `float` (convertida desde `double`) en el atributo que leerá el timer; se hace antes de mirar si hay que publicar, así el timer siempre tiene el último valor aunque el transform no se publique todavía. |
| 100 | ★ `if (!ref_valid_interceptor_ || !ref_valid_target_) {` | Nuevo: si falta la referencia global de alguno de los dos vehículos, el offset entre orígenes no se puede calcular todavía. |
| 101 | ★ `// Missing global reference of interceptor and/or target: the NED offset` | Comentario que explica el porqué del `return`. |
| 102 | ★ `// between origins can't be computed yet, so don't publish a wrong transform` | Continúa el comentario. |
| 103 | ★ `std::string missing_refs;` | Nuevo: declara una cadena vacía que irá acumulando qué referencias faltan. |
| 104 | ★ `if (!ref_valid_interceptor_) {` | Si falta la referencia del interceptor, entra en este bloque. |
| 105 | ★ `missing_refs += " interceptor";` | Añade `" interceptor"` a la cadena. |
| 106 | ★ `}` | Cierra el `if` anterior. |
| 107 | ★ `if (!ref_valid_target_) {` | Si falta la referencia del target, entra en este bloque. |
| 108 | ★ `missing_refs += missing_refs.empty() ? " target" : " and target";` | Si `missing_refs` seguía vacía (solo falta el target) añade `" target"`; si ya llevaba `" interceptor"` (faltan las dos) añade `" and target"`, para que el aviso quede `of interceptor and target` y no `of interceptor target`. |
| 109 | ★ `}` | Cierra el `if` anterior. |
| 110 | ★ `RCLCPP_WARN_THROTTLE(` | Avisa por log, limitado en frecuencia. |
| 111 | ★ `this->get_logger(), *this->get_clock(), 5000,` | Como mucho un aviso cada 5000 ms. |
| 112 | ★ `"Waiting for global reference (ref_lat/ref_lon/ref_alt) of%s before publishing "` | Primera parte del texto del aviso: ahora un solo `%s` (antes eran dos), que rellena `missing_refs`. |
| 113 | ★ `"map -> %s/base_link",` | Segunda parte del texto del aviso. |
| 114 | ★ `missing_refs.c_str(),` | Rellena el `%s` con la cadena construida arriba (`" interceptor"`, `" target"` o `" interceptor and target"`). |
| 115 | ★ `vehicle_name_.c_str());` | Nombre del vehículo (`target`) que aparece al final del aviso. |
| 116 | ★ `return;` | No publica `map -> target/base_link` mientras falte alguna referencia global. |
| 117 | ★ `}` | Cierra el bloque `if`. |
| 119 | `// PX4 position is NED, ROS/tf2 expects ENU` | Igual que en el otro conversor. |
| 120 | `Eigen::Vector3d position_ned(msg->position[0], msg->position[1], msg->position[2]);` | Igual. |
| 122 | ★ `// NED offset of the target's origin with respect to the interceptor's,` | Nuevo comentario. |
| 123 | ★ `// so both vehicles end up expressed in the same origin (map = interceptor's)` | Continúa el comentario. |
| 124 | ★ `Eigen::Vector3d global_position_interceptor(` | Nuevo: arma el vector lat/lon/alt de referencia del interceptor. |
| 125 | ★ `ref_lat_interceptor_, ref_lon_interceptor_, ref_alt_interceptor_);` | Continúa la construcción del vector. |
| 126 | ★ `Eigen::Vector3d global_position_target(` | Nuevo: arma el vector lat/lon/alt de referencia del target. |
| 127 | ★ `ref_lat_target_, ref_lon_target_, ref_alt_target_);` | Continúa la construcción del vector. |
| 128 | ★ `Eigen::Vector3f origin_offset_ned = px4_ros2::vectorToGlobalPosition(` | Nuevo: calcula el desplazamiento NED del origen del target respecto al del interceptor. |
| 129 | ★ `global_position_interceptor, global_position_target);` | Argumentos: posición global "ahora" (interceptor) y "siguiente" (target). |
| 130 | ★ `position_ned += origin_offset_ned.cast<double>();` | Nuevo: suma ese desplazamiento a la posición NED del target, antes de convertirla a ENU. |
| 132 | `Eigen::Vector3d position_enu = ned_to_enu_local_frame(position_ned);` | Igual que en el otro conversor, pero ahora con la posición ya desplazada al origen del interceptor. |
| 134 | `// PX4 quaternion is (w, x, y, z), aircraft frame relative to NED` | Igual. |
| 135 | `Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);` | Igual. |
| 136 | `Eigen::Quaterniond q_enu = px4_to_ros_orientation(q_ned);` | Igual. |
| 138 | `geometry_msgs::msg::TransformStamped t;` | Igual. |
| 139 | `t.header.stamp = this->get_clock()->now();` | Igual. |
| 140 | `t.header.frame_id = "map";` | Igual. |
| 141 | `t.child_frame_id = vehicle_name_ + "/base_link";` | Aquí resuelve a `target/base_link`. |
| 143 | `t.transform.translation.x = position_enu.x();` | Igual que en el otro conversor. |
| 144 | `t.transform.translation.y = position_enu.y();` | Igual. |
| 145 | `t.transform.translation.z = position_enu.z();` | Igual. |
| 147 | `t.transform.rotation.w = q_enu.w();` | Igual. |
| 148 | `t.transform.rotation.x = q_enu.x();` | Igual. |
| 149 | `t.transform.rotation.y = q_enu.y();` | Igual. |
| 150 | `t.transform.rotation.z = q_enu.z();` | Igual. |
| 152 | `tf_broadcaster_->sendTransform(t);` | Publica `map -> target/base_link`, ya con la posición del target expresada en el origen del interceptor. |
| 153 | `});` | Cierra la lambda y `create_subscription`. |
| 155 | `}` | Cierra el constructor. |
| 157 | `private:` | Igual. |
| 158 | `rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;` | Igual. |
| 159-160 | ★ `rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr` / `local_position_sub_interceptor_;` | Nuevo: guarda la suscripción a la posición local del interceptor. |
| 161 | ★ `rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_target_;` | Nuevo: guarda la suscripción a la posición local del target. |
| 162 | `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;` | Igual. |
| 163 | ★ `rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;` | Guarda el publisher de velocidad. |
| 164 | ★ `Eigen::Vector3f _target_velocity_enu{Eigen::Vector3f::Zero()};` | Guarda la última velocidad ENU conocida, inicializada a cero para que el timer no lea basura antes del primer mensaje PX4. |
| 165 | ★ `rclcpp::TimerBase::SharedPtr timer_;` | Guarda el timer de 100 ms. |
| 166 | `std::string vehicle_name_;` | Igual. |
| 167 | ★ `double ref_lat_interceptor_{0.0};` | Nuevo: última latitud de referencia del interceptor recibida. |
| 168 | ★ `double ref_lon_interceptor_{0.0};` | Nuevo: longitud de referencia del interceptor. |
| 169 | ★ `float ref_alt_interceptor_{0.0F};` | Nuevo: altitud de referencia del interceptor. |
| 170 | ★ `bool ref_valid_interceptor_{false};` | Nuevo: si esa referencia ya es válida. |
| 171 | ★ `double ref_lat_target_{0.0};` | Nuevo: lo mismo para el target. |
| 172 | ★ `double ref_lon_target_{0.0};` | Nuevo. |
| 173 | ★ `float ref_alt_target_{0.0F};` | Nuevo. |
| 174 | ★ `bool ref_valid_target_{false};` | Nuevo. |
| 175 | `};` | Cierra la clase. |
| 177 | `int main(int argc, char *argv[])` | Igual estructura que el otro conversor. |
| 178 | `{` | Abre `main`. |
| 179 | `std::cout << "Starting target_tf2_odometry frame publisher..." << std::endl;` | Mensaje de arranque, ahora coincide con el nombre del binario (`target_tf2_odometry`). |
| 180 | `setvbuf(stdout, NULL, _IONBF, BUFSIZ);` | Igual. |
| 181 | `rclcpp::init(argc, argv);` | Igual. |
| 182 | `rclcpp::spin(std::make_shared<FramePublisher>());` | Igual. |
| 184 | `rclcpp::shutdown();` | Igual. |
| 185 | `return 0;` | Igual. |
| 186 | `}` | Cierra `main`. |

La callback de odometría (líneas 88-153) hace tres cosas con una sola entrada:
guarda la velocidad para que el timer, en un ritmo distinto (100 ms fijos), la
publique por separado (líneas 76-86); si falta la referencia global de
interceptor o target, avisa y sale sin publicar (líneas 100-117); y si ambas
referencias ya son válidas, desplaza la posición del target al origen del
interceptor antes de publicar el transform (líneas 122-152). Por eso hay dos
"relojes" distintos funcionando en este mismo archivo.

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
| 50-51 | `// Store frame names in variables that will be used to` + `// compute transformations` | Comentario explicativo, partido en dos líneas. |
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
