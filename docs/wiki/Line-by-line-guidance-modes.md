# Análisis línea por línea: modos de guiado

Esta es la tercera de tres páginas que cubren, línea por línea, todos los
archivos propios de `src/interceptor`. Empieza por
[Archivos de proyecto y construcción](Line-by-line-code-analysis.md) si aún no
la has leído: ahí está el glosario de sintaxis de C++ (`override`, `explicit`,
`constexpr`, plantillas, referencias, `.cross()`/`.normalized()`...) que
también hace falta aquí. La segunda página es
[Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md).

Esta página cubre `pursuit_mode.cpp` y `PN_mode.cpp`: los dos modos de vuelo
que calculan un setpoint de trayectoria a partir de la posición (y, en PN, la
velocidad) del target.

## 1. `pursuit_mode.cpp`

Archivo: [`pursuit_mode.cpp`](../../src/interceptor/src/pursuit_mode.cpp)

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1 | `#include <Eigen/Eigen>` | Trae vectores y operaciones de álgebra lineal (`Eigen::Vector3f`, `.norm()`, `.normalized()`...). |
| 2 | `#include <geometry_msgs/msg/transform_stamped.hpp>` | Trae `TransformStamped`, el resultado de una consulta tf2. |
| 3 | `#include <px4_ros2/components/mode.hpp>` | Trae `ModeBase`, la clase base de un modo de vuelo personalizado. |
| 4 | `#include <px4_ros2/components/node_with_mode.hpp>` | Trae `NodeWithMode`, el envoltorio que registra el modo como nodo ROS 2. |
| 5 | `#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>` | Trae `TrajectorySetpointType`, usado para enviar velocidad/aceleración/yaw a PX4. |
| 6 | `#include <px4_ros2/odometry/local_position.hpp>` | Trae `OdometryLocalPosition`, para leer la posición/velocidad propias. |
| 7 | `#include <px4_ros2/utils/geometry.hpp>` | Utilidades geométricas de la librería `px4_ros2`. |
| 8 | `#include <px4_ros_com/frame_transforms.h>` | Trae las funciones de conversión NED↔ENU. |
| 9 | `#include <rclcpp/rclcpp.hpp>` | Trae `Node`, `init`, `spin`, logging. |
| 10 | `#include <tf2/exceptions.h>` | Trae `tf2::TransformException`. |
| 11 | `#include <tf2_ros/buffer.h>` | Trae `Buffer`, donde tf2 guarda las transformaciones. |
| 12 | `#include <tf2_ros/transform_listener.h>` | Trae `TransformListener`, que llena ese buffer. |
| 14 | `using namespace std::chrono_literals;  // NOLINT` | Permite escribir `50ms` como duración; el comentario `NOLINT` pide al linter que no avise por este `using namespace`. |
| 16 | `static const std::string kName = "Pursuit Intercept";` | Nombre del modo tal como lo mostrará PX4. |
| 17 | `static const std::string kMapFrame = "map";` | Nombre del frame padre usado en las consultas tf2. |
| 19 | `class PursuitMode : public px4_ros2::ModeBase` | Declara la clase, heredando de `ModeBase`: así PX4 puede registrarla y llamar a sus métodos. |
| 20 | `{` | Abre el cuerpo de la clase. |
| 21 | `public:` | Lo siguiente es accesible desde fuera. |
| 22 | `explicit PursuitMode(rclcpp::Node & node)` | Constructor que recibe una referencia al nodo ROS 2 que lo envuelve. |
| 23 | `: ModeBase(node, Settings{kName})` | Inicializa la clase base con el nodo y el nombre del modo. |
| 24 | `{` | Abre el cuerpo del constructor. |
| 25 | `_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);` | Crea el objeto que se usará para enviar los setpoints de trayectoria. |
| 26 | `_own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);` | Crea el lector de la posición/velocidad propias del interceptor. |
| 28 | `_target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");` | Declara el parámetro `target_frame` (por defecto `target/base_link`) y guarda su valor. |
| 30 | `_tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());` | Crea el buffer tf2, ligado al reloj del nodo. |
| 31 | `_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);` | Crea el listener que llena ese buffer escuchando tf2. |
| 33 | `_target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});` | Crea un timer que llama a `updateTargetPosition()` cada 50 ms. |
| 34 | `}` | Cierra el constructor. |
| 36 | `void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override` | Método que PX4 llama para decidir si el modo puede armarse; `override` confirma que sustituye al de `ModeBase`. |
| 37 | `{` | Abre el cuerpo del método. |
| 38 | `if (!_target_valid) {` | Si todavía no se ha recibido ninguna posición válida del target... |
| 39-42 | `reporter.armingCheckFailureExt(px4_ros2::events::ID("pursuit_no_target"), px4_ros2::events::Log::Error, "No target odometry received yet");` | ...informa a PX4 de un fallo de armado, con un identificador y un mensaje de error. |
| 43 | `}` | Cierra el `if`. |
| 44 | `}` | Cierra `checkArmingAndRunConditions`. |
| 46 | `void updateSetpoint(float dt_s) override` | Método que PX4 llama en cada ciclo del modo para pedir un nuevo setpoint. |
| 47 | `{` | Abre el cuerpo del método. |
| 48 | `(void)dt_s;` | Marca el parámetro `dt_s` como deliberadamente no usado (forma parte de la interfaz de `ModeBase`, pero este modo no lo necesita). |
| 50 | `if (!_target_valid) {` | Si aún no hay posición válida del target... |
| 51 | `RCLCPP_WARN(node().get_logger(), "Target position not valid yet. Skipping setpoint update.");` | ...registra un aviso... |
| 52 | `return;` | ...y sale sin calcular ningún setpoint. |
| 53 | `}` | Cierra el `if`. |
| 55 | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();` | Calcula la línea de visión (LOS): posición del target menos posición propia, ambas en NED. |
| 57 | `if (los.norm() < 1.0f) {` | Si la distancia al target es menor de 1 metro... |
| 58 | `RCLCPP_INFO(node().get_logger(), "Target reached. Stopping pursuit.");` | ...registra que se alcanzó el objetivo... |
| 59 | `completed(px4_ros2::Result::Success);` | ...avisa a PX4/la librería de que el modo terminó con éxito... |
| 60 | `return;` | ...y sale sin calcular más setpoints. |
| 61 | `}` | Cierra el `if`. |
| 63 | `const Eigen::Vector2f los_horizontal(los.x(), los.y());` | Toma solo las componentes X e Y de `los` (quita el eje vertical). |
| 64 | `Eigen::Vector2f velocity_horizontal = Eigen::Vector2f::Zero();` | Empieza la velocidad horizontal en cero por si no se recalcula. |
| 65 | `if (los_horizontal.norm() > kMinHorizontalDistance) {` | Si la distancia horizontal es mayor que el mínimo (`0.1 m`), para evitar normalizar un vector casi nulo... |
| 66 | `velocity_horizontal = los_horizontal.normalized() * kMaxHorizontalSpeed;` | ...la velocidad horizontal apunta hacia el target a la velocidad máxima (`5 m/s`). |
| 67 | `_last_yaw = atan2f(los_horizontal.y(), los_horizontal.x());` | Calcula el ángulo de rumbo (yaw) hacia el target y lo guarda como "último yaw válido". |
| 68 | `}` | Cierra el `if`. |
| 70 | `const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);` | Limita la componente vertical de `los` entre `-2` y `2 m/s`: esa es la velocidad vertical pedida. |
| 72 | `const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};` | Junta las tres componentes en un único vector de velocidad 3D. |
| 74 | `_trajectory_setpoint->update(velocity, {}, _last_yaw);` | Envía el setpoint a PX4: velocidad calculada, sin aceleración (`{}`) y el yaw guardado. |
| 75 | `}` | Cierra `updateSetpoint`. |
| 77 | `private:` | Lo siguiente solo es accesible dentro de la clase. |
| 78 | `void updateTargetPosition()` | Declara el método que el timer de la línea 33 llama cada 50 ms. |
| 79 | `{` | Abre el cuerpo del método. |
| 80 | `geometry_msgs::msg::TransformStamped t;` | Variable donde se guardará el resultado de la consulta tf2. |
| 82 | `try {` | Empieza el bloque que puede fallar si el frame del target aún no existe. |
| 83 | `t = _tf_buffer->lookupTransform(kMapFrame, _target_frame, tf2::TimePointZero);` | Pide la transformación `map -> target/base_link`, usando el último dato disponible. |
| 84 | `} catch (const tf2::TransformException & ex) {` | Si falla (el frame no existe todavía), se captura aquí. |
| 85-87 | `RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000, "Could not transform %s to %s: %s", ...);` | Registra un aviso, pero como máximo uno cada 5000 ms (5 s), para no inundar el log. |
| 88 | `return;` | Sale sin actualizar la posición del target. |
| 89 | `}` | Cierra el bloque `catch`. |
| 91-92 | `const Eigen::Vector3d position_enu(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);` | Arma un vector `double` con la traslación ENU recibida. |
| 93-94 | `const Eigen::Vector3d position_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(position_enu);` | Convierte esa posición de ENU a NED, para poder compararla con la odometría propia (que también está en NED). |
| 96 | `_target_position_ned = position_ned.cast<float>();` | Guarda la posición convertida como `float` (el atributo está declarado así). |
| 97 | `_target_valid = true;` | Marca que ya hay una posición válida del target; a partir de aquí `updateSetpoint` puede calcular. |
| 98 | `}` | Cierra `updateTargetPosition`. |
| 100 | `static constexpr float kMaxHorizontalSpeed = 5.0f;  // [m/s]` | Constante: velocidad horizontal máxima, 5 m/s. |
| 101 | `static constexpr float kMaxVerticalSpeed = 2.0f;    // [m/s]` | Constante: velocidad vertical máxima, 2 m/s. |
| 102 | `static constexpr float kMinHorizontalDistance = 0.1f;  // [m]` | Constante: distancia horizontal mínima para normalizar `los_horizontal`, 0.1 m. |
| 104 | `std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;` | Guarda el emisor de setpoints. |
| 105 | `std::shared_ptr<px4_ros2::OdometryLocalPosition> _own_position;` | Guarda el lector de la posición/velocidad propias. |
| 107 | `std::string _target_frame;` | Guarda el nombre del frame tf2 del target. |
| 108 | `std::unique_ptr<tf2_ros::Buffer> _tf_buffer;` | Guarda el buffer tf2. |
| 109 | `std::shared_ptr<tf2_ros::TransformListener> _tf_listener;` | Guarda el listener tf2. |
| 110 | `rclcpp::TimerBase::SharedPtr _target_lookup_timer;` | Guarda el timer de 50 ms. |
| 112 | `Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};` | Última posición NED conocida del target, inicializada a cero. |
| 113 | `bool _target_valid{false};` | Indica si ya se recibió al menos una posición válida del target. |
| 114 | `float _last_yaw{0.f};` | Último yaw calculado, para conservarlo cuando el target está casi encima. |
| 115 | `};` | Cierra la clase `PursuitMode`. |
| 117 | `using PursuitModeNode = px4_ros2::NodeWithMode<PursuitMode>;` | Define un alias: `PursuitModeNode` es "un `PursuitMode` envuelto como nodo ROS 2/PX4". |
| 119 | `static const std::string kNodeName = "pursuit_mode";` | Nombre del ejecutable/nodo que verá ROS 2. |
| 120 | `static const bool kEnableDebugOutput = true;` | Activa salida de depuración adicional de la librería `px4_ros2`. |
| 122 | `int main(int argc, char * argv[])` | Punto de entrada del programa. |
| 123 | `{` | Abre `main`. |
| 124 | `rclcpp::init(argc, argv);` | Inicializa ROS 2. |
| 125 | `rclcpp::spin(std::make_shared<PursuitModeNode>(kNodeName, kEnableDebugOutput));` | Crea el nodo con ese nombre y esa opción de depuración, y lo mantiene vivo procesando eventos (timer, `checkArmingAndRunConditions`, `updateSetpoint`...). |
| 126 | `rclcpp::shutdown();` | Libera los recursos de ROS 2 al terminar `spin`. |
| 127 | `return 0;` | Fin sin errores. |
| 128 | `}` | Cierra `main`. |

## 2. `PN_mode.cpp`

Archivo: [`PN_mode.cpp`](../../src/interceptor/src/PN_mode.cpp)

`PN_mode.cpp` comparte casi toda la estructura de `pursuit_mode.cpp` (mismos
includes, mismo patrón de constructor, mismo ciclo de vida). La tabla marca
con **★** las líneas que no tienen equivalente en `pursuit_mode.cpp`.

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1-10 | Comentario Doxygen `/** ... */` | Documenta que este archivo implementa Proportional Navigation (PN). |
| 12-23 | Mismos `#include` que `pursuit_mode.cpp` (líneas 1-12) | Idénticos, mismo propósito. |
| 24 | ★ `#include <geometry_msgs/msg/twist_stamped.hpp>` | Extra: trae `TwistStamped`, necesario porque PN sí necesita la velocidad del target. |
| 26 | `using namespace std::chrono_literals;  // NOLINT` | Igual que en Pursuit. |
| 28 | `static const std::string kName = "PN mode";` | Nombre del modo mostrado por PX4 (distinto texto que Pursuit). |
| 29 | `static const std::string kMapFrame = "map";` | Igual que en Pursuit. |
| 31 | `class PN_Mode : public px4_ros2::ModeBase` | Declara la clase `PN_Mode` (nombre distinto de `PursuitMode`), misma clase base. |
| 32 | `{` | Abre el cuerpo. |
| 33 | `public:` | Igual. |
| 34 | `explicit PN_Mode(rclcpp::Node & node)` | Constructor, igual patrón que Pursuit. |
| 35 | `: ModeBase(node, Settings{kName})` | Igual. |
| 36 | `{` | Abre el constructor. |
| 37 | `_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);` | Igual que en Pursuit. |
| 38 | `_own_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);` | Igual. |
| 40 | `_target_frame = node.declare_parameter<std::string>("target_frame", "target/base_link");` | Igual. |
| 41 | `_target_lookup_timer = node.create_wall_timer(50ms, [this] {updateTargetPosition();});` | Igual, aunque aquí aparece antes en el archivo que en Pursuit; el orden de estas líneas no cambia su comportamiento. |
| 42-43 | ★ `_target_velocity_topic = node.declare_parameter<std::string>("target_velocity_topic", "target/velocity");` | Extra: declara el parámetro del tópico de velocidad, por defecto `target/velocity`. |
| 45 | ★ `// Subscribe to the target's velocity topic. ENU to NED conversion is done in the callback.` | Comentario explicativo. |
| 46-47 | ★ `_target_velocity_sub = node.create_subscription<geometry_msgs::msg::TwistStamped>(_target_velocity_topic, 10,` | Extra: crea la suscripción a `target/velocity`, con cola de 10 mensajes. |
| 48 | ★ `[this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {` | Lambda ejecutada con cada mensaje de velocidad recibido. |
| 49 | ★ `using px4_ros_com::frame_transforms::enu_to_ned_local_frame;` | Acorta el nombre de la función de conversión. |
| 51 | ★ `// Convert the received ENU velocity to NED frame` | Comentario explicativo. |
| 52-53 | ★ `Eigen::Vector3d velocity_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);` | Arma un vector `double` con la velocidad lineal ENU del mensaje `TwistStamped`. |
| 54 | ★ `Eigen::Vector3d velocity_ned = enu_to_ned_local_frame(velocity_enu);` | Convierte esa velocidad a NED. |
| 56 | ★ `_target_velocity_ned = velocity_ned.cast<float>(); // .cast to convert from double to float` | Guarda la velocidad convertida como `float`. |
| 58 | ★ `_target_velocity_valid = true;` | Marca que ya se recibió al menos una velocidad válida del target. |
| 59 | ★ `});` | Cierra la lambda y `create_subscription`. |
| 61 | `_tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());` | Igual que en Pursuit. |
| 62 | `_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);` | Igual. |
| 64 | `}` | Cierra el constructor. |
| 67 | `void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override` | Mismo método que en Pursuit. |
| 68 | `{` | Abre el cuerpo. |
| 69 | ★ `if (!_target_valid` OR `!_target_velocity_valid) {` | **Diferencia clave**: aquí se exige posición *y* velocidad válidas (la condición usa el operador "o" de C++ entre ambas negaciones); Pursuit solo exige posición. |
| 70-73 | `reporter.armingCheckFailureExt(...)` | Igual que en Pursuit, mismo mensaje de error. |
| 74 | `}` | Cierra el `if`. |
| 75 | `}` | Cierra el método. |
| 78 | `void updateSetpoint(float dt_s) override` | Igual firma que en Pursuit. |
| 79 | `{` | Abre el cuerpo. |
| 80 | `(void)dt_s;` | Igual: parámetro no usado. |
| 82-85 | `if (!_target_valid) { RCLCPP_WARN(...); return; }` | Igual que en Pursuit. |
| 87 | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();` | Igual que en Pursuit: línea de visión. |
| 88 | ★ `const Eigen::Vector3f v_rel = _target_velocity_ned - _own_position->velocityNed();` | Extra: velocidad relativa = velocidad del target menos velocidad propia. |
| 90-94 | `if (los.norm() < 1.0f) { ...; completed(Success); return; }` | Igual que en Pursuit: termina si la distancia es menor de 1 m. |
| 96 | ★ `Eigen::Vector3f a_cmd = Eigen::Vector3f::Zero();` | Extra: la aceleración de navegación empieza en cero. |
| 99 | ★ `if (los.norm() < kPnMinRange) {` | Si la distancia es menor que `kPnMinRange` (7 m)... |
| 100 | ★ `a_cmd = Eigen::Vector3f::Zero();` | ...no se aplica aceleración PN (demasiado cerca para que la fórmula sea fiable). |
| 101 | ★ `} else {` | Si la distancia es mayor o igual a `kPnMinRange`... |
| 102-103 | ★ `const Eigen::Vector3f los_rotation_rate = los.cross(v_rel) / (los.squaredNorm() + 1e-6f);` | Calcula ω, la velocidad de giro de la línea de visión; el `1e-6f` evita dividir exactamente por cero. |
| 104 | ★ `a_cmd = kNavigationConstant * los_rotation_rate.cross(v_rel);` | Aplica la fórmula de navegación proporcional: aceleración proporcional a ω × velocidad relativa. |
| 105 | ★ `const float a_cmd_norm = a_cmd.norm();` | Calcula el tamaño de esa aceleración. |
| 107 | ★ `if (a_cmd_norm > 1e-6f) {` | Si la aceleración calculada no es prácticamente cero... |
| 108 | ★ `a_cmd = a_cmd.normalized() * std::min(a_cmd_norm, kMaxAcceleration);` | ...se normaliza y se limita a `kMaxAcceleration` (3 m/s²) como máximo. |
| 109 | ★ `} else {` | Si es prácticamente cero... |
| 110 | ★ `a_cmd = Eigen::Vector3f::Zero();` | ...se deja en cero exacto (evita normalizar un vector casi nulo). |
| 111 | ★ `}` | Cierra el `if`/`else` interior. |
| 112 | ★ `}` | Cierra el `if`/`else` de `kPnMinRange`. |
| 114-119 | `los_horizontal`, `velocity_horizontal`, `_last_yaw = atan2f(...)` | Igual patrón que en Pursuit para la velocidad de persecución horizontal. |
| 121 | `const float velocity_z = std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed);` | Igual que en Pursuit. |
| 123 | `const Eigen::Vector3f velocity{velocity_horizontal.x(), velocity_horizontal.y(), velocity_z};` | Igual que en Pursuit. |
| 125 | ★ `_trajectory_setpoint->update(velocity, a_cmd, _last_yaw);` | **Diferencia clave**: aquí sí se envía `a_cmd` (Pursuit envía `{}`, sin aceleración). |
| 126 | `}` | Cierra `updateSetpoint`. |
| 129-149 | `updateTargetPosition()` completo | Idéntico línea por línea al de `pursuit_mode.cpp` (líneas 78-98 de esa tabla): mismo `lookupTransform`, mismo `catch`, misma conversión ENU→NED. |
| 151 | `static constexpr float kMaxHorizontalSpeed = 7.0f;  // [m/s]` | **Valor distinto**: 7 m/s en vez de 5 m/s. |
| 152 | `static constexpr float kMaxVerticalSpeed = 2.0f;    // [m/s]` | Igual valor que en Pursuit. |
| 153 | `static constexpr float kMinHorizontalDistance = 0.1f;  // [m]` | Igual valor que en Pursuit. |
| 155 | ★ `static constexpr float kNavigationConstant = 3.5f;  // Proportional navigation constant (N/lambda)` | Extra: ganancia de la navegación proporcional. |
| 156 | ★ `static constexpr float kMaxAcceleration = 3.0f;  // [m/s^2]` | Extra: aceleración máxima permitida. |
| 157 | ★ `static constexpr float kPnMinRange = 7.0f;  // [m] Minimum range for PN to be active` | Extra: distancia mínima para activar la aceleración PN. |
| 159-165 | Miembros `_trajectory_setpoint`, `_own_position`, `_target_frame`, `_tf_buffer`, `_tf_listener`, `_target_lookup_timer` | Idénticos a los de Pursuit. |
| 167 | `Eigen::Vector3f _target_position_ned{Eigen::Vector3f::Zero()};` | Igual que en Pursuit. |
| 168 | ★ `Eigen::Vector3f _target_velocity_ned{Eigen::Vector3f::Zero()};` | Extra: última velocidad NED conocida del target. |
| 169 | ★ `std::string _target_velocity_topic;` | Extra: nombre del tópico de velocidad. |
| 170 | ★ `rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _target_velocity_sub;` | Extra: guarda la suscripción de velocidad. |
| 172 | ★ `bool _target_velocity_valid{false};` | Extra: indica si ya llegó una velocidad válida del target. |
| 173 | `bool _target_valid{false};` | Igual que en Pursuit. |
| 174 | `float _last_yaw{0.f};` | Igual que en Pursuit. |
| 175 | `};` | Cierra la clase `PN_Mode`. |
| 177 | `using PN_ModeNode = px4_ros2::NodeWithMode<PN_Mode>;` | Mismo patrón que `PursuitModeNode`, para esta clase. |
| 179 | `static const std::string kNodeName = "PN_mode";` | Nombre distinto de ejecutable/nodo. |
| 180 | `static const bool kEnableDebugOutput = true;` | Igual que en Pursuit. |
| 182-188 | `main` completo | Idéntico patrón a `pursuit_mode.cpp` (líneas 122-128 de esa tabla), usando `PN_ModeNode` en vez de `PursuitModeNode`. |

Comparar estas dos tablas línea a línea es la forma más directa de ver
exactamente qué añade PN sobre Pursuit: la suscripción de velocidad
(líneas 42-59), `v_rel` (línea 88), el cálculo de `a_cmd` (líneas 96-112) y
que `a_cmd` sí viaja en el `update()` final (línea 125). Todo lo demás —
estructura de la clase, `checkArmingAndRunConditions`, `updateTargetPosition`,
`main`— es el mismo patrón con nombres distintos.

## Cómo relacionar líneas con comportamiento observable

Una explicación de una línea es más útil cuando se puede comprobar en ejecución.
Usa esta tabla como puente entre código y herramientas:

| Código que estudias | Observación que puedes hacer |
| --- | --- |
| `create_subscription` | `ros2 node info` muestra la suscripción. |
| `create_publisher` | `ros2 topic info` muestra el publisher. |
| `create_wall_timer` | La frecuencia se aprecia con `ros2 topic hz` si el timer publica. |
| `sendTransform` | `tf2_echo` puede consultar el frame generado. |
| `lookupTransform` | Los warnings aparecen si el frame aún no existe. |
| `TrajectorySetpointType::update` | El modo produce referencias que PX4 consume. |
| `armingCheckFailureExt` | El modo informa de por qué aún no está listo para armar. |
| `completed(Success)` | El modo comunica que ha alcanzado el criterio de finalización. |

Este método evita leer el código como texto aislado: cada bloque tiene una
consecuencia en el grafo ROS 2, en el árbol tf2 o en el estado del modo PX4.

## Ejemplo de lectura de una callback

Para leer una callback de odometría, separa mentalmente sus operaciones:

1. **Entrada**: ¿qué tipo de mensaje recibe y quién lo publica?
2. **Extracción**: ¿qué campos se leen (`position`, `velocity`, `q`)?
3. **Conversión**: ¿cambia de frame o de tipo numérico?
4. **Almacenamiento**: ¿se guarda para otro timer o método?
5. **Salida**: ¿publica un tópico, un transform o un log?

En `target_tf2_odometry`, por ejemplo, una misma entrada produce dos salidas
distintas: el transform se publica inmediatamente y la velocidad convertida se
guarda para el timer de 100 ms. Esta separación explica por qué hay dos ritmos
de ejecución.

## Qué no debe inferirse de una línea

Una línea que crea una suscripción no demuestra que haya mensajes. Una línea que
declara un publisher no garantiza que exista un subscriber. Una conversión
matemática correcta tampoco garantiza que los datos tengan timestamps válidos.
Siempre hay que combinar:

- lo que declara el código;
- lo que muestra `ros2 node/topic`;
- la frecuencia real;
- los logs;
- y, en el caso de tf2, el árbol de frames.

La comprensión completa aparece al comparar esas cinco fuentes.

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md) · ➡️ Siguiente: [Mapa del workspace y dependencias](Workspace-file-map.md)
