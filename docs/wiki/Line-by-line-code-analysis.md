# Análisis línea por línea del código propio

Esta página es la referencia detallada para estudiar **todos los archivos de
`src/interceptor` y `.github`**. No analiza línea por línea las librerías de
terceros, tal como se decidió: esas carpetas se documentan en el
[mapa del workspace](Workspace-file-map.md), pero su código pertenece a sus
proyectos upstream.

## Cómo leer una explicación técnica

Una línea de código puede parecer una frase extraña porque combina palabras,
signos y tipos. Léela en este orden:

1. Busca la acción principal, normalmente el nombre que aparece antes de `(`.
2. Mira los valores entre paréntesis: son los datos que recibe esa acción.
3. Comprueba el tipo situado antes del nombre: indica qué clase de dato se
   guarda o devuelve.
4. Busca `=` para distinguir “crear o calcular algo” de “usar algo existente”.
5. Si aparece `->` o `.`, significa “acceder a algo que pertenece a un objeto”.

Por ejemplo, `msg->velocity[0]` se puede leer como: “del mensaje llamado `msg`,
entra en el campo `velocity` y toma su primera posición”. Los índices empiezan
en cero en C++, de modo que `[0]`, `[1]` y `[2]` son los tres componentes.

No es necesario dominar todos los signos de una vez. La explicación debajo de
cada bloque indica qué dato entra, qué transformación se realiza y qué resultado
sale.

## Cómo usar este análisis

- Los números de línea corresponden al estado actual de cada archivo.
- Las líneas en blanco separan bloques y no ejecutan ninguna operación.
- Cuando varias líneas forman una única instrucción CMake, XML, Python o C++ se
  explican juntas, indicando qué hace cada una.
- Los comentarios no cambian el programa; explican intención para quien lee.
- Para comprobar el texto exacto, abre el enlace al archivo fuente de cada
  sección.

## 1. `src/interceptor/.gitignore`

Archivo: [`src/interceptor/.gitignore`](../../src/interceptor/.gitignore)

Cada patrón indica a Git qué archivos locales no debe incluir:

| Línea | Explicación |
| --- | --- |
| `build/` | Ignora artefactos de compilación de CMake/colcon. |
| `install/` | Ignora la instalación generada por colcon. |
| `log/` | Ignora logs generados por ROS 2. |
| `*.o` | Ignora objetos compilados de C/C++. |
| `*.so` | Ignora bibliotecas compartidas compiladas. |
| `__pycache__/` | Ignora bytecode generado por Python. |

El archivo evita subir resultados reproducibles de una compilación. No evita
subir archivos fuente ni cambia el comportamiento del paquete.

## 2. `src/interceptor/LICENSE`

Archivo: [`src/interceptor/LICENSE`](../../src/interceptor/LICENSE)

| Línea/conjunto | Qué significa |
| --- | --- |
| `<license>Proprietary` | Declara que el código es propietario, no una licencia open source estándar. |
| `All rights reserved [...]` | Reserva los derechos a los autores indicados. |
| `This software ... proprietary and confidential.` | Extiende esa protección al software y su documentación. |
| `Unauthorized copying ... prohibited ...` | Prohíbe copiar, distribuir, modificar o usar sin permiso escrito previo. |
| `</license>` | Cierra el elemento XML que envuelve el texto legal. |

Este texto es legal, no código ejecutable. La etiqueta XML es una particularidad
del archivo actual; `package.xml` también declara una licencia propietaria.

## 3. `src/interceptor/README.md`

Archivo: [`src/interceptor/README.md`](../../src/interceptor/README.md)

### Identidad y requisitos

| Línea/conjunto | Explicación |
| --- | --- |
| `# interceptor` | Título Markdown del paquete. |
| Párrafo inicial | Resume que es un paquete ROS 2 `ament_cmake` para seguir e interceptar un target mediante PX4. |
| `## Requisitos` | Comienza la lista de software necesario. |
| `Ubuntu 24.04`, `ROS2 Jazzy`, `PX4 toolchain` | Define el sistema operativo, distribución ROS 2 y herramientas PX4 esperadas. |

### Dependencias y preparación

La sección `Dependencias` explica que el workspace debe contener `px4_msgs`,
`px4_ros_com` y `px4-ros2-interface-lib`. Los enlaces llevan a sus repositorios
oficiales; no descargan nada automáticamente.

El bloque:

```bash
mkdir -p ~/ws_interceptor/src
cd ~/ws_interceptor/src
```

crea la carpeta del workspace y entra en su directorio `src`. Los cuatro
comandos `git clone` descargan el repositorio propio y las tres dependencias.
El alias local `interceptor` solo es el nombre de la carpeta del repositorio
principal.

### Instalación, compilación y ejecución

| Línea/comando | Qué hace |
| --- | --- |
| `cd ~/ws_interceptor` | Vuelve a la raíz del workspace. |
| `source /opt/ros/jazzy/setup.bash` | Añade ROS 2 Jazzy al entorno de la terminal. |
| `rosdep update` | Actualiza el índice de dependencias del sistema. |
| `rosdep install --from-paths src --ignore-src -r -y` | Instala dependencias descritas por paquetes de `src`, sin reinstalar paquetes fuente. |
| `colcon build --symlink-install` | Compila todos los paquetes y usa enlaces simbólicos para facilitar desarrollo. |
| `source install/setup.bash` | Hace visibles los paquetes recién compilados. |
| `colcon build --packages-up-to interceptor --symlink-install` | Compila `interceptor` y solo sus dependencias. |
| `ros2 launch interceptor interceptor.launch.py` | Ejecuta el escenario descrito por el launch. |
| `ros2 run interceptor pursuit_mode` | Ejecuta solo un binario del paquete. |

La última sección enlaza documentación oficial para que una persona pueda
investigar ROS 2, PX4 y las dependencias sin confundirlas con este código.

## 4. `src/interceptor/package.xml`

Archivo: [`src/interceptor/package.xml`](../../src/interceptor/package.xml)

| Línea | Explicación |
| --- | --- |
| `<?xml version="1.0"?>` | Declara que el documento usa XML 1.0. |
| `<?xml-model ... package_format3.xsd ...?>` | Permite validar el documento contra el esquema de paquetes ROS 2. |
| `<package format="3">` | Abre un paquete con el formato 3. |
| `<name>interceptor</name>` | Nombre que usan `ros2 run`, `ros2 launch` y colcon. |
| `<version>0.1.1</version>` | Versión declarada del paquete. |
| `<description>...</description>` | Descripción breve para herramientas ROS 2. |
| `<maintainer email=...>deireb</maintainer>` | Persona responsable y correo de mantenimiento. |
| `<license>...</license>` | Nombre de la licencia declarada. |
| `<buildtool_depend>ament_cmake</buildtool_depend>` | Indica que CMake/ament construye el paquete. |

Las etiquetas `<depend>` de `rclcpp`, `tf2_ros`, `tf2`, `geometry_msgs`,
`px4_msgs`, `px4_ros_com`, `px4_ros2_cpp` y `sensor_msgs` declaran dependencias
usadas al compilar y ejecutar. Cada nombre debe corresponder a un paquete ROS 2
que `find_package` pueda localizar.

`<exec_depend>launch</exec_depend>` y `<exec_depend>launch_ros</exec_depend>`
son necesarias al ejecutar el archivo Python de launch. Las dos
`<test_depend>` se usan solo para lint/tests. `<export>` y
`<build_type>ament_cmake</build_type>` indican a ROS 2 cómo construir el
paquete. `</package>` cierra el documento.

## 5. `src/interceptor/CMakeLists.txt`

Archivo: [`src/interceptor/CMakeLists.txt`](../../src/interceptor/CMakeLists.txt)

### Proyecto y compilador, líneas 1-6

| Código | Explicación |
| --- | --- |
| `cmake_minimum_required(VERSION 3.8)` | Rechaza versiones de CMake demasiado antiguas. |
| `project(interceptor)` | Define el nombre del proyecto CMake. |
| `if(CMAKE_COMPILER_IS_GNUCXX OR ... Clang)` | Entra si el compilador es GCC o Clang. |
| `add_compile_options(-Wall -Wextra -Wpedantic)` | Activa advertencias sobre errores y construcciones no portables. |
| `endif()` | Cierra la condición. |

### Dependencias, líneas 8-17

Cada `find_package(NOMBRE REQUIRED)` busca un paquete y hace fallar la
configuración si no existe:

- `ament_cmake`: integración CMake/ROS 2.
- `rclcpp`: API C++ de ROS 2.
- `tf2_ros` y `tf2`: transformaciones y excepciones.
- `geometry_msgs`: transformaciones, twists y mensajes geométricos.
- `px4_msgs`: `VehicleOdometry`.
- `px4_ros_com`: conversiones de marcos.
- `px4_ros2_cpp`: modos y setpoints PX4.
- `sensor_msgs`: dependencia declarada por los nodos.

### Construcción de cada ejecutable

Cada pareja `add_executable` + `ament_target_dependencies` hace dos cosas:

1. Asocia un nombre de comando a un `.cpp`.
2. Asocia las bibliotecas que ese binario necesita.

| Ejecutable | Fuente | Dependencias particulares |
| --- | --- | --- |
| `target_vehicle_odometry_subscriber` | `src/target_vehicle_odometry_subscriber.cpp` | ROS 2, tf2, geometría, mensajes PX4 y sensores. |
| `interceptor_vehicle_odometry_subscriber` | `src/interceptor_vehicle_odometry_subscriber.cpp` | Igual que el anterior. |
| `interceptor_tf2_odometry` | `src/interceptor_tf2_odometry.cpp` | Añade `px4_ros_com` para convertir frames. |
| `target_tf2_odometry` | `src/target_tf2_odometry.cpp` | Igual, además publica velocidad. |
| `tf2_listener` | `src/tf2_listener.cpp` | Buffer/listener tf2 y odometría PX4. |
| `pursuit_mode` | `src/pursuit_mode.cpp` | `px4_ros2_cpp` y transformaciones. |
| `PN_mode` | `src/PN_mode.cpp` | Igual que pursuit y mensajes geométricos. |

Las líneas con los nombres repetidos dentro de `ament_target_dependencies` no
son código duplicado accidental: indican al linker qué bibliotecas usa cada
target.

### Tests e instalación

Dentro de `if(BUILD_TESTING)`:

- `find_package(ament_lint_auto REQUIRED)` localiza el sistema de lint.
- Las dos asignaciones `ament_cmake_*_FOUND = TRUE` desactivan copyright y
  cpplint, siguiendo los comentarios del propio archivo.
- `ament_lint_auto_find_test_dependencies()` registra los tests automáticos.

`install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` instala los siete
binarios en la carpeta estándar del paquete. `install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})` instala el launch. Finalmente,
`ament_package()` genera metadatos para que ROS 2 encuentre el paquete.

## 6. `src/interceptor/launch/interceptor.launch.py`

Archivo: [`interceptor.launch.py`](../../src/interceptor/launch/interceptor.launch.py)

### Cabecera y imports, líneas 1-13

| Línea/bloque | Explicación |
| --- | --- |
| `#!/usr/bin/env python` | Permite ejecutar el archivo como script Python. |
| Docstring triple | Documenta el escenario, el agente y qué PX4 se arranca manualmente. |
| `import os` | Importa funciones del sistema operativo. |
| `from launch import LaunchDescription` | Importa el contenedor de acciones. |
| `ExecuteProcess` | Permite iniciar un comando externo. |
| `Node` | Permite iniciar un nodo ROS 2. |
| `MICRO_XRCE_DDS_AGENT_DIR = ...` | Expande `~` y guarda la carpeta del agente. |

### `generate_launch_description`

La función sin argumentos es el punto de entrada que ROS 2 busca. Crea
`micro_xrce_agent` con:

- `cmd=[['./build/MicroXRCEAgent udp4 -p 8888']]`: comando del agente UDP.
- `shell=True`: ejecuta la cadena mediante shell.
- `cwd=...`: cambia al directorio esperado.
- `output='log'`: manda su salida a los logs.

Cada bloque `Node(...)` especifica `package='interceptor'`,
`executable='...'` y `output`. Los seis bloques activos arrancan dos
suscriptores de diagnóstico, dos conversores tf2 y dos modos. El bloque de
`tf2_listener` empieza por `#`, así que Python lo trata como comentario y no lo
ejecuta.

La lista de `LaunchDescription` devuelve primero el agente y luego los nodos.
Esto no arranca PX4 SITL: los procesos PX4 se ejecutan en terminales separadas.

## 7. Patrón de los suscriptores de odometría

Archivos:

- [`target_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/target_vehicle_odometry_subscriber.cpp)
- [`interceptor_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/interceptor_vehicle_odometry_subscriber.cpp)

Son casi iguales; cambia el nombre del nodo, el mensaje inicial y el tópico.

### Includes y clase

| Línea/bloque | Explicación |
| --- | --- |
| Comentario Doxygen inicial | Describe archivo, propósito y autor para documentación. |
| `#include <rclcpp/rclcpp.hpp>` | Importa Node, init, spin y ROS 2. |
| `#include <px4_msgs/msg/vehicle_odometry.hpp>` | Importa el tipo del mensaje PX4. |
| `class VehicleOdometrySubscriber : public rclcpp::Node` | Declara un nodo especializado. |
| Constructor `: Node("...")` | Registra el nombre visible del nodo. |

### QoS, suscripción y callback

`rmw_qos_profile_sensor_data` selecciona un perfil apropiado para sensores.
`QoSInitialization(..., 5)` conserva una profundidad de cinco mensajes.

`create_subscription<VehicleOdometry>(topic, qos, lambda)` registra:

1. el tipo de mensaje;
2. el tópico;
3. la calidad de servicio;
4. la función que se ejecuta al recibir datos.

La lambda imprime timestamp, `pose_frame`, tres posiciones, cuatro componentes
del cuaternión, tres velocidades y la magnitud calculada con la raíz cuadrada
de `vx² + vy² + vz²`. Las muchas líneas `std::cout << "\n"` limpian
visualmente la consola antes de cada muestra.

El `private` contiene el `SharedPtr` de la suscripción para mantenerla viva.
En `main`, `std::cout` anuncia el nodo, `setvbuf` desactiva buffering de salida,
`rclcpp::init` inicializa ROS 2, `spin` procesa mensajes y `shutdown` cierra.

### Diferencia real de los dos archivos

Aunque los nombres sugieren una cosa, actualmente el archivo `target_...`
escucha `/fmu/out/vehicle_odometry` (instancia 0), mientras que el archivo
`interceptor_...` escucha `px4_1/fmu/out/vehicle_odometry` (instancia 1).
Esto se documenta deliberadamente para que nadie aprenda una asociación falsa
entre nombre de archivo y tópico.

## 8. `interceptor_tf2_odometry.cpp`

Archivo: [`interceptor_tf2_odometry.cpp`](../../src/interceptor/src/interceptor_tf2_odometry.cpp)

### Includes, comentarios y clase

`memory`, `sstream` y `string` proporcionan punteros, construcción de tópicos y
cadenas. `rclcpp` crea el nodo; `TransformStamped` representa el transform;
`VehicleOdometry` es la entrada PX4; `frame_transforms` convierte NED/ENU;
`TransformBroadcaster` publica tf2.

`FramePublisher : public rclcpp::Node` crea un nodo. En el constructor,
`Node("interceptor_tf2_frame_publisher")` asigna su nombre.

### Constructor línea por línea

| Instrucción | Explicación |
| --- | --- |
| `declare_parameter<string>("vehicle_name", "interceptor")` | Declara nombre configurable del vehículo. |
| `make_unique<TransformBroadcaster>(*this)` | Crea el broadcaster ligado al nodo. |
| `rmw_qos_profile_sensor_data` | Selecciona QoS de sensor. |
| `QoSInitialization(..., 5)` | Convierte ese perfil en QoS de ROS 2 con profundidad 5. |
| `ostringstream stream` | Crea un constructor de texto. |
| `stream << "/fmu/out/vehicle_odometry"` | Forma el tópico de la instancia 0. |
| `subscription_ = create_subscription(...)` | Empieza a recibir odometría. |

La lambda captura `this` y recibe un `UniquePtr`. Los `using` acortan los nombres
de dos funciones de conversión. Construye `position_ned` desde los tres campos
del mensaje y la transforma a `position_enu`. Construye el cuaternión PX4 en el
orden `(w,x,y,z)` y lo adapta al convenio ROS.

`TransformStamped t` se rellena con la hora actual, padre `map`, hijo
`vehicle_name_/base_link`, traslación ENU y rotación ENU. `sendTransform(t)`
publica el frame. Los atributos privados guardan la suscripción, el broadcaster
y el nombre para que sigan existiendo mientras vive el nodo.

El `main` imprime un aviso, desactiva buffering, inicializa ROS 2, crea
`FramePublisher`, entra en `spin` y finalmente llama a `shutdown`.

## 9. `target_tf2_odometry.cpp`

Archivo: [`target_tf2_odometry.cpp`](../../src/interceptor/src/target_tf2_odometry.cpp)

Este archivo repite la estructura del conversor anterior, pero tiene tres
diferencias funcionales:

1. El nombre de nodo por defecto es `target_tf2_frame_publisher`.
2. Escucha `/px4_1/fmu/out/vehicle_odometry`.
3. Publica `target/velocity`.

`create_publisher<TwistStamped>("target/velocity", 10)` crea el publisher con
cola de diez mensajes. La lambda del timer se ejecuta cada 100 ms: crea el
mensaje, asigna timestamp y `frame_id`, copia las tres componentes de
`_target_velocity_enu` y publica.

La callback de odometría transforma posición y velocidad de NED a ENU, convierte
la velocidad double a float con `.cast<float>()`, rellena el mismo
`TransformStamped` y lo publica. El timer y la suscripción comparten la última
velocidad guardada en `_target_velocity_enu`.

Los miembros privados son la suscripción, broadcaster, publisher, vector de
velocidad inicializado a cero, timer y nombre. Inicializar a cero evita leer
basura si el timer dispara antes del primer mensaje PX4.

## 10. `tf2_listener.cpp`

Archivo: [`tf2_listener.cpp`](../../src/interceptor/src/tf2_listener.cpp)

### Preparación

Los includes estándar aportan duraciones, funciones, memoria y strings.
`TransformStamped` representa el resultado; `VehicleOdometry` proporciona
velocidad; `rclcpp` crea el nodo; `tf2` aporta excepciones; buffer/listener
reciben transforms.

`using namespace std::chrono_literals` permite escribir `1s`. El constructor
crea el nodo `tf2_frame_listener`, declara `target_frame` con valor
`target/base_link`, construye el buffer con el reloj y conecta el listener.

### Suscripción y timer

La suscripción a `/px4_1/fmu/out/vehicle_odometry` guarda las tres velocidades
en `target_velocity_`. El timer llama a `on_timer` cada segundo mediante
`std::bind(&FrameListener::on_timer, this)`.

### `on_timer`

`fromFrameRel` es el frame del target y `toFrameRel` el del interceptor.
`lookupTransform(toFrameRel, fromFrameRel, TimePointZero)` pide la última
transformación disponible. Si falta un frame, el `catch` imprime el error y
`return` evita usar datos inválidos.

Si funciona, `RCLCPP_INFO` imprime traslación y velocidad. Los atributos guardan
timer, suscripción, array de velocidad, listener, buffer y parámetro. `main`
usa el mismo ciclo init-spin-shutdown que los demás nodos.

## 11. `pursuit_mode.cpp`

Archivo: [`pursuit_mode.cpp`](../../src/interceptor/src/pursuit_mode.cpp)

### Includes, constantes y clase

`Eigen` aporta álgebra vectorial; los headers `px4_ros2` aportan modos,
setpoints y odometría; `frame_transforms` convierte ENU/NED; `tf2` aporta
listener y excepciones; `rclcpp` aporta nodo y logging.

`kName` es el texto del modo, `kMapFrame` es el padre tf2 y las duraciones
literales permiten usar `50ms`. `PursuitMode` hereda de `ModeBase`, por lo que
PX4 puede registrarlo y llamar a sus métodos virtuales.

### Constructor

`ModeBase(node, Settings{kName})` inicializa la clase base con la configuración.
`make_shared<TrajectorySetpointType>` prepara la salida de trayectoria y
`OdometryLocalPosition` la posición propia. El parámetro `target_frame` permite
cambiar el frame; buffer y listener llenan la caché tf2. El timer llama
`updateTargetPosition` cada 50 ms.

### Seguridad y setpoint

`checkArmingAndRunConditions` comprueba `_target_valid`. Si es falso, informa a
PX4 de un fallo de armado con un identificador y mensaje.

En `updateSetpoint`, `(void)dt_s` marca el parámetro de interfaz como no usado.
Si no hay target, registra un warning y sale. `los` resta posición propia a la
del target. Si su norma es menor que 1 m, registra éxito y termina.

`los_horizontal` elimina el eje vertical. Si supera `0.1 m`, se normaliza y se
multiplica por `5 m/s`; `atan2f` apunta el yaw hacia el target. `std::clamp`
limita la velocidad vertical a ±`2 m/s`. El vector final se envía con velocidad,
aceleración vacía y yaw.

### `updateTargetPosition`

El método intenta consultar `map -> target/base_link`. El `catch` registra un
warning como máximo cada 5 segundos. Si hay transform, toma sus tres
traslaciones ENU, las convierte a NED, cambia double a float, guarda la posición
y marca `_target_valid`.

Las constantes `kMaxHorizontalSpeed`, `kMaxVerticalSpeed` y
`kMinHorizontalDistance` hacen visible el ajuste del algoritmo. Los miembros
privados guardan publishers/lectores, estado del target y yaw anterior.

`PursuitModeNode` es un alias de `NodeWithMode<PursuitMode>`. `main` inicializa
ROS 2, crea ese wrapper con nombre `pursuit_mode` y activa salida de depuración.

## 12. `PN_mode.cpp`

Archivo: [`PN_mode.cpp`](../../src/interceptor/src/PN_mode.cpp)

PN repite la estructura de pursuit para posición, tf2, setpoint y ciclo de vida.
Añade `geometry_msgs/msg/twist_stamped.hpp` porque necesita velocidad del target.

### Suscripción de velocidad

El parámetro `target_velocity_topic` tiene por defecto `target/velocity`.
`create_subscription<TwistStamped>` recibe el mensaje, construye un vector ENU,
lo convierte a NED y lo guarda en `_target_velocity_ned`. Después marca
`_target_velocity_valid = true`.

`checkArmingAndRunConditions` usa la condición combinada
`!_target_valid || !_target_velocity_valid`; PN no se puede armar sin ambos
datos.

### Cálculo PN

`los` es la posición relativa y `v_rel` es velocidad target menos velocidad
propia. Si la distancia es menor que 1 m, el modo termina.

`a_cmd` empieza en cero. Si la distancia es menor que `kPnMinRange` (7 m), se
mantiene cero. En caso contrario:

1. `los.cross(v_rel) / (los.squaredNorm() + 1e-6f)` calcula la tasa de giro de
   la línea de visión evitando división exacta por cero.
2. `kNavigationConstant * los_rotation_rate.cross(v_rel)` calcula aceleración.
3. Si la norma es significativa, normaliza y limita a `3 m/s²`.
4. Si es casi cero, conserva el vector cero.

Después PN calcula la velocidad de persecución horizontal/vertical igual que
Pursuit, pero con `7 m/s` horizontales. `update(velocity, a_cmd, _last_yaw)`
envía velocidad, aceleración y yaw.

Los miembros adicionales son la velocidad NED, el tópico y la suscripción, más
la bandera de validez. `PN_ModeNode` y `main` hacen el mismo wrapping que
Pursuit, con nombre `PN_mode`.

## 13. Workflows de `.github`

### `.github/workflows/ci-build.yml`

Archivo: [`ci-build.yml`](../../.github/workflows/ci-build.yml)

| Línea/bloque | Explicación |
| --- | --- |
| `name: CI - Build ROS 2 workspace` | Nombre visible en GitHub Actions. |
| `on: push` | Ejecuta CI al subir cambios. |
| `branches: [main, master]` | Solo para esas ramas. |
| `pull_request` | Ejecuta CI en pull requests hacia esas ramas. |
| `jobs: build` | Define un job llamado `build`. |
| `runs-on: ubuntu-24.04` | Usa runner Ubuntu 24.04. |
| `container: ros:jazzy-ros-base` | Ejecuta pasos dentro de una imagen ROS 2 Jazzy. |
| `actions/checkout@v4` | Descarga el repositorio al runner. |
| `apt-get update` | Actualiza índices de paquetes Debian. |
| `apt-get install ...` | Instala colcon, rosdep y compilador. |
| `rosdep init || true` | Inicializa rosdep; `true` evita fallar si ya estaba inicializado. |
| `rosdep update` | Actualiza el índice de rosdep. |
| `rosdep install ...` | Instala dependencias de todos los paquetes fuente. |
| `source /opt/ros/jazzy/setup.bash` | Prepara el entorno ROS. |
| `colcon build --packages-up-to interceptor ...` | Compila el paquete objetivo y dependencias. |
| `colcon test --packages-select interceptor ...` | Ejecuta tests/lint del paquete propio. |
| `colcon test-result --verbose` | Muestra resultados y fallos detallados. |

La indentación YAML es significativa: `steps` contiene acciones, cada acción
tiene `name` y `run`, y el bloque `|` conserva comandos multilínea.

### `.github/workflows/summary.yml`

Archivo: [`summary.yml`](../../.github/workflows/summary.yml)

`name` identifica el workflow. `on: issues: types: [opened]` lo limita a issues
nuevos. `permissions` concede solo lectura de modelos/contenido y escritura de
issues.

El job corre en `ubuntu-latest`, hace checkout y ejecuta
`actions/ai-inference@v1`. `id: inference` permite referenciar su salida
`steps.inference.outputs.response`. El prompt usa el título y cuerpo del issue
como datos no confiables y ordena resumirlos, no obedecer instrucciones que
contengan.

La última acción ejecuta `gh issue comment`. Sus variables de entorno reciben el
token de GitHub, número del issue y respuesta generada. Por tanto, este workflow
publica automáticamente un comentario, no modifica el código.

### `.github/workflows/tagging.yml`

Archivo: [`tagging.yml`](../../.github/workflows/tagging.yml)

`workflow_run` espera a que termine el workflow cuyo nombre exacto es
`CI - Build ROS 2 workspace`, únicamente en `main`; `workflow_dispatch` permite
lanzarlo manualmente. El job solo continúa si fue manual o si CI terminó con
éxito.

`permissions: contents: write` permite crear tags/releases. `checkout` usa
`fetch-depth: 0` para disponer del historial completo. El action
`github-tag-action` calcula un tag semántico usando `GITHUB_TOKEN`, prefijo `v`
y bump por defecto `none`. Si produce `new_tag`, `action-gh-release` crea una
release, usa el tag como nombre y genera notas automáticamente.

## 14. Orden recomendado para revisar el código

1. Lee `.gitignore`, `README.md` y `package.xml` para conocer la estructura del paquete.
2. Lee `CMakeLists.txt` y relaciona cada ejecutable con su `.cpp`.
3. Lee primero los suscriptores de odometría: son callbacks sencillas.
4. Continúa con los conversores tf2 y verifica sus tópicos.
5. Estudia `pursuit_mode.cpp` antes de `PN_mode.cpp`.
6. Lee los workflows para ver cómo el proyecto comprueba y publica cambios.
7. Usa el [mapa completo](Workspace-file-map.md) para distinguir código propio
   de dependencias.

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

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Lectura guiada del código](Code-walkthrough.md) · ➡️ Siguiente: [Mapa del workspace y dependencias](Workspace-file-map.md)
