# Análisis línea por línea: archivos de proyecto y construcción

Esta es la primera de tres páginas que cubren, línea por línea, **todos los
archivos propios de `src/interceptor`**. Esta página se ocupa de los archivos
de construcción, configuración y arranque del paquete. Las otras dos son:

- [Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md):
  los subscriptores de diagnóstico, los dos conversores tf2 y `tf2_listener`.
- [Modos de guiado, línea por línea](Line-by-line-guidance-modes.md):
  `pursuit_mode.cpp` y `PN_mode.cpp`.

No se analizan línea por línea las librerías de terceros, tal como se decidió:
esas carpetas se documentan en el [mapa del workspace](Workspace-file-map.md),
pero su código pertenece a sus proyectos upstream. Los workflows de
`.github` (CI, resúmenes de issues, releases) se documentan línea por línea
dentro del [mapa del workspace](Workspace-file-map.md), junto al resto de
automatización del repositorio.

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

## Sintaxis de C++ que vas a encontrar constantemente

[Code-walkthrough.md](Code-walkthrough.md) ya explica lo más básico (`{`, `;`,
`//`, clases, funciones, variables, lambdas, `shared_ptr`/`unique_ptr`). Las
tablas de esta página usan además una serie de construcciones de C++ más
específicas que no se explican en ningún otro sitio de la wiki. Si te
encuentras con alguna de ellas y no sabes qué significa, vuelve a esta tabla:

| Sintaxis | Qué significa | Ejemplo en este paquete |
| --- | --- | --- |
| `override` | Escrito después de un método, indica "esto sustituye a un método que ya existía en la clase de la que heredo". El compilador avisa si el nombre o los parámetros no coinciden exactamente con el original. | `void updateSetpoint(float dt_s) override` sustituye al `updateSetpoint` vacío que declara `px4_ros2::ModeBase`. |
| `explicit` | Delante de un constructor, impide que C++ lo use "a escondidas" para convertir un valor en un objeto de esa clase sin que el programador lo pida explícitamente. Es una medida de seguridad, no cambia lo que hace el constructor. | `explicit PursuitMode(rclcpp::Node & node)`. |
| `static constexpr` | Un valor constante que se calcula en tiempo de compilación (antes de ejecutar el programa) y que comparten todas las instancias de la clase; nunca cambia mientras el programa corre. | `static constexpr float kMaxHorizontalSpeed = 5.0f;`. |
| `static` fuera de una clase | Una variable o función `static` a nivel de archivo solo es visible dentro de ese `.cpp`; ningún otro archivo puede usarla aunque tenga el mismo nombre. | `static const std::string kName = "Pursuit Intercept";`. |
| `Plantilla<Tipo>` (por ejemplo `shared_ptr<Buffer>`) | Los `< >` después del nombre de una clase indican una plantilla: un molde genérico que se rellena con un tipo concreto. `shared_ptr<Buffer>` es "un puntero compartido, pero específicamente a un objeto `Buffer`", no a cualquier cosa. | `std::unique_ptr<tf2_ros::Buffer> _tf_buffer;`. |
| `Tipo & nombre` (referencia) | Recibe el objeto original, no una copia. Si la función modifica ese parámetro, el cambio se ve fuera de la función también. | `explicit PursuitMode(rclcpp::Node & node)`: el modo recibe el nodo real, no una copia. |
| `const Tipo & nombre` | Como el anterior, pero además promete que la función no va a modificar ese objeto: solo lo lee. | `catch (const tf2::TransformException & ex)`. |
| `const` delante de una variable | El valor no se puede cambiar después de crearse. | `const Eigen::Vector3f los = _target_position_ned - _own_position->positionNed();`. |
| `Espacio::Nombre` (`::`) | Se lee "dentro de". `std::string` es `string` dentro del espacio de nombres `std`; `px4_ros2::Result::Success` es el valor `Success`, dentro de `Result`, dentro de `px4_ros2`. | `px4_ros2::Result::Success`, `std::chrono::milliseconds(100)`. |
| `.cross(v)` | Producto vectorial entre dos vectores 3D: da como resultado otro vector, perpendicular a los dos originales, relacionado con cuánto y hacia dónde "giran" uno respecto al otro. | `los.cross(v_rel)` en `PN_mode.cpp`. |
| `.norm()` | La longitud (magnitud) de un vector. | `los.norm() < 1.0f`. |
| `.squaredNorm()` | La longitud al cuadrado. Se usa en vez de `.norm()` cuando no hace falta la raíz cuadrada exacta, porque calcularla es más rápido. | `los.squaredNorm() + 1e-6f`. |
| `.normalized()` | Devuelve el mismo vector pero con longitud 1, conservando la dirección. Sirve para quedarte solo con "hacia dónde apunta" un vector, sin su tamaño. | `los_horizontal.normalized() * kMaxHorizontalSpeed`. |
| `.cast<float>()` | Convierte los números de un vector/cuaternión de un tipo a otro (aquí, de `double` a `float`). | `position_ned.cast<float>()`. |
| `std::bind(&Clase::metodo, this)` | Empaqueta un método de un objeto concreto para poder pasarlo como si fuera una función suelta (por ejemplo, a un timer). | `std::bind(&FrameListener::on_timer, this)` en `tf2_listener.cpp`. |
| `std::clamp(valor, min, max)` | Si `valor` es menor que `min`, devuelve `min`; si es mayor que `max`, devuelve `max`; si no, devuelve `valor` tal cual. Limita un número a un rango. | `std::clamp(los.z(), -kMaxVerticalSpeed, kMaxVerticalSpeed)`. |
| `std::min(a, b)` | Devuelve el menor de los dos valores. | `std::min(a_cmd_norm, kMaxAcceleration)`. |
| `(void)nombre;` | Le dice al compilador "sé que no uso esta variable, es intencionado", para que no avise de "parámetro sin usar". | `(void)dt_s;` en `updateSetpoint`. |

## Cómo usar este análisis

- Los números de línea corresponden al estado actual de cada archivo.
- Las líneas en blanco separan bloques y no ejecutan ninguna operación.
- Cuando varias líneas forman una única instrucción CMake, XML, Python o C++ se
  explican juntas, indicando qué hace cada una.
- Los comentarios no cambian el programa; explican intención para quien lee.
- Para comprobar el texto exacto, abre el enlace al archivo fuente de cada
  sección.

## Orden recomendado para leer estas tres páginas

Las secciones de esta página y de las otras dos ya están ordenadas siguiendo
esta progresión; no hace falta saltar de un lado a otro, basta con leer las
tres páginas de arriba hacia abajo, en este orden:

1. **En esta página**: `.gitignore`, `README.md` y `package.xml`, para conocer
   la estructura del paquete; después `CMakeLists.txt`, para relacionar cada
   ejecutable con su `.cpp`; y por último `interceptor.launch.py`, para ver
   qué arranca y en qué orden.
2. **En [nodos de odometría y tf2](Line-by-line-odometry-nodes.md)**: primero
   los subscriptores de diagnóstico (son las callbacks más simples del
   paquete), después los conversores tf2 y `tf2_listener`.
3. **En [modos de guiado](Line-by-line-guidance-modes.md)**: `pursuit_mode.cpp`
   antes que `PN_mode.cpp` (Pursuit es la base sobre la que PN añade cosas).

Después de terminar las tres páginas, usa el
[mapa completo del workspace](Workspace-file-map.md) para distinguir código
propio de dependencias vendorizadas y para ver los workflows de `.github`.

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

Esta tabla cubre **todas** las líneas del archivo, en orden. Las líneas en
blanco no aparecen porque no ejecutan nada; verifica en el archivo que el
número de línea coincide.

| Línea | Código | Explicación |
| ---: | --- | --- |
| 1 | `#!/usr/bin/env python` | *Shebang*: permite ejecutar el archivo directamente como script Python en sistemas Unix. |
| 3 | `"""` | Abre un docstring: un comentario de varias líneas que documenta el archivo. |
| 4 | `Launch file para el escenario interceptor/target.` | Texto descriptivo, no se ejecuta. |
| 6 | `- Agente Micro-XRCE-DDS (puente uXRCE-DDS <-> ROS 2)` | Lista lo que arranca este launch. |
| 7 | `- Los 4 nodos del paquete interceptor` | Nota: el comentario dice "4" pero el launch arranca 6 nodos activos; es el comentario el que quedó desactualizado, no el código — no asumas que el número de un comentario es exacto. |
| 8 | `(el dron 0 y el dron 1 se lanzan a mano, cada uno en su propia terminal,` | Aclara que PX4 no se arranca desde este archivo. |
| 9 | `dron 0 (interceptor) make px4_sitl gz_x500` | Comando de ejemplo para arrancar la instancia 0. |
| 10 | `dron1 (target) PX4_SIM_MODEL=gz_x500 /build/px4_sitl_default/bin/px4 -i 1` | Comando de ejemplo para arrancar la instancia 1. |
| 11 | `para tener control manual e interactivo de ambos)` | Cierra la explicación. |
| 12 | `"""` | Cierra el docstring. |
| 14 | `import os` | Importa utilidades del sistema operativo, aquí solo se usa para expandir `~`. |
| 16 | `from launch import LaunchDescription` | Importa la clase que representa "la lista de acciones a ejecutar". |
| 17 | `from launch.actions import ExecuteProcess` | Importa la acción que lanza un comando de shell arbitrario (se usará para el agente DDS). |
| 18 | `from launch_ros.actions import Node` | Importa la acción que lanza un nodo ROS 2 de un paquete. |
| 20 | `MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')` | Expande `~` a la ruta absoluta del usuario actual y la guarda en una constante. |
| 23 | `def generate_launch_description():` | Define la función sin argumentos que ROS 2 busca y ejecuta al hacer `ros2 launch`. |
| 25 | `micro_xrce_agent = ExecuteProcess(` | Empieza a construir la acción que arrancará el agente DDS. |
| 26 | `cmd=[['./build/MicroXRCEAgent udp4 -p 8888']],` | Comando a ejecutar: el binario del agente, transporte UDP/IPv4, puerto 8888. |
| 27 | `shell=True,` | Ejecuta ese comando a través de una shell, como si se escribiera en una terminal. |
| 28 | `cwd=MICRO_XRCE_DDS_AGENT_DIR,` | Carpeta de trabajo donde se lanza el comando (debe existir `build/MicroXRCEAgent` ahí dentro). |
| 29 | `output='log',` | La salida del proceso va a los logs de ROS 2, no directamente a la terminal. |
| 30 | `)` | Cierra la llamada a `ExecuteProcess`. |
| 32 | `target_vehicle_odometry_subscriber_node = Node(` | Empieza a construir el bloque del primer nodo. |
| 33 | `package='interceptor',` | Paquete ROS 2 donde buscar el ejecutable. |
| 34 | `executable='target_vehicle_odometry_subscriber',` | Nombre del binario a ejecutar (el subscriptor de diagnóstico con nombre "invertido", ver [Nodos y tópicos](Nodes-and-topics.md)). |
| 35 | `output='log',` | Su salida va a los logs, no a pantalla. |
| 36 | `)` | Cierra el bloque de este nodo. |
| 38 | `target_tf2_odometry_node = Node(` | Empieza el bloque del conversor tf2 del target. |
| 39 | `package='interceptor',` | Igual que la línea 33. |
| 40 | `executable='target_tf2_odometry',` | Ejecutable que convierte la odometría del target y publica `target/base_link` y `target/velocity`. |
| 41 | `output='screen',` | Aquí la salida sí va directamente a la terminal (`screen`), a diferencia de los diagnósticos. |
| 42 | `)` | Cierra el bloque. |
| 44 | `interceptor_vehicle_odometry_subscriber_node = Node(` | Empieza el bloque del segundo subscriptor de diagnóstico. |
| 45 | `package='interceptor',` | Igual que arriba. |
| 46 | `executable='interceptor_vehicle_odometry_subscriber',` | Este ejecutable escucha en realidad la odometría del target (instancia 1); ver el aviso de nombres invertidos. |
| 47 | `output='log',` | Salida a logs. |
| 48 | `)` | Cierra el bloque. |
| 50 | `interceptor_tf2_odometry_node = Node(` | Empieza el bloque del conversor tf2 del interceptor. |
| 51 | `package='interceptor',` | Igual que arriba. |
| 52 | `executable='interceptor_tf2_odometry',` | Publica `map -> interceptor/base_link`. |
| 53 | `output='screen',` | Salida a pantalla. |
| 54 | `)` | Cierra el bloque. |
| 56 | `# tf2_listener_node = Node(` | Línea comentada: empieza con `#`, así que Python la ignora por completo. |
| 57 | `#     package='interceptor',` | Comentada, ignorada. |
| 58 | `#     executable='tf2_listener',` | Comentada, ignorada. |
| 59 | `#     output='screen',` | Comentada, ignorada. |
| 60 | `# )` | Comentada, ignorada. Todo este bloque de 5 líneas está desactivado: por eso `tf2_listener` no arranca con el launch por defecto. |
| 62 | `pursuit_mode_node = Node(` | Empieza el bloque del modo de persecución pura. |
| 63 | `package='interceptor',` | Igual que arriba. |
| 64 | `executable='pursuit_mode',` | Ejecutable del modo `pursuit_mode`. |
| 65 | `output='screen',` | Salida a pantalla. |
| 66 | `)` | Cierra el bloque. |
| 68 | `PN_mode_node = Node(` | Empieza el bloque del modo de navegación proporcional. |
| 69 | `package='interceptor',` | Igual que arriba. |
| 70 | `executable='PN_mode',` | Ejecutable del modo `PN_mode`. |
| 71 | `output='screen',` | Salida a pantalla. |
| 72 | `)` | Cierra el bloque. |
| 74 | `return LaunchDescription([` | Construye y devuelve la lista de acciones que ROS 2 ejecutará; el orden de esta lista es el orden de arranque. |
| 75 | `micro_xrce_agent,` | Primero se arranca el agente DDS. |
| 76 | `target_vehicle_odometry_subscriber_node,` | Después este nodo. |
| 77 | `target_tf2_odometry_node,` | Después este. |
| 78 | `interceptor_vehicle_odometry_subscriber_node,` | Después este. |
| 79 | `interceptor_tf2_odometry_node,` | Después este. |
| 80 | `pursuit_mode_node,` | Después este. |
| 81 | `PN_mode_node,` | Y por último este. |
| 82 | `])` | Cierra la lista y la llamada a `LaunchDescription`. |

`tf2_listener_node` no aparece en esta lista final porque su variable ni
siquiera llegó a crearse (está comentada arriba): no basta con comentar el
bloque, si estuviera descomentado también habría que añadirlo aquí para que se
ejecute. PX4 SITL no aparece en ningún punto de este archivo: se arranca a
mano en otras terminales, como recuerda el docstring del principio.

Con esto terminan los archivos de construcción y arranque. Continúa con
[Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md).

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Lectura guiada del código](Code-walkthrough.md) · ➡️ Siguiente: [Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md)
