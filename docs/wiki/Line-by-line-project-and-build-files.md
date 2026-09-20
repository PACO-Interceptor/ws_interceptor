# Análisis línea por línea: archivos de proyecto y construcción

Esta es la primera de tres páginas que cubren **todos los archivos propios de
`src/interceptor`**. Esta página se ocupa de los archivos de construcción,
configuración y arranque del paquete. Las otras dos son:

- [Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md):
  los suscriptores de diagnóstico, los dos conversores tf2 y `tf2_listener`.
- [Modos de guiado, línea por línea](Line-by-line-guidance-modes.md):
  `pursuit_mode.cpp` y `PN_mode.cpp`.

La mayoría de archivos de esta página se analizan línea por línea porque son
código o configuración que ejecuta la máquina (`.gitignore`, `package.xml`,
`CMakeLists.txt`, el launch). `LICENSE` y `README.md` son la excepción: son
texto para personas, no instrucciones que ROS 2 o CMake interpreten letra a
letra, así que sus secciones se limitan a explicar qué son y para qué sirven.

No se analizan línea por línea las librerías de terceros:
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
| `.cast<float>()` | Convierte los números de un vector/cuaternión de un tipo a otro (aquí, de `double` a `float`). La `f` final de `Eigen::Vector3f` significa que sus tres componentes son `float`; la `d` de `Eigen::Vector3d`, que son `double`. Las funciones de conversión de frames suelen trabajar en `double`, mientras que el estado interno de los modos se guarda en `float` — de ahí que este `.cast<float>()` aparezca tan a menudo. | `position_ned.cast<float>()`. |
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

1. **En esta página**: `.gitignore`, `LICENSE` y `README.md`, para conocer la
   estructura del paquete; después `package.xml` y `CMakeLists.txt`, para
   relacionar cada ejecutable con su `.cpp`; y por último
   `interceptor.launch.py`, para ver qué arranca y en qué orden.
2. **En [nodos de odometría y tf2](Line-by-line-odometry-nodes.md)**: primero
   el suscriptor de diagnóstico (son las callbacks más simples del
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

Es el texto legal del paquete, no código ejecutable: declara que la licencia
es propietaria (no open source), reserva los derechos a los autores
indicados y prohíbe copiar, distribuir, modificar o usar el software sin
permiso escrito previo. Envuelve ese texto en una etiqueta
`<license>...</license>`, una particularidad de este archivo y no un formato
estándar de licencias. `package.xml` declara la misma licencia propietaria,
de forma independiente, en su propia etiqueta `<license>` (sección 4, más
abajo).

## 3. `src/interceptor/README.md`

Archivo: [`src/interceptor/README.md`](../../src/interceptor/README.md)

Es la puerta de entrada al paquete para quien lo clona por primera vez.
Resume qué es (un paquete ROS 2 `ament_cmake` para seguir e interceptar un
target usando datos de odometría PX4), qué sistema operativo y herramientas
requiere (Ubuntu 24.04, ROS 2 Jazzy, PX4 toolchain), y qué tres paquetes
externos debe contener el workspace además de este (`px4_msgs`,
`px4_ros_com`, `px4-ros2-interface-lib`, con enlaces a sus repositorios
oficiales). A partir de ahí da los comandos para crear el workspace, clonar
esas dependencias, instalarlas, compilar y ejecutar el paquete; esos mismos
comandos ya se explican con detalle, orden y contexto en
[Instalación y compilación](Installation-and-build.md), así que aquí no se
repiten uno por uno. Termina enlazando la documentación oficial de ROS 2 y
PX4 para quien quiera profundizar sin confundirla con este código.

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
`px4_msgs`, `px4_ros_com` y `px4_ros2_cpp` declaran dependencias que el código
sí usa al compilar y ejecutar. Cada nombre debe corresponder a un paquete
ROS 2 que `find_package` pueda localizar. `sensor_msgs` es distinta: está
declarada aquí y en `CMakeLists.txt`, pero ningún archivo `.cpp` del paquete
la incluye ni usa ninguno de sus tipos — es una dependencia declarada sin uso
en el código fuente actual.

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
- `sensor_msgs`: dependencia declarada pero sin uso en el código actual (ver
  la nota en la sección de `package.xml`, más arriba).

### Construcción de cada ejecutable

Cada pareja `add_executable` + `ament_target_dependencies` hace dos cosas:

1. Asocia un nombre de comando a un `.cpp`.
2. Asocia las bibliotecas que ese binario necesita.

| Ejecutable | Fuente | Dependencias particulares |
| --- | --- | --- |
| `vehicle_odometry_subscriber` | `src/vehicle_odometry_subscriber.cpp` | ROS 2, tf2, geometría, mensajes PX4 y sensores. El launch lo arranca dos veces (una por vehículo) con `name=`/`parameters=` distintos. |
| `interceptor_tf2_odometry` | `src/interceptor_tf2_odometry.cpp` | Añade `px4_ros_com` para convertir frames. |
| `target_tf2_odometry` | `src/target_tf2_odometry.cpp` | Añade `px4_ros_com` para convertir frames y `px4_ros2_cpp` para calcular el desplazamiento de origen (`vectorToGlobalPosition`); además publica velocidad. |
| `tf2_listener` | `src/tf2_listener.cpp` | Buffer/listener tf2 y odometría PX4. |
| `pursuit_mode` | `src/pursuit_mode.cpp` | `px4_ros2_cpp` y transformaciones. |
| `PN_mode` | `src/PN_mode.cpp` | Idénticas a las de `pursuit_mode`. |

Las líneas con los nombres repetidos dentro de `ament_target_dependencies` no
son código duplicado accidental: indican al linker qué bibliotecas usa cada
target.

### Tests e instalación

Dentro de `if(BUILD_TESTING)`:

- `find_package(ament_lint_auto REQUIRED)` localiza el sistema de lint.
- Las dos asignaciones `ament_cmake_*_FOUND = TRUE` desactivan copyright y
  cpplint, siguiendo los comentarios del propio archivo.
- `ament_lint_auto_find_test_dependencies()` registra los tests automáticos.

`install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` instala los seis
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
| 6 | `Arranca el agente Micro XRCE-DDS, los 4 nodos de odometria/diagnostico` | Resume qué arranca este launch: el agente DDS y los 4 nodos de odometría/diagnóstico. |
| 7 | `(dos instancias de vehicle_odometry_subscriber, una por vehiculo, mas` | Nombra esos nodos: dos instancias del suscriptor de diagnóstico, una por vehículo. |
| 8 | `target_tf2_odometry e interceptor_tf2_odometry) y un solo modo de` | Cierra la lista con los dos conversores tf2 y añade que solo se lanza un modo de guiado. |
| 9 | `guiado, elegido con el argumento modo:=pn\|pursuit.` | Indica que el modo se elige con el argumento `modo`, que acepta `pn` o `pursuit`. |
| 11 | `Ejemplo de uso:` | Introduce el ejemplo de invocación. |
| 12 | `    ros2 launch interceptor interceptor.launch.py modo:=pn` | Comando de ejemplo, con `modo:=pn`. |
| 14 | `PX4 se lanza a mano, cada instancia en su propia terminal (desde ~/PX4-Autopilot):` | Aclara que PX4 no se arranca desde este archivo, y desde qué carpeta se ejecutan los comandos siguientes. |
| 15 | `    interceptor (instancia 0): make px4_sitl gz_x500` | Comando de ejemplo para arrancar la instancia 0 (interceptor). |
| 16 | `    target (instancia 1):` | Introduce el comando de la instancia 1 (target). |
| 17 | `        GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 \` | Comando de ejemplo para arrancar la instancia 1 (target), 20 m al norte del interceptor (`PX4_GZ_MODEL_POSE="0,20"`); la barra invertida continúa en la línea siguiente. |
| 18 | `            ./build/px4_sitl_default/bin/px4 -i 1` | Resto del comando de la instancia 1. |
| 20 | `Al pulsar Ctrl-C, el launch tarda unos 5 s en cerrarse: a proposito, para dar tiempo al` | Avisa de que el cierre con Ctrl-C tarda unos 5 s, y que es intencional. |
| 21 | `modo de guiado a darse de baja en PX4 antes de que el agente muera.` | Explica el motivo: dar tiempo al modo de guiado a darse de baja en PX4. |
| 22 | `"""` | Cierra el docstring. |
| 24 | `import os` | Importa utilidades del sistema operativo, aquí solo se usa para expandir `~`. |
| 26 | `from launch import LaunchDescription` | Importa la clase que representa "la lista de acciones a ejecutar". |
| 27 | `from launch.actions import DeclareLaunchArgument, ExecuteProcess` | Importa la acción que declara un argumento de launch (`modo`) y la que lanza un comando de shell arbitrario (el agente DDS). |
| 28 | `from launch.conditions import IfCondition` | Importa la condición que decide si una acción se ejecuta o no, según una expresión booleana. |
| 29 | `from launch.substitutions import EqualsSubstitution, LaunchConfiguration` | Importa la sustitución que compara dos valores en tiempo de lanzamiento (`EqualsSubstitution`) y la que lee el valor de un argumento declarado (`LaunchConfiguration`). |
| 30 | `from launch_ros.actions import Node` | Importa la acción que lanza un nodo ROS 2 de un paquete. |
| 32 | `MICRO_XRCE_DDS_AGENT_DIR = os.path.expanduser('~/Micro-XRCE-DDS-Agent')` | Expande `~` a la ruta absoluta del usuario actual y la guarda en una constante. |
| 35 | `def generate_launch_description():` | Define la función sin argumentos que ROS 2 busca y ejecuta al hacer `ros2 launch`. |
| 37 | `modo_arg = DeclareLaunchArgument(` | Empieza a declarar el argumento de launch `modo`. |
| 38 | `'modo',` | Nombre del argumento: se pasa como `modo:=<valor>` en la línea de comandos. |
| 39 | `choices=['pn', 'pursuit'],` | Restringe los valores válidos a `pn` y `pursuit`; sin valor por defecto, así que hay que indicarlo siempre. |
| 40 | `description=(` | Empieza la descripción del argumento (se ve con `ros2 launch interceptor interceptor.launch.py --show-args`). |
| 41 | `'Modo de guiado que se registra en PX4 (pn = PN mode, pursuit = Pursuit '` | Primera parte del texto de la descripción. |
| 42 | `'Intercept); solo se lanza uno.'` | Segunda parte: aclara que solo se lanza un modo. |
| 43 | `),` | Cierra la tupla de strings concatenados de `description`. |
| 44 | `)` | Cierra la llamada a `DeclareLaunchArgument`. |
| 46 | `# El agente ignora SIGINT para seguir vivo tras el Ctrl-C hasta que el launch escala a` | Comentario: explica por qué el agente ignora SIGINT. |
| 47 | ``# SIGTERM: asi el modo de guiado tiene tiempo de enviar `Unregistering` a PX4.`` | Continúa el comentario: da tiempo al modo de guiado a darse de baja en PX4. |
| 48 | `micro_xrce_agent = ExecuteProcess(` | Empieza a construir la acción que arrancará el agente DDS. |
| 49 | `cmd=[["trap '' INT; exec ./build/MicroXRCEAgent udp4 -p 8888"]],` | Comando a ejecutar: `trap '' INT` hace que el proceso ignore SIGINT (Ctrl-C no lo mata); `exec` sustituye la shell por el binario del agente, transporte UDP/IPv4, puerto 8888, para que la señal de cierre (SIGTERM) le llegue directamente a él. |
| 50 | `shell=True,` | Ejecuta ese comando a través de una shell, como si se escribiera en una terminal. |
| 51 | `cwd=MICRO_XRCE_DDS_AGENT_DIR,` | Carpeta de trabajo donde se lanza el comando (debe existir `build/MicroXRCEAgent` ahí dentro). |
| 52 | `output='log',` | La salida del proceso va a los logs de ROS 2, no directamente a la terminal. |
| 53 | `)` | Cierra la llamada a `ExecuteProcess`. |
| 55 | `target_vehicle_odometry_subscriber_node = Node(` | Empieza a construir el bloque del primer nodo. |
| 56 | `package='interceptor',` | Paquete ROS 2 donde buscar el ejecutable. |
| 57 | `executable='vehicle_odometry_subscriber',` | Nombre del binario a ejecutar: el suscriptor de diagnóstico parametrizado (ver [Nodos y tópicos](Nodes-and-topics.md)). |
| 58 | `name='target_vehicle_odometry_subscriber',` | Nombre visible del nodo en `ros2 node list`, distinto del ejecutable. |
| 59 | `parameters=[{` | Empieza la lista de parámetros de esta instancia. |
| 60 | `'vehicle_name': 'target',` | Fija `vehicle_name` a `target`. |
| 61 | `'odometry_topic': '/px4_1/fmu/out/vehicle_odometry',` | Fija `odometry_topic` al tópico de la instancia 1 (target): con esto el nombre del nodo y el tópico que escucha coinciden. |
| 62 | `}],` | Cierra la lista de parámetros. |
| 63 | `output='log',` | Su salida va a los logs, no a pantalla. |
| 64 | `)` | Cierra el bloque de este nodo. |
| 66 | `target_tf2_odometry_node = Node(` | Empieza el bloque del conversor tf2 del target. |
| 67 | `package='interceptor',` | Igual que la línea 56. |
| 68 | `executable='target_tf2_odometry',` | Ejecutable que convierte la odometría del target y publica `target/base_link` y `target/velocity`. |
| 69 | `output='screen',` | Aquí la salida sí va directamente a la terminal (`screen`), a diferencia de los diagnósticos. |
| 70 | `)` | Cierra el bloque. |
| 72 | `interceptor_vehicle_odometry_subscriber_node = Node(` | Empieza el bloque del segundo suscriptor de diagnóstico. |
| 73 | `package='interceptor',` | Igual que arriba. |
| 74 | `executable='vehicle_odometry_subscriber',` | Mismo ejecutable que la primera instancia, con otros parámetros. |
| 75 | `name='interceptor_vehicle_odometry_subscriber',` | Nombre visible de esta instancia. |
| 76 | `parameters=[{` | Empieza la lista de parámetros. |
| 77 | `'vehicle_name': 'interceptor',` | Fija `vehicle_name` a `interceptor`. |
| 78 | `'odometry_topic': '/fmu/out/vehicle_odometry',` | Fija `odometry_topic` al tópico de la instancia 0 (interceptor). |
| 79 | `}],` | Cierra la lista de parámetros. |
| 80 | `output='log',` | Salida a logs. |
| 81 | `)` | Cierra el bloque. |
| 83 | `interceptor_tf2_odometry_node = Node(` | Empieza el bloque del conversor tf2 del interceptor. |
| 84 | `package='interceptor',` | Igual que arriba. |
| 85 | `executable='interceptor_tf2_odometry',` | Publica `map -> interceptor/base_link`. |
| 86 | `output='screen',` | Salida a pantalla. |
| 87 | `)` | Cierra el bloque. |
| 89 | `# tf2_listener_node = Node(` | Línea comentada: empieza con `#`, así que Python la ignora por completo. |
| 90 | `#     package='interceptor',` | Comentada, ignorada. |
| 91 | `#     executable='tf2_listener',` | Comentada, ignorada. |
| 92 | `#     output='screen',` | Comentada, ignorada. |
| 93 | `# )` | Comentada, ignorada. Todo este bloque de 5 líneas está desactivado: por eso `tf2_listener` no arranca con el launch por defecto. |
| 95 | `pursuit_mode_node = Node(` | Empieza el bloque del modo de persecución pura. |
| 96 | `package='interceptor',` | Igual que arriba. |
| 97 | `executable='pursuit_mode',` | Ejecutable del modo `pursuit_mode`. |
| 98 | `output='screen',` | Salida a pantalla. |
| 99 | `condition=IfCondition(EqualsSubstitution(LaunchConfiguration('modo'), 'pursuit')),` | Solo se lanza este nodo si `modo` vale `pursuit`: compara el valor del argumento con la cadena `'pursuit'`. |
| 100 | `)` | Cierra el bloque. |
| 102 | `PN_mode_node = Node(` | Empieza el bloque del modo de navegación proporcional. |
| 103 | `package='interceptor',` | Igual que arriba. |
| 104 | `executable='PN_mode',` | Ejecutable del modo `PN_mode`. |
| 105 | `output='screen',` | Salida a pantalla. |
| 106 | `condition=IfCondition(EqualsSubstitution(LaunchConfiguration('modo'), 'pn')),` | Solo se lanza este nodo si `modo` vale `pn`. |
| 107 | `)` | Cierra el bloque. |
| 109 | `return LaunchDescription([` | Construye y devuelve la lista de acciones que ROS 2 ejecutará; el orden de esta lista es el orden de arranque. |
| 110 | `modo_arg,` | Primero se declara el argumento `modo` (debe ir antes de usarse en las condiciones de los nodos). |
| 111 | `micro_xrce_agent,` | Después se arranca el agente DDS. |
| 112 | `target_vehicle_odometry_subscriber_node,` | Después este nodo. |
| 113 | `target_tf2_odometry_node,` | Después este. |
| 114 | `interceptor_vehicle_odometry_subscriber_node,` | Después este. |
| 115 | `interceptor_tf2_odometry_node,` | Después este. |
| 116 | `pursuit_mode_node,` | Este solo arranca de verdad si `modo:=pursuit`. |
| 117 | `PN_mode_node,` | Y este solo si `modo:=pn`. |
| 118 | `])` | Cierra la lista y la llamada a `LaunchDescription`. |

`tf2_listener_node` no aparece en esta lista final porque su variable ni
siquiera llegó a crearse (está comentada arriba): no basta con comentar el
bloque, si estuviera descomentado también habría que añadirlo aquí para que se
ejecute. `pursuit_mode_node` y `PN_mode_node` sí están siempre en la lista,
pero su `condition` decide en tiempo de lanzamiento si el proceso llega a
arrancar; solo uno de los dos lo hace, según `modo`. PX4 SITL no aparece en
ningún punto de este archivo: se arranca a mano en otras terminales, como
recuerda el docstring del principio.

Con esto terminan los archivos de construcción y arranque. Continúa con
[Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md).

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Lectura guiada del código](Code-walkthrough.md) · ➡️ Siguiente: [Nodos de odometría y tf2, línea por línea](Line-by-line-odometry-nodes.md)
