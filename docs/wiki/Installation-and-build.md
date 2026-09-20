# Instalación y compilación

Esta página contiene órdenes para escribir en una terminal. Una terminal es una
ventana donde se escriben instrucciones y el sistema operativo responde con
texto. No es necesario conocer Linux de antemano: copia una orden, pulsa Enter,
lee el resultado y continúa solo si no aparece un error.

Un **workspace** es la carpeta que reúne el código fuente, las dependencias y
los resultados de compilación de varios paquetes ROS 2 a la vez; en este
proyecto es la carpeta `ws_interceptor` que descargas más abajo con
`git clone`. Vas a ver esta palabra todo el rato en esta página.

## Requisitos

- **Ubuntu 24.04 (Noble)**: aporta compiladores, Python y las herramientas que
  esperan los paquetes ROS 2 de esta configuración.
- **ROS 2 Jazzy**: proporciona el runtime, los mensajes estándar y `rclcpp`.
  "Jazzy" es el nombre de esta versión (o *distribución*) de ROS 2.
- **`colcon`**: coordina la compilación de varios paquetes con dependencias.
- **`rosdep`**: traduce dependencias ROS 2 a paquetes instalables del sistema.
- **PX4**: produce la odometría y recibe los setpoints de vuelo.
- **Micro XRCE-DDS Agent**, instalado en `~/Micro-XRCE-DDS-Agent`: conecta el
  transporte uXRCE-DDS de PX4 con el grafo DDS que utilizan los nodos ROS 2.
- **QGroundControl v5.1.4**: la estación de tierra desde la que se arman los
  drones, se mueve el target y se elige el modo de guiado del interceptor.

No basta con tener los comandos instalados: también deben ser compatibles entre
sí — la sección de PX4 más abajo indica exactamente qué versión conviene
compilar para que coincida con el `px4_msgs` vendorizado en este repositorio.

### Qué es cada pieza, en más detalle

Un **runtime** es el conjunto de procesos y bibliotecas que deben estar
activos para que un programa ROS 2 funcione mientras se ejecuta, no solo para
compilarlo. **`rclcpp`** es la biblioteca de ROS 2 para escribir nodos en
C++: aporta `Node`, `spin`, publishers, subscribers y logging — se explica
con más detalle en [Lectura guiada del código](Code-walkthrough.md).

**[PX4](https://px4.io/)** es un piloto automático (*autopilot*) de código
abierto para drones y otros vehículos no tripulados: el software que, dentro
del vehículo, lee los sensores, estima su posición/velocidad/orientación (la
odometría) y controla los motores para seguir una referencia de vuelo (el
setpoint). Este proyecto no usa hardware real: cada instancia de PX4 corre
como **SITL** (*Software In The Loop*), el mismo software ejecutándose como
un programa normal en el ordenador, con sensores y física simulados en vez de
un dron físico.

**DDS** (*Data Distribution Service*) es el protocolo que usa ROS 2 para
repartir mensajes entre nodos sin que se conozcan directamente. PX4 no habla
DDS completo — pesa demasiado para una placa de vuelo pequeña — sino una
versión ligera llamada **[Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)**
("XRCE" es *eXtremely Resource Constrained Environments*, "entornos con
recursos extremadamente limitados"). El **Micro XRCE-DDS Agent** es el
programa puente que traduce esos mensajes ligeros de PX4 al DDS completo que
hablan los nodos ROS 2 (`rclcpp`), para que puedan suscribirse a ellos como a
cualquier otro tópico.

### Cómo instalar cada requisito si aún no lo tienes

Si te falta alguno, aquí tienes su guía oficial de instalación:

- **Ubuntu 24.04**: sigue el
  [tutorial oficial de instalación](https://ubuntu.com/desktop/docs/en/latest/tutorial/install-ubuntu-desktop/)
  (máquina física, máquina virtual o WSL2 en Windows). Fíjate bien en la
  versión que descargas: tiene que ser **24.04**, no la última disponible si
  para entonces ya hay una más nueva.
- **ROS 2 Jazzy**: sigue la
  [guía oficial de instalación](https://docs.ros.org/en/jazzy/Installation.html)
  (paquetes oficiales por `apt`). La instalación "Desktop"
  (`ros-jazzy-desktop`) ya incluye `rclcpp` y las herramientas básicas.
- **`colcon` y `rosdep`**: si instalaste ROS 2 con `apt`, súmalos con:

  ```bash
  sudo apt install python3-colcon-common-extensions python3-rosdep
  sudo rosdep init   # solo la primera vez en el sistema
  ```
- **QGroundControl v5.1.4**: descarga `QGroundControl-x86_64.AppImage` de la
  [release v5.1.4](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.4)
  y sigue la
  [guía oficial de instalación para Linux](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html),
  que explica los paquetes del sistema que necesita. Usa esa versión concreta
  y no la "última" ni la *Daily*: es la que se ha probado junto con la versión
  de PX4 de esta página. Un detalle de esa guía: pide instalar `libfuse2`, pero
  en Ubuntu 24.04 ese paquete se llama `libfuse2t64`
  (`sudo apt install libfuse2t64`). Sin él, el AppImage no llega a abrirse.

Una **estación de tierra** (*GCS*, *Ground Control Station*) es el programa
con el que una persona supervisa y manda órdenes al vehículo: armar, despegar,
cambiar de modo, indicar un destino en el mapa. PX4 comprueba que haya una
conectada antes de dejar armar; sin QGroundControl abierto, el armado se
rechaza con `No connection to the GCS`.

PX4 y el Micro XRCE-DDS Agent no se cubren con un enlace suelto, porque la
[guía oficial de PX4 + ROS 2, sección Jazzy](https://docs.px4.io/main/en/ros2/user_guide#jazzy)
da por hecho cosas que aquí no aplican (una versión de PX4 sin fijar, y el
Agent instalado como paquete `colcon` en vez de binario suelto). En vez de
remitir ahí y luego corregirlo, estos son directamente los comandos que hacen
falta para este proyecto; la guía de PX4 sigue siendo la fuente de las
versiones que se usan abajo.

#### PX4

PX4 y los nodos ROS 2 solo se entienden si usan **exactamente las mismas
definiciones de mensajes**. Muchos mensajes de PX4 llevan una versión
(`MESSAGE_VERSION`) que forma parte del nombre del tópico: por ejemplo, un
`VehicleStatus` de versión 4 se publica en `/fmu/out/vehicle_status_v4`. Si
PX4 publica una versión y `px4_ros2_cpp` espera otra, los modos no encuentran
los tópicos y mueren al arrancar con `Registration failed`.

El `px4_msgs` vendorizado en `src/px4_msgs` coincide, mensaje a mensaje, con
PX4 `main` en el commit **`14b3f44081`** (28 de julio de 2026). Su
`CHANGELOG.rst` dice "1.17.0", pero **no** es compatible con la release
`v1.17.0` (allí `VehicleStatus` va por la versión 1), ni con el `main` actual
de PX4. Por eso hay que compilar ese commit concreto. No hace falta crear un
workspace ROS 2 con `px4_msgs` (un paso que sí pide la guía de PX4 en
general): aquí ya está vendorizado.

PX4-Autopilot no tiene que estar dentro de `ws_interceptor` ni en ninguna
ruta fija — nada de este proyecto lo busca en un sitio concreto. Clónalo
donde prefieras guardar tus proyectos, por ejemplo tu carpeta personal:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout 14b3f44081
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

`git checkout` fija el commit (no existe una etiqueta con nombre para él, por
eso no se usa `-b` al clonar). `git submodule update --init --recursive`
descarga los submódulos de git que usa PX4 en las versiones de ese commit. El
script `Tools/setup/ubuntu.sh` instala las herramientas de compilación y de
simulación (incluida Gazebo). El último comando compila y arranca una vez de
prueba con el modelo `gz_x500`, el mismo que se usa en
[Simulation.md](Simulation.md).

#### Micro XRCE-DDS Agent

`interceptor.launch.py` espera un binario suelto en una ruta concreta
(`~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`), así que se compila como
proyecto CMake independiente, no como paquete `colcon` dentro de un
workspace ROS 2 (que es como lo instala la guía de PX4 en general):

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
```

`v2.4.3` es la versión que la guía de PX4 recomienda para Jazzy. Con esto
basta: no hace falta `sudo make install`, porque `interceptor.launch.py`
ejecuta directamente `build/MicroXRCEAgent` dentro de esta misma carpeta. Si
lo instalas en otra ruta, ajusta `MICRO_XRCE_DDS_AGENT_DIR` en ese archivo.

Ubuntu, ROS 2, `colcon`/`rosdep`, PX4, el Agent y QGroundControl son
instalaciones de sistema, independientes de este repositorio: se hacen una sola
vez por máquina, no cada vez que compilas `interceptor`.

## Descargar el workspace

Este sí puedes clonarlo donde quieras guardar tus proyectos (tu carpeta
personal, por ejemplo):

```bash
git clone https://github.com/Deireb/ws_interceptor.git
cd ws_interceptor
```

A partir de aquí, todos los comandos de esta página se ejecutan dentro de
esta carpeta (`ws_interceptor`), salvo que se diga lo contrario.

## Instalar dependencias

El repositorio ya incluye dentro de `src/` (`src` viene de *source*, "código
fuente") el paquete propio y las dependencias vendorizadas necesarias para
compilar:

- `src/interceptor`
- `src/px4_msgs`
- `src/px4_ros_com`
- `src/px4-ros2-interface-lib`

Por eso el siguiente comando apunta a `src` como argumento
(`--from-paths src`): le dice a `rosdep` dónde buscar qué instalar.

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` instala dependencias del sistema. La opción `--ignore-src` evita
intentar descargar otra vez los paquetes que ya están dentro de `src/`.

Si `rosdep` informa de una dependencia no resuelta, no conviene ocultar el
error con `-r`: hay que leer qué paquete falta, instalarlo y repetir el comando.
La opción `-y` solo responde automáticamente que sí a las instalaciones
confirmadas por rosdep.

Si el sistema pregunta por la contraseña, es la contraseña normal de tu usuario
de Ubuntu, no una contraseña del proyecto. Si aparece texto en rojo o una orden
termina con un código de error, detente y conserva el mensaje completo para
diagnosticarlo.

## Compilar

Para compilar todo el workspace:

```bash
colcon build --symlink-install
```

Para compilar el paquete propio y las dependencias que necesita:

```bash
colcon build --packages-up-to interceptor --symlink-install
```

Después de compilar, cada terminal que vaya a usar ROS 2 debe cargar el
workspace:

```bash
source install/setup.bash
```

`build/`, `install/` y `log/` son directorios generados localmente. No contienen
código fuente y se pueden regenerar si se limpia el workspace.

## Qué ocurre durante `colcon build`

1. `colcon` descubre el `package.xml` de cada paquete (el archivo que dice su
   nombre y de qué depende).
2. Construye un grafo de dependencias: un mapa de qué paquete necesita a
   cuál otro (por ejemplo, `interceptor` necesita `px4_msgs`), para saber en
   qué orden compilar.
3. Configura cada paquete con su herramienta de compilación (`ament_cmake`
   en este proyecto).
4. Compila las bibliotecas y los binarios — los ejecutables ya compilados
   que después lanzas con `ros2 run`, como `pursuit_mode`.
5. Instala en `install/` esos binarios, los headers (los archivos `.h`/`.hpp`
   que otros `.cpp` pueden traer con `#include` sin ver cómo están hechos por
   dentro), el contenido de `launch/` (para que `ros2 launch` lo encuentre) y
   los metadatos del paquete (datos sobre él, como el propio `package.xml`,
   no código).
6. Guarda resultados y logs para poder diagnosticar fallos.

`--symlink-install` hace que algunos archivos de desarrollo se expongan mediante
enlaces simbólicos. Esto reduce copias y hace más cómodo iterar, pero no evita
tener que recompilar cuando cambia código C++.

Un **enlace simbólico** es un archivo que apunta a otro archivo o carpeta. No es
una segunda copia; por eso ahorra espacio y refleja cambios de ciertos archivos
más rápidamente.

## Comprobar la instalación

```bash
ros2 pkg list | grep interceptor
ros2 pkg executables interceptor
```

El segundo comando debe mostrar los ejecutables del paquete, como
`pursuit_mode`, `PN_mode` y los nodos de odometría.

También puedes comprobar que ROS 2 encuentra los recursos instalados:

```bash
ros2 pkg prefix interceptor
ros2 interface show px4_msgs/msg/VehicleOdometry
```

El primer comando muestra la instalación que está usando la terminal. El segundo
permite aprender qué campos tiene el mensaje antes de escribir una callback.

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Inicio](Home.md) · ➡️ Siguiente: [Ejecución de la simulación](Simulation.md)
