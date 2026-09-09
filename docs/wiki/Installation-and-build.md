# Instalación y compilación

Esta página contiene órdenes para escribir en una terminal. Una terminal es una
ventana donde se escriben instrucciones y el sistema operativo responde con
texto. No es necesario conocer Linux de antemano: copia una orden, pulsa Enter,
lee el resultado y continúa solo si no aparece un error.

## Requisitos

- Ubuntu 24.04 (Noble).
- ROS 2 Jazzy.
- `colcon` y `rosdep`.
- PX4 si se va a ejecutar SITL.
- Micro XRCE-DDS Agent instalado en `~/Micro-XRCE-DDS-Agent`.

Una **distribución** de ROS 2 es una versión identificada por un nombre; aquí
usamos Jazzy. Un **workspace** es la carpeta que reúne el código fuente, las
dependencias y los resultados de compilación. `src` viene de *source* y
significa “código fuente”.

## Qué significa cada requisito

- **Ubuntu** aporta compiladores, Python y las herramientas que esperan los
  paquetes ROS 2 de esta configuración.
- **ROS 2 Jazzy** proporciona el runtime, los mensajes estándar y `rclcpp`.
- **colcon** coordina la compilación de varios paquetes con dependencias.
- **rosdep** traduce dependencias ROS 2 a paquetes instalables del sistema.
- **PX4** produce la odometría y recibe los setpoints de vuelo.
- **Micro XRCE-DDS Agent** conecta el transporte uXRCE-DDS de PX4 con el
  grafo DDS que utilizan los nodos ROS 2.

No basta con tener los comandos instalados: también deben ser compatibles entre
sí. Una versión diferente de `px4_msgs` puede tener campos distintos y provocar
incompatibilidades al registrar un modo.

El repositorio incluye dentro de `src/` el paquete propio y las dependencias
vendorizadas necesarias para compilar:

- `src/interceptor`
- `src/px4_msgs`
- `src/px4_ros_com`
- `src/px4-ros2-interface-lib`

## Descargar el workspace

```bash
git clone https://github.com/Deireb/ws_interceptor.git
cd ws_interceptor
```

## Instalar dependencias

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

1. `colcon` descubre los `package.xml`.
2. Construye un grafo de dependencias.
3. Configura cada paquete con su herramienta (`ament_cmake` en este proyecto).
4. Compila bibliotecas y ejecutables.
5. Instala binarios, headers, launch y metadatos en `install/`.
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
