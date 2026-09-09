# ws_interceptor

Workspace ROS 2 para el seguimiento e interceptación de un vehículo usando datos de odometría PX4.

## Requisitos

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- `colcon`
- `rosdep`
- PX4 y Micro XRCE-DDS Agent si se va a ejecutar la simulación

## Obtener el workspace

```bash
git clone https://github.com/Deireb/ws_interceptor.git
cd ws_interceptor
```

El repositorio ya incluye el paquete propio `interceptor` y las dependencias PX4 necesarias dentro de `src/`.

## Instalar dependencias y compilar

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Para compilar el paquete propio junto con sus dependencias:

```bash
colcon build --packages-up-to interceptor --symlink-install
```

## Ejecutar

```bash
ros2 launch interceptor interceptor.launch.py
```

El lanzador espera encontrar Micro XRCE-DDS Agent en `~/Micro-XRCE-DDS-Agent` y que los vehículos PX4 estén ejecutándose. Los nodos individuales también pueden iniciarse con `ros2 run interceptor <ejecutable>`.

## Estructura

- `src/interceptor`: nodos y lanzador de este proyecto.
- `src/px4_msgs`: mensajes PX4.
- `src/px4_ros_com`: comunicación PX4-ROS 2.
- `src/px4-ros2-interface-lib`: biblioteca C++ de la interfaz PX4-ROS 2.

Los directorios `build/`, `install/` y `log/` se generan localmente y no forman parte del repositorio.

## Documentación para nuevos colaboradores

La [wiki introductoria local](docs/wiki/Home.md) explica paso a paso cómo
preparar el workspace, ejecutar la simulación y entender el papel de cada nodo.
Está pensada para personas que se incorporan al proyecto y todavía no dominan
ROS 2, tf2 o PX4.