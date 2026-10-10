# interceptor

Paquete ROS2 (ament_cmake) para el seguimiento e interceptación de un vehículo objetivo usando datos de odometría PX4.

## Requisitos

- Ubuntu 24.04 (Noble)
- ROS2 Jazzy
- Para ejecutar la simulación: PX4 en el commit `14b3f44081`, Micro XRCE-DDS Agent `v2.4.3` y
  QGroundControl `v5.1.4` (versiones probadas juntas; ver la
  [guía de instalación](../../docs/wiki/Installation-and-build.md))

## Dependencias

Este paquete se compila dentro del workspace `ws_interceptor`, que ya incluye en `src/` los tres
paquetes PX4 que necesita, vendorizados (no hay que clonarlos aparte):

- [px4_msgs](https://github.com/PX4/px4_msgs.git)
- [px4_ros_com](https://github.com/PX4/px4_ros_com.git)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) (provee `px4_ros2_cpp`)

Dependencias ROS2 estándar (se instalan con `rosdep`): `rclcpp`, `tf2_ros`, `tf2`, `geometry_msgs`, `sensor_msgs`, `launch`, `launch_ros`.

## Preparar el workspace

```bash
git clone https://github.com/Deireb/ws_interceptor.git
cd ws_interceptor
```

## Instalar dependencias

Desde la carpeta del workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Compilar

```bash
colcon build --symlink-install
source install/setup.bash
```

Para compilar solo este paquete (y sus dependencias):

```bash
colcon build --packages-up-to interceptor --symlink-install
```

## Ejecutar

El launch no arranca PX4: las dos instancias SITL se lanzan a mano antes, cada una en su terminal
(ver [Ejecución de la simulación](../../docs/wiki/Simulation.md)). Con el entorno cargado
(`source /opt/ros/jazzy/setup.bash` y `source install/setup.bash` desde la carpeta del workspace):

```bash
ros2 launch interceptor interceptor.launch.py mode:=pn
```

El argumento `mode` es opcional (`pn` por defecto, o `pursuit`): solo se lanza el modo de guiado elegido. `modo:=` se acepta temporalmente como alias.

O ejecutar un nodo individual, por ejemplo:

```bash
ros2 run interceptor pursuit_mode
```

## Dudas

Si tenéis cualquier duda, consultadme o revisad la documentación de PX4/ROS2 antes de tocar el código:

- [PX4 ROS2 Interface](https://docs.px4.io/main/en/ros2/)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Documentación ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
