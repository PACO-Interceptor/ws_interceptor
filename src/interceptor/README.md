# interceptor

Paquete ROS2 (ament_cmake) para el seguimiento e interceptación de un vehículo objetivo usando datos de odometría PX4.

## Requisitos

- Ubuntu 24.04 (Noble)
- ROS2 Jazzy
- PX4 toolchain

## Dependencias

Este paquete debe compilarse dentro de un workspace ROS2 que además contenga estos paquetes en `src/`:

- [px4_msgs](https://github.com/PX4/px4_msgs.git)
- [px4_ros_com](https://github.com/PX4/px4_ros_com.git)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) (provee `px4_ros2_cpp`)

Dependencias ROS2 estándar (se instalan con `rosdep`): `rclcpp`, `tf2_ros`, `tf2`, `geometry_msgs`, `sensor_msgs`, `launch`, `launch_ros`.

## Preparar el workspace

```bash
mkdir -p ~/ws_interceptor/src
cd ~/ws_interceptor/src

git clone git@github.com:Deireb/ws_interceptor.git interceptor
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/Auterion/px4-ros2-interface-lib.git
```

## Instalar dependencias

```bash
cd ~/ws_interceptor
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Compilar

```bash
cd ~/ws_interceptor
colcon build --symlink-install
source install/setup.bash
```

Para compilar solo este paquete (y sus dependencias):

```bash
colcon build --packages-up-to interceptor --symlink-install
```

## Ejecutar

```bash
ros2 launch interceptor interceptor.launch.py modo:=pn
```

El argumento `modo` es opcional (`pn` por defecto, o `pursuit`): solo se lanza el modo de guiado elegido.

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
