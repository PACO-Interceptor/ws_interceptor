# ws_interceptor

Workspace ROS 2 para el seguimiento e interceptación de un vehículo usando datos de odometría PX4.

## Requisitos

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- `colcon`
- `rosdep`
- Para ejecutar la simulación: PX4 en el commit `14b3f44081`, Micro XRCE-DDS
  Agent `v2.4.3` y QGroundControl `v5.1.4` (versiones probadas juntas; ver
  [Instalación y compilación](docs/wiki/Installation-and-build.md))

## Obtener el workspace

```bash
git clone https://github.com/PACO-Interceptor/ws_interceptor.git
cd ws_interceptor
```

El repositorio ya incluye el paquete propio `interceptor` y las dependencias PX4 necesarias dentro de `src/`.

## Instalar dependencias y compilar

Desde la carpeta del workspace (la del `cd` anterior; si abres una terminal
nueva, vuelve a entrar en ella con `cd ~/ws_interceptor`):

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

Cada terminal nueva que vaya a usar `ros2` necesita cargar el entorno otra vez,
desde la carpeta del workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Ejecutar la simulación

Hacen falta cuatro terminales, una por pieza. Los detalles y qué debería verse
en cada paso están en
[Ejecución de la simulación](docs/wiki/Simulation.md).

```bash
# 1. Interceptor (instancia 0). Abre también la ventana de Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# 2. Target (instancia 1), 20 m al norte
cd ~/PX4-Autopilot
GZ_IP=127.0.0.1 PX4_GZ_MODEL_POSE="0,20" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1

# 3. Nodos del proyecto, con el modo de guiado elegido
cd ~/ws_interceptor          # la ruta donde clonaste el workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interceptor interceptor.launch.py modo:=pn

# 4. Estación de tierra
~/QGroundControl-x86_64.AppImage
```

En QGroundControl: despega el target (vehículo 2), mándalo a otro punto con
*Go to location*, despega el interceptor (vehículo 1) y elígele **PN mode** o
**Pursuit Intercept** en el selector de modo.

El argumento `modo` es opcional y solo acepta `pn` o `pursuit`: elige qué modo
de guiado se registra en PX4, ya que solo se lanza uno. Si se omite, vale `pn`.
Sin QGroundControl conectado, PX4 no deja armar. Los nodos individuales también
pueden iniciarse con `ros2 run interceptor <ejecutable>`.

Para parar: Ctrl-C primero en el launch, que tarda unos 5 segundos a propósito,
y después en las instancias PX4.

## Estructura

- `src/interceptor`: nodos y lanzador de este proyecto.
- `src/px4_msgs`: mensajes PX4.
- `src/px4_ros_com`: comunicación PX4-ROS 2.
- `src/px4-ros2-interface-lib`: biblioteca C++ de la interfaz PX4-ROS 2.

Los directorios `build/`, `install/` y `log/` se generan localmente y no forman parte del repositorio.

## Wiki

La [wiki introductoria local](docs/wiki/Home.md) explica paso a paso cómo
preparar el workspace, ejecutar la simulación y entender el papel de cada nodo.
Es la única fuente: la [wiki de GitHub](https://github.com/PACO-Interceptor/ws_interceptor/wiki)
se genera a partir de ella y no se edita a mano.

```bash
python3 tools/publicar-wiki.py --dry-run   # ver qué cambiaría
python3 tools/publicar-wiki.py             # publicar
```

El script convierte los enlaces al formato de la wiki (páginas sin extensión y
con su nombre en español, rutas al código como URLs absolutas) y sube el
resultado. Al añadir una página nueva en `docs/wiki/`, hay que darle nombre en
el diccionario `PAGINAS` de ese script.

## License

The `interceptor` package and the rest of the code in this repository are
released under the [Apache-2.0](LICENSE) licence.

The PX4 dependencies included in `src/` (`px4_msgs`, `px4_ros_com` and
`px4-ros2-interface-lib`) keep their original BSD-3-Clause licence, found in the
`LICENSE` file of each one.
