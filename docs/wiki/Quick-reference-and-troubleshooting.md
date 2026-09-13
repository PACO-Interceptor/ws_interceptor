# Referencia rápida y solución de problemas

Esta página es una lista de comprobaciones para cuando algo no funciona. Un
**error** no siempre significa que el código esté mal: puede faltar un proceso,
una dependencia, una configuración o un dato. La estrategia es comprobar una
capa cada vez y no cambiar muchas cosas simultáneamente.

## Comandos habituales

```bash
# Ver nodos activos
ros2 node list

# Ver tópicos y tipos
ros2 topic list
ros2 topic type /target/velocity

# Inspeccionar mensajes
ros2 topic echo /target/velocity
ros2 topic echo /px4_1/fmu/out/vehicle_odometry

# Ejecutar un nodo del paquete
ros2 run interceptor target_tf2_odometry
```

`echo` muestra los mensajes que pasan por un tópico, `hz` calcula cuántos
mensajes llegan por segundo e `info` muestra quién publica o recibe. `run`
arranca un ejecutable. Estos comandos normalmente observan el sistema; no
cambian la trayectoria del dron por sí solos.

## Secuencia de diagnóstico recomendada

Ejecuta las comprobaciones en este orden, porque cada paso depende del anterior:

```bash
# 1. ¿Está ROS 2 y qué nodos existen?
ros2 node list

# 2. ¿Llegan datos de PX4?
ros2 topic list
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic hz /px4_1/fmu/out/vehicle_odometry

# 3. ¿El mensaje tiene la estructura esperada?
ros2 topic type /px4_1/fmu/out/vehicle_odometry
ros2 interface show px4_msgs/msg/VehicleOdometry

# 4. ¿Existe el árbol tf2?
ros2 run tf2_ros tf2_echo map target/base_link
ros2 run tf2_ros tf2_echo map interceptor/base_link

# 5. ¿Llega la velocidad transformada?
ros2 topic hz /target/velocity
```

Si un comando falla, corrige ese nivel antes de continuar. No tiene sentido
depurar PN mientras el tópico de odometría del target está vacío.

## Problemas frecuentes

### No aparece `VehicleOdometry`

Comprueba que las dos instancias PX4 estén ejecutándose, que el agente esté
activo y que cada terminal tenga cargado ROS 2. La instancia 1 debe publicar
con el prefijo `/px4_1/`.

Distingue entre “el tópico no existe” y “existe pero no publica”. `ros2 topic
list` comprueba lo primero y `ros2 topic hz` lo segundo. Si existe pero no hay
frecuencia, revisa PX4, el agente y el transporte UDP.

### No aparece el target en tf2

Comprueba que `target_tf2_odometry` esté activo y que reciba
`/px4_1/fmu/out/vehicle_odometry`. Si el tópico existe pero el frame no aparece,
revisa la salida del nodo y el parámetro `vehicle_name`.

Usa `tf2_echo` para saber qué frame falta. Si aparece `map` pero no
`target/base_link`, el problema está en `target_tf2_odometry` o en su entrada,
no en PN.

### PN no permite armar

PN necesita dos datos: `map -> target/base_link` y `target/velocity`. Inicia
`target_tf2_odometry` y confirma el tópico:

```bash
ros2 topic echo /target/velocity
```

También confirma que el modo recibe la posición:

```bash
ros2 run tf2_ros tf2_echo map target/base_link
```

PN necesita que ambas fuentes existan; que solo funcione una no es suficiente.

### El launch no encuentra Micro XRCE-DDS Agent

El archivo de launch espera encontrar el agente en:

```text
~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
```

Si la ruta de instalación es diferente, hay que ajustar
`MICRO_XRCE_DDS_AGENT_DIR` en `interceptor.launch.py`.

### Los cambios de C++ no se reflejan

Compila de nuevo y vuelve a cargar el workspace:

```bash
colcon build --packages-up-to interceptor --symlink-install
source install/setup.bash
```

Si sigue apareciendo una versión antigua, comprueba:

```bash
ros2 pkg prefix interceptor
which ros2
```

Puede que la terminal esté usando otro workspace superpuesto. Cada `source`
modifica el entorno de esa terminal, no de las demás.

### Aparecen warnings de tf2

Los modos consultan transformaciones periódicamente. Es normal recibir avisos
al principio, antes de que el conversor haya recibido la primera odometría.
Warnings continuos indican que falta un nodo, un tópico o un nombre de frame.

No ocultes el warning aumentando el intervalo sin entenderlo. Los nombres
`map`, `target/base_link` e `interceptor/base_link` deben coincidir exactamente,
incluyendo mayúsculas, barras y namespace.

## Orden seguro para modificar código

1. Identifica si el cambio afecta a un nodo de diagnóstico, a tf2 o al guiado.
2. Revisa los tópicos y marcos que ya utiliza el nodo.
3. Mantén las conversiones NED/ENU en un solo lugar y documenta cualquier
   cambio de convención.
4. Compila con `colcon build --packages-up-to interceptor --symlink-install`.
5. Ejecuta los tests/lint del paquete:

```bash
colcon test --packages-select interceptor --event-handlers console_direct+
colcon test-result --verbose
```

Antes de abrir una pull request, revisa también el diff:

```bash
git status
git diff --check
git diff -- README.md docs/
```

Una buena contribución explica qué cambió, por qué, cómo se probó y qué
limitaciones siguen existiendo.

## Referencias oficiales

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [PX4 (sitio oficial)](https://px4.io/)
- [Guía ROS 2 de PX4](https://docs.px4.io/main/en/ros2/)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Micro XRCE-DDS (documentación del protocolo)](https://micro-xrce-dds.docs.eprosima.com/en/latest/)
- [Micro-XRCE-DDS-Agent (repositorio)](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Mapa del workspace y dependencias](Workspace-file-map.md) · 🏠 Volver a [Inicio](Home.md)
