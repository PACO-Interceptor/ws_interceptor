# ws_interceptor

`ws_interceptor` es un workspace de ROS 2 que conecta dos vehículos PX4:

- **Interceptor**: el vehículo que intenta alcanzar al target.
- **Target**: el vehículo cuya posición y movimiento se siguen.

El código propio está en [`src/interceptor`](../../src/interceptor). El workspace
también contiene tres dependencias vendorizadas: `px4_msgs`,
`px4_ros_com` y `px4-ros2-interface-lib`. Estas dependencias proporcionan los
mensajes PX4, las conversiones de coordenadas y la biblioteca para registrar
modos de vuelo personalizados.

## Funcionamiento en una frase

PX4 publica odometría, los nodos la convierten y la relacionan mediante tf2, y
un modo de guiado calcula un setpoint que PX4 intenta seguir.

## Recorrido recomendado

La documentación está ordenada para seguir el sistema de extremo a extremo:

1. [Instalación y compilación](Installation-and-build.md)
2. [Ejecución de la simulación](Simulation.md)
3. [Arquitectura y flujo de datos](Architecture.md)
4. [Nodos de odometría y diagnóstico](Nodes-and-topics.md)
5. [Modos de guiado](Guidance-modes.md)
6. [Lectura guiada del código](Code-walkthrough.md)
7. [Análisis detallado de cada archivo propio](Line-by-line-code-analysis.md)
8. [Mapa del workspace y dependencias](Workspace-file-map.md)
9. [Referencia y solución de problemas](Quick-reference-and-troubleshooting.md)

## Alcance del proyecto

El paquete propio:

- recibe `VehicleOdometry` de dos instancias PX4;
- convierte datos entre los marcos NED y ENU;
- publica transforms tf2 para ambos vehículos;
- publica la velocidad del target;
- ofrece los modos `pursuit_mode` y `PN_mode`;
- proporciona nodos auxiliares para inspección y diagnóstico.

El launch del proyecto no inicia PX4 SITL. Las instancias PX4 y el
Micro XRCE-DDS Agent deben estar preparados antes de ejecutar el launch.

## Referencias rápidas

| Concepto | Significado en este proyecto |
| --- | --- |
| Nodo | Programa ROS 2 que realiza una tarea concreta. |
| Tópico | Canal con nombre por el que se intercambian mensajes. |
| Mensaje | Estructura de datos que viaja por un tópico. |
| tf2 | Sistema que relaciona posiciones y orientaciones entre marcos. |
| Odometría | Posición, orientación y velocidad estimadas. |
| Setpoint | Referencia de movimiento que se entrega a PX4. |
| SITL | PX4 ejecutado como software para simular un vehículo. |

Los términos técnicos que aparecen en las páginas se explican donde son
necesarios. La [referencia y solución de problemas](Quick-reference-and-troubleshooting.md)
reúne los comandos habituales.

## Límites y seguridad

Esta wiki documenta el comportamiento actual del repositorio; no sustituye la
documentación oficial de ROS 2, PX4 ni la teoría de control. El proyecto debe
probarse en simulación antes de cualquier uso con hardware real, respetando las
medidas de seguridad y los límites configurados en PX4.

---

➡️ Siguiente: [Instalación y compilación](Installation-and-build.md)
