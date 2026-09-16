# ws_interceptor

`ws_interceptor` es un workspace de ROS 2 que conecta dos vehículos PX4
simulados: un **interceptor**, que intenta alcanzar a un **target**. El
código propio está en [`src/interceptor`](../../src/interceptor). El
workspace también contiene tres dependencias vendorizadas: `px4_msgs`,
`px4_ros_com` y `px4-ros2-interface-lib`, que proporcionan los mensajes PX4,
las conversiones de coordenadas y la biblioteca para registrar modos de
vuelo personalizados.

## Resumen en una página

PX4 publica odometría, los nodos la convierten y la relacionan mediante tf2,
y un modo de guiado calcula un setpoint que PX4 intenta seguir. Si es tu
primera vez aquí, esto es todo lo esencial antes de entrar en detalle:

- Hay dos drones simulados con PX4: el **interceptor** (el que persigue) y el
  **target** (el perseguido).
- Cada PX4 publica su posición y velocidad (`VehicleOdometry`) en un tópico
  ROS 2 distinto: instancia `0` para el interceptor, instancia `1` para el
  target.
- Dos nodos (`interceptor_tf2_odometry`, `target_tf2_odometry`) convierten
  esos datos de las coordenadas de PX4 (NED) a las de ROS (ENU) y publican el
  resultado como transforms tf2: "dónde está cada dron respecto a un origen
  común llamado `map`".
- El nodo del target también publica su velocidad en un tópico aparte
  (`target/velocity`), porque tf2 solo guarda posición y orientación, no
  velocidad.
- Dos modos de vuelo (`pursuit_mode` y `PN_mode`) leen esa posición —y, en el
  caso de PN, también la velocidad— y calculan hacia dónde debe moverse el
  interceptor. Se lo entregan a PX4 como un *setpoint* (una referencia de
  velocidad/aceleración); PX4 se encarga de moverlo de verdad.
- `pursuit_mode` es el más simple: apunta directo hacia la posición actual
  del target. `PN_mode` es más sofisticado: anticipa el movimiento del
  target usando su velocidad (navegación proporcional).
- El paquete incluye además nodos auxiliares de inspección y diagnóstico, que
  solo imprimen datos por consola sin intervenir en el control de vuelo.
- Nada de esto arranca PX4 por ti: hay que lanzar las dos simulaciones PX4 a
  mano. El `launch` de este proyecto sí arranca el Micro XRCE-DDS Agent y los
  nodos propios, pero necesita que el agente ya esté compilado en
  `~/Micro-XRCE-DDS-Agent`.

Con esto ya tienes la idea general. El resto de la wiki explica cada pieza
con más detalle, con diagramas y con los comandos exactos para instalar,
ejecutar y depurar.

## Recorrido recomendado

La documentación está ordenada para seguir el sistema de extremo a extremo:

1. [Instalación y compilación](Installation-and-build.md)
2. [Ejecución de la simulación](Simulation.md)
3. [Arquitectura y flujo de datos](Architecture.md)
4. [Nodos de odometría y diagnóstico](Nodes-and-topics.md)
5. [Modos de guiado](Guidance-modes.md)
6. [Lectura guiada del código](Code-walkthrough.md)
7. [Análisis línea por línea: proyecto y construcción](Line-by-line-project-and-build-files.md)
8. [Análisis línea por línea: nodos de odometría y tf2](Line-by-line-odometry-nodes.md)
9. [Análisis línea por línea: modos de guiado](Line-by-line-guidance-modes.md)
10. [Mapa del workspace y dependencias](Workspace-file-map.md)
11. [Referencia y solución de problemas](Quick-reference-and-troubleshooting.md)

## Glosario mínimo

Dieciséis palabras que se repiten en toda la wiki. No sustituyen a las
explicaciones de cada página, pero sirven para no perderse si llegas
directamente a una página intermedia sin leer el resumen de arriba:

| Concepto | Significado en este proyecto |
| --- | --- |
| Nodo | Programa ROS 2 que realiza una tarea concreta. |
| Modo (de vuelo) | Estrategia de guiado registrada en PX4 (`pursuit_mode`, `PN_mode`). No es lo mismo que un nodo, aunque cada modo se implementa dentro de uno — fíjate bien: "nodo" y "modo" solo se diferencian en una letra. |
| Tópico | Canal con nombre por el que se intercambian mensajes. |
| Mensaje | Estructura de datos que viaja por un tópico. |
| Publisher | Lo que crea un nodo para **enviar** mensajes a un tópico. |
| Suscripción (subscriber) | Lo que crea un nodo para **recibir** mensajes de un tópico; cada mensaje que llega dispara una callback. |
| Timer | Dispara una callback repetidamente a intervalos fijos (por ejemplo, cada 50 ms), sin depender de que llegue ningún mensaje. |
| Logging | El mecanismo para imprimir mensajes de diagnóstico (`RCLCPP_INFO`, `RCLCPP_WARN`...) en la consola o en los logs. |
| Callback | Función que no llamas tú: se la entregas a ROS 2 (al suscribirte a un tópico, al crear un timer...) y es ROS 2 quien la ejecuta automáticamente cada vez que llega un mensaje o dispara el temporizador. |
| DDS | Protocolo que reparte los mensajes entre nodos sin que se conozcan directamente entre sí. PX4 usa una versión ligera (uXRCE-DDS) que el Micro XRCE-DDS Agent traduce al DDS que hablan los nodos ROS 2. |
| Frame (marco) | Sistema de referencia con nombre (`map`, `interceptor/base_link`, `target/base_link`) que tf2 usa para ubicar cada vehículo. |
| tf2 | Sistema que relaciona posiciones y orientaciones entre marcos. |
| Odometría | Posición, orientación y velocidad estimadas. |
| Setpoint | Referencia de movimiento que se entrega a PX4. |
| PX4 | Piloto automático de código abierto: el software que, dentro del vehículo, estima su posición y controla los motores para seguir un setpoint. Más detalle en [Instalación y compilación](Installation-and-build.md). |
| SITL | PX4 ejecutado como software en el ordenador, sin hardware real, simulando sensores y física. |

Esto es solo un glosario de bolsillo; los términos se explican con más
detalle donde hace falta en cada página. Para comandos y pasos de
diagnóstico, usa la [referencia rápida y solución de problemas](Quick-reference-and-troubleshooting.md)
— es una página distinta, centrada en órdenes de terminal, no en conceptos.

## Límites y seguridad

Esta wiki documenta el comportamiento actual del repositorio; no sustituye la
documentación oficial de ROS 2, PX4 ni la teoría de control. El proyecto debe
probarse en simulación antes de cualquier uso con hardware real, respetando las
medidas de seguridad y los límites configurados en PX4.

---

➡️ Siguiente: [Instalación y compilación](Installation-and-build.md)
