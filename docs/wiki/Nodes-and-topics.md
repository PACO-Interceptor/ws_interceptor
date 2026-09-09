# Nodos, tópicos y diagnóstico

Un **diagnóstico** es una comprobación para descubrir por qué algo no funciona.
En ROS 2, los comandos de esta página observan el sistema sin modificar el
código. Un **topic** es lo mismo que un tópico: un canal identificado por un
nombre.

## Nodos principales

| Ejecutable | Responsabilidad |
| --- | --- |
| `interceptor_tf2_odometry` | Convierte la odometría de la instancia 0 y publica el frame del interceptor. |
| `target_tf2_odometry` | Convierte la odometría de la instancia 1, publica el frame del target y `target/velocity`. |
| `pursuit_mode` | Modo de persecución pura basado en la posición. |
| `PN_mode` | Modo de navegación proporcional basado en posición y velocidad. |

Un ejecutable es un programa; un nodo es la instancia ROS 2 que ese programa
crea al arrancar. Normalmente aquí hay una relación uno a uno, pero no son
conceptos idénticos: el mismo ejecutable podría iniciar varias instancias con
nombres o namespaces distintos.

Un **ejecutable** es un archivo que el sistema puede iniciar. Una **instancia**
es una copia concreta de un programa en funcionamiento. Un **namespace** es un
prefijo de nombres que evita que dos copias choquen entre sí.

## Nodos auxiliares

| Ejecutable | Uso |
| --- | --- |
| `target_vehicle_odometry_subscriber` | Imprime por pantalla la odometría recibida del tópico sin prefijo. |
| `interceptor_vehicle_odometry_subscriber` | Imprime datos de odometría para diagnóstico. |
| `tf2_listener` | Muestra la transformación relativa y la velocidad del target una vez por segundo. |

Los suscriptores de odometría son herramientas de inspección, no forman parte
del cálculo de control. `tf2_listener` está comentado en el launch por defecto.

## Tabla de entradas y salidas

| Nodo | Entrada | Salida |
| --- | --- | --- |
| `target_tf2_odometry` | `/px4_1/fmu/out/vehicle_odometry` | `map -> target/base_link`, `target/velocity` |
| `interceptor_tf2_odometry` | `/fmu/out/vehicle_odometry` | `map -> interceptor/base_link` |
| `pursuit_mode` | tf2 target, odometría propia | setpoint de trayectoria |
| `PN_mode` | tf2 target, odometría propia, `target/velocity` | setpoint de trayectoria |
| suscriptores de diagnóstico | `VehicleOdometry` | texto en consola |
| `tf2_listener` | tf2 y odometría target | texto en consola |

La tabla es una vista de dependencias: si una entrada falta, no tiene sentido
dejar el nodo de control como primera sospecha.

## Parámetros

Los modos declaran estos parámetros:

| Parámetro | Valor por defecto | Uso |
| --- | --- | --- |
| `target_frame` | `target/base_link` | Frame tf2 que representa al target. |
| `target_velocity_topic` | `target/velocity` | Tópico de velocidad usado por PN. |

Los conversores declaran `vehicle_name`, con valor `interceptor` o `target`
según el ejecutable.

Los parámetros son configuración, no mensajes. Se consultan al crear el nodo y
permiten cambiar nombres sin modificar el binario. Para inspeccionarlos:

```bash
ros2 param list /pursuit_mode
ros2 param get /pursuit_mode target_frame
```

El nombre exacto del nodo depende de cómo lo registre `NodeWithMode` y de si se
usa un namespace.

## Inspección desde otra terminal

```bash
ros2 node list
ros2 topic list
ros2 topic echo /px4_1/fmu/out/vehicle_odometry
ros2 topic echo /target/velocity
ros2 topic hz /target/velocity
ros2 run tf2_tools view_frames
```

Otros comandos útiles:

```bash
ros2 node info /target_tf2_frame_publisher
ros2 topic info /target/velocity --verbose
ros2 interface show geometry_msgs/msg/TwistStamped
ros2 run tf2_ros tf2_echo map target/base_link
```

`node info` muestra conexiones, `topic info --verbose` muestra publishers,
subscribers y QoS, `interface show` enseña la estructura del mensaje y
`tf2_echo` imprime una transformación en tiempo real.

Los nombres absolutos de los tópicos comienzan por `/`. En el código de los
modos, `target/velocity` se usa como nombre relativo y ROS 2 lo resuelve dentro
del namespace actual.

## Qué mirar primero si algo falla

1. ¿Aparecen los dos `VehicleOdometry` con `ros2 topic list`?
2. ¿Está el agente conectado en el puerto `8888`?
3. ¿Existen los dos frames publicados por `tf2`?
4. ¿Está `target/velocity` publicando datos?
5. ¿Se ha cargado `source install/setup.bash` en la terminal?

Anota siempre tres cosas al diagnosticar: el nombre exacto que aparece en
`ros2 topic list`, el tipo que devuelve `ros2 topic type` y la frecuencia de
`ros2 topic hz`. Los nombres parecidos pero no idénticos son una causa muy
frecuente de errores ROS 2.

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Arquitectura y flujo de datos](Architecture.md) · ➡️ Siguiente: [Modos de guiado](Guidance-modes.md)
