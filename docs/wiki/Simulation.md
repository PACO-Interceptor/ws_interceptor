# Ejecución de la simulación

Esta página describe cómo poner varias piezas a funcionar al mismo tiempo.
Cuando se dice “otra terminal”, se trata de otra ventana de comandos; cada una
mantiene su propio proceso activo.

El launch del proyecto conecta ROS 2 con dos instancias PX4 SITL. PX4 SITL
simula el autopiloto y el vehículo; el Micro XRCE-DDS Agent hace de puente
entre PX4 y ROS 2.

## Qué se está simulando

SITL no es solo una ventana con un modelo 3D. Ejecuta el software de PX4 y sus
estimadores como procesos normales, de forma que ROS 2 recibe mensajes muy
parecidos a los de un vehículo real. Esto permite probar la integración sin
arriesgar hardware, aunque no reproduce todas las condiciones físicas.

Las instancias deben tener identidades diferentes:

- índice `0`: interceptor, topics sin `/px4_1/`;
- índice `1`: target, topics con `/px4_1/`.

El índice no es un nombre visual: forma parte del direccionamiento de los
mensajes. Si se intercambian los índices, el código puede seguir compilando
pero consumir datos del vehículo equivocado.

`udp4` significa comunicación UDP usando IPv4. UDP es una forma de enviar
paquetes por la red sin mantener una conexión permanente; aquí se usa dentro
del ordenador para que el agente reciba datos.

## Orden recomendado

Abre terminales separadas y carga ROS 2 y el workspace cuando corresponda.

```mermaid
%%{init: {"theme": "dark"}}%%
sequenceDiagram
    participant T1 as Terminal 1 (PX4 interceptor)
    participant T2 as Terminal 2 (PX4 target)
    participant T3 as Terminal 3 (launch del proyecto)

    T1->>T1: make px4_sitl gz_x500 (instancia 0)
    Note over T1: esperar a que termine el arranque
    T2->>T2: PX4_SIM_MODEL=gz_x500 ... px4 -i 1 (instancia 1)
    Note over T2: esperar a que termine el arranque
    T3->>T3: ros2 launch interceptor interceptor.launch.py
    T3->>T3: arranca Micro XRCE-DDS Agent (udp4, puerto 8888)
    T3->>T3: arranca conversores tf2, diagnóstico y modos de vuelo
    T1-->>T3: VehicleOdometry (instancia 0)
    T2-->>T3: VehicleOdometry (instancia 1)
```

El orden importa: si el launch arranca antes que las dos instancias PX4, sus
nodos simplemente no reciben odometría todavía y esperan; si el agente DDS no
está corriendo, ninguna de las dos instancias PX4 llega a ROS 2 aunque estén
"arrancadas".

### 1. Iniciar PX4 del interceptor

En el directorio de PX4:

```bash
make px4_sitl gz_x500
```

Esta es la instancia `0`, que publica sus temas sin el prefijo `px4_1`.

Espera a que PX4 termine su arranque antes de continuar. En una prueba real
conviene confirmar que el proceso está estable y que el vehículo aparece en la
herramienta de simulación.

### 2. Iniciar PX4 del target

En otra terminal de PX4:

```bash
PX4_SIM_MODEL=gz_x500 /build/px4_sitl_default/bin/px4 -i 1
```

La instancia `1` publica bajo `/px4_1/`.

La ruta `/build/px4_sitl_default/bin/px4` depende de dónde esté el árbol de
PX4. Si ese binario no existe, localiza el ejecutable generado en el build de
PX4 y conserva el argumento `-i 1`, que es el que asigna la instancia.

### 3. Iniciar el launch del proyecto

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch interceptor interceptor.launch.py
```

El launch inicia el agente con `udp4` en el puerto `8888`, los conversores de
odometría, los nodos de diagnóstico y los dos nodos de modo de vuelo.

> El launch no inicia PX4 SITL. Los dos comandos anteriores son obligatorios si
> se quiere probar con simulación.

## Cómo saber si el arranque funcionó

En una terminal nueva, después de hacer `source install/setup.bash`:

```bash
ros2 node list
ros2 topic list | grep -E 'vehicle_odometry|target/velocity'
ros2 topic hz /target/velocity
```

Deberías ver los nodos del paquete, los tópicos de odometría de las dos
instancias y una frecuencia aproximada de 10 Hz para `target/velocity`. La
frecuencia exacta puede variar; lo importante al principio es que existan datos
que cambien.

## Ejecutar un nodo individual

```bash
ros2 run interceptor pursuit_mode
ros2 run interceptor PN_mode
ros2 run interceptor target_tf2_odometry
```

No ejecutes dos copias del mismo nodo sin una razón clara: podrían publicar el
mismo transform o consumir recursos duplicados.

Para detener una prueba, detén primero el launch y después las instancias PX4.
Si dejas procesos antiguos activos, la siguiente prueba puede recibir mensajes
duplicados o fallar al abrir puertos.

## Nota sobre los modos

El launch actual inicia tanto `pursuit_mode` como `PN_mode`. Ambos se registran
como modos personalizados de PX4 mediante `px4_ros2_cpp`. Para entender sus
diferencias y sus datos necesarios, consulta [Modos de guiado](Guidance-modes.md).

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Instalación y compilación](Installation-and-build.md) · ➡️ Siguiente: [Arquitectura y flujo de datos](Architecture.md)
