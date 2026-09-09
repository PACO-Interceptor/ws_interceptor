# Mapa completo de archivos y dependencias

Esta página cubre **todos los archivos que forman parte de `src/` y `.github/`**.
El inventario actual contiene 591 archivos:

| Zona | Archivos | Propósito |
| --- | ---: | --- |
| `src/interceptor` | 13 | Código propio del proyecto. |
| `src/px4_msgs` | 289 | Definiciones ROS 2 equivalentes a mensajes, servicios y acciones de PX4. |
| `src/px4_ros_com` | 28 | Ejemplos y utilidades de comunicación entre ROS 2 y PX4. |
| `src/px4-ros2-interface-lib` | 258 | Biblioteca vendorizada para registrar modos PX4 y enviar setpoints. |
| `.github/workflows` | 3 | Automatización de CI, resúmenes de issues y releases. |

## Cómo consultar este mapa

No hace falta abrir los 591 archivos en orden. El número indica cuántos archivos
hay, no una secuencia de lectura. Empieza por `src/interceptor`,
porque es el código de este proyecto; después consulta solo la dependencia que
necesites entender.

Cuando veas una carpeta, piensa que es una caja con un propósito. Cuando veas un
archivo, pregunta si contiene instrucciones, datos, configuración o
documentación. Esta clasificación es más útil que memorizar los nombres.

## Cómo interpretar el inventario

No todos los archivos son algoritmos ejecutados por el interceptor. Hay cinco
tipos principales:

1. **Código propio**: `src/interceptor`.
2. **Dependencias vendorizadas**: los otros tres paquetes dentro de `src/`.
3. **Interfaces**: archivos `.msg` y `.srv` que describen estructuras de datos.
4. **Configuración y construcción**: `CMakeLists.txt`, `package.xml`, YAML,
   JSON, repositorios y archivos de formato.
5. **Automatización y documentación**: workflows, README, scripts y documentos
   de contribución.

Una dependencia vendorizada no se debe modificar para arreglar un comportamiento
del interceptor. Primero se debe entender qué interfaz ofrece y cambiar solo
`src/interceptor`, salvo que el objetivo sea actualizar esa dependencia.

## `src/interceptor`: los 13 archivos propios

| Archivo | Qué hace |
| --- | --- |
| [`CMakeLists.txt`](../../src/interceptor/CMakeLists.txt) | Declara dependencias, compila los siete ejecutables e instala binarios y launch. |
| [`package.xml`](../../src/interceptor/package.xml) | Declara el nombre, versión, licencia y dependencias ROS 2 del paquete. |
| [`README.md`](../../src/interceptor/README.md) | Guía breve de instalación y enlaces oficiales. |
| [`LICENSE`](../../src/interceptor/LICENSE) | Condiciones legales del paquete propio. |
| [`.gitignore`](../../src/interceptor/.gitignore) | Evita guardar artefactos locales del paquete. |
| [`launch/interceptor.launch.py`](../../src/interceptor/launch/interceptor.launch.py) | Inicia agente DDS, conversores, diagnóstico y modos. |
| [`src/interceptor_tf2_odometry.cpp`](../../src/interceptor/src/interceptor_tf2_odometry.cpp) | Convierte odometría de PX4 y publica el frame del interceptor. |
| [`src/target_tf2_odometry.cpp`](../../src/interceptor/src/target_tf2_odometry.cpp) | Publica el frame del target y su velocidad ENU. |
| [`src/pursuit_mode.cpp`](../../src/interceptor/src/pursuit_mode.cpp) | Modo de persecución pura basado en posición. |
| [`src/PN_mode.cpp`](../../src/interceptor/src/PN_mode.cpp) | Modo de navegación proporcional basado en posición y velocidad. |
| [`src/tf2_listener.cpp`](../../src/interceptor/src/tf2_listener.cpp) | Herramienta para imprimir transformaciones relativas. |
| [`src/target_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/target_vehicle_odometry_subscriber.cpp) | Imprime una odometría para diagnóstico. |
| [`src/interceptor_vehicle_odometry_subscriber.cpp`](../../src/interceptor/src/interceptor_vehicle_odometry_subscriber.cpp) | Imprime otra odometría para diagnóstico. |

La explicación línea por línea de estos archivos está en
[Lectura guiada del código](Code-walkthrough.md).

## `src/px4_msgs`: mensajes y servicios PX4

Este paquete no implementa el control de vuelo. Define los tipos que permiten
que ROS 2 intercambie datos con PX4.

| Zona | Archivos actuales | Explicación |
| --- | ---: | --- |
| `msg/*.msg` | 261 | Cada archivo describe campos y tipos de un mensaje uORB de PX4. |
| `srv/VehicleCommand.srv` | 1 | Describe una petición/respuesta para comandos de vehículo. |
| `CMakeLists.txt` | 1 | Genera código ROS 2 a partir de `.msg` y `.srv`. |
| `package.xml` | 1 | Declara generadores y runtime de interfaces. |
| `README.md` | 1 | Documenta el paquete oficial y su uso. |
| `CHANGELOG.rst` | 1 | Historial de cambios. |
| `CONTRIBUTING.md` | 1 | Reglas para contribuir al paquete externo. |
| `QUALITY_DECLARATION.md` | 1 | Declaración de calidad del paquete. |
| `CODE_OF_CONDUCT.md` | 1 | Normas de convivencia. |
| `SECURITY.md` | 1 | Proceso de reporte de vulnerabilidades. |
| `LICENSE` | 1 | Licencia BSD-3-Clause. |
| `.github/` | 15 | Plantillas, configuración de dependabot y workflows del upstream. |
| `container/` | 2 | Dockerfile y scripts para construir paquetes Debian. |
| `scripts/` | 7 | Automatización de changelog, releases y builds. |
| `.gitignore`, `.dockerignore`, `.pre-commit-config.yaml` | 4 | Exclusiones y herramientas de desarrollo. |

### Cómo leer un `.msg`

Un `.msg` contiene líneas como `float32 x` o `uint64 timestamp`. No es una
clase C++ escrita a mano: el generador de ROS 2 convierte esa descripción en
tipos de programación. Por eso el interceptor puede escribir
`px4_msgs::msg::VehicleOdometry` y acceder a `position`, `velocity` o `q`.

Los 261 mensajes se organizan por capacidad de PX4: odometría, sensores,
actuadores, navegación, estado, setpoints y comandos. Cada archivo concreto
describe un contrato de datos; cambiarlo puede romper la compatibilidad con la
versión de PX4 que publica esos campos.

## `src/px4_ros_com`: puente y ejemplos ROS 2/PX4

| Zona | Archivos actuales | Explicación |
| --- | ---: | --- |
| `CMakeLists.txt` y `package.xml` | 2 | Construyen el paquete y declaran Eigen, ROS 2 y `px4_msgs`. |
| `include/px4_ros_com/` | 1 header | Declara las conversiones de marcos, usadas por el interceptor. |
| `src/lib/frame_transforms.cpp` | 1 | Implementa conversiones NED/ENU y orientaciones PX4/ROS. |
| `src/examples/` | 11 | Ejemplos C++ de listeners, advertisers y offboard. |
| `src/examples/offboard_py/` | 1 | Ejemplo Python de control offboard. |
| `launch/` | 1 | Launch de ejemplos de comunicación. |
| `test/` | 2 | Pruebas del paquete externo. |
| `scripts/` | 3 | Scripts de instalación y compilación. |
| `README.md`, `LICENSE` | 2 | Documentación y licencia. |
| `.github/`, `.vscode/`, `.gitignore` | 5 | CI, configuración del editor y exclusiones. |

El archivo que usa directamente nuestro código es
`include/px4_ros_com/frame_transforms.h`. Sus funciones evitan duplicar a mano
los cambios de signos y ejes entre NED y ENU.

## `src/px4-ros2-interface-lib`: biblioteca de modos PX4

Esta dependencia proporciona `px4_ros2::ModeBase`,
`px4_ros2::NodeWithMode`, `OdometryLocalPosition` y
`TrajectorySetpointType`.

| Zona | Archivos actuales | Explicación |
| --- | ---: | --- |
| `px4_ros2_cpp/` | 136 aprox. | Biblioteca C++: headers públicos, implementación, componentes, odometría, navegación, setpoints y tests. |
| `px4_ros2_py/` | 14 aprox. | Bindings y paquete Python experimental. |
| `examples/cpp/` | 47 aprox. | Modos y ejemplos C++: goto, misión, rover, VTOL, manual y navegación. |
| `examples/python/` | 11 aprox. | Ejemplos Python de modos. |
| `mission/` | 8 aprox. | Esquemas y ejemplos JSON para misiones. |
| `python_docs/` | 4 aprox. | Configuración y páginas de documentación Python. |
| `scripts/` | 5 aprox. | Comprobación de compatibilidad, topics, clang-tidy y Doxygen. |
| `CMakeLists.txt`, `package.xml` y `rosdep-*.yaml` | 10 aprox. | Construcción y dependencias por distribución ROS. |
| `.github/` | 14 aprox. | CI, lint, publicación, ramas y validación. |
| `.clang-*`, `.pre-commit-config.yaml`, `.vscode/` | 8 aprox. | Formato, análisis estático, hooks y editor. |
| `README.md`, `LICENSE`, `Doxyfile`, `.gitignore` | 4 | Documentación, licencia y generación de API. |

### Qué ocurre cuando usamos esta biblioteca

`pursuit_mode.cpp` no publica directamente un mensaje PX4 de bajo nivel. Hereda
de `ModeBase`, crea un `TrajectorySetpointType` y deja que la biblioteca:

1. registre el modo con PX4;
2. compruebe compatibilidad de mensajes;
3. gestione el estado del modo;
4. traduzca velocidad/aceleración a los mensajes que PX4 espera.

Los ejemplos vendorizados son material didáctico útil, pero no forman parte del
algoritmo del interceptor.

## Cómo viajan las dependencias durante la compilación

La relación entre paquetes puede imaginarse como una cadena:

```text
px4_msgs
    └── px4_ros_com
            └── interceptor
px4_msgs
    └── px4_ros2_cpp
            └── interceptor
```

`px4_msgs` define tipos. `px4_ros_com` usa esos tipos y ofrece conversiones.
`px4_ros2_cpp` usa mensajes y publica la infraestructura de modos. Finalmente,
`interceptor` combina esas piezas en sus nodos y algoritmos.

Por eso `colcon build --packages-up-to interceptor` puede construir más de un
paquete aunque el cambio esté en un solo `.cpp`: necesita tener disponible toda
la cadena previa.

## Extensiones de archivo como guía de lectura

| Extensión | Pregunta que ayuda a responder |
| --- | --- |
| `.cpp` | ¿Qué comportamiento ejecutable implementa? |
| `.hpp`/`.h` | ¿Qué interfaz declara para otros archivos? |
| `.msg` | ¿Qué campos viajan por un mensaje ROS 2? |
| `.srv` | ¿Qué petición y respuesta define un servicio? |
| `.xml` | ¿Qué metadatos o dependencias declara? |
| `CMakeLists.txt` | ¿Cómo se configura y enlaza la compilación? |
| `.py` | ¿Es launch, herramienta, ejemplo o binding? |
| `.yaml`/`.yml` | ¿Es configuración de herramientas o automatización CI? |
| `.sh`/`.bash` | ¿Qué pasos repetitivos de shell se automatizan? |
| `.rst`/`.md` | ¿Qué instrucciones o decisiones documenta? |
| `.json` | ¿Qué datos de ejemplo o misión se representan? |

La extensión no lo explica todo, pero permite decidir qué leer primero cuando se
entra en una carpeta desconocida.

## Qué pertenece al producto y qué pertenece al entorno

El producto que el equipo está desarrollando es principalmente
`src/interceptor`. El resto del workspace hace posible compilarlo y conectarlo
con PX4. `.github/workflows` automatiza el mantenimiento del repositorio, pero
no se ejecuta durante el vuelo.

Esta distinción ayuda a decidir dónde abrir un issue o proponer un cambio:

- fallo en el algoritmo o en un tópico propio: `src/interceptor`;
- mensaje ausente o incompatible: revisar `px4_msgs` y la versión de PX4;
- conversión de coordenadas: revisar `px4_ros_com`;
- registro del modo/setpoint: revisar `px4-ros2-interface-lib`;
- compilación o pruebas automáticas: raíz `.github` y `CMakeLists.txt`;
- documentación o instrucciones: README y wiki.

## Límite de la documentación de terceros

Los inventarios de terceros indican qué hay y cómo se relaciona con el proyecto,
pero no sustituyen sus manuales ni describen cada línea de cientos de archivos.
Cuando sea necesario modificar una dependencia, hay que consultar la versión
upstream correspondiente, su licencia, su changelog y sus pruebas. Así se evita
atribuir al equipo interceptor una implementación que realmente mantiene otro
proyecto.

## `.github/workflows`: automatización del repositorio

### `ci-build.yml`

Se ejecuta en `push` y pull request hacia `main` o `master`. Usa Ubuntu 24.04
con `ros:jazzy-ros-base`, instala `colcon`, `rosdep` y dependencias, compila
hasta `interceptor`, ejecuta tests y muestra el resultado. Es la reproducción
automática de los comandos de [Instalación y compilación](Installation-and-build.md).

### `summary.yml`

Se activa cuando se abre un issue. Da permisos para leer modelos y escribir
issues, llama a `actions/ai-inference@v1` para producir un resumen y publica un
comentario usando `gh issue comment`. El título y cuerpo del issue se tratan como
texto no confiable; el prompt indica que no se deben obedecer instrucciones
incluidas dentro de ese texto.

### `tagging.yml`

Se ejecuta después de que termine correctamente el workflow de CI en `main`, o
manualmente. Usa `github-tag-action` para calcular una etiqueta semántica y
`action-gh-release` para crear una release con notas automáticas. Necesita
permiso `contents: write`, por lo que no se debe modificar sin entender sus
efectos sobre publicaciones reales.

## Archivos `.github` dentro de dependencias

Los directorios `.github` de `px4_msgs`, `px4_ros_com` y
`px4-ros2-interface-lib` pertenecen a esos proyectos upstream. Contienen
workflows de compilación/lint/release, plantillas de pull request, dependabot,
codeowners y configuración de seguridad. No controlan directamente el workflow
raíz de `ws_interceptor`; cada repositorio conserva sus propias automatizaciones
porque las dependencias se han copiado dentro de este workspace.

## Regla práctica para cualquier archivo nuevo

Antes de modificar un archivo, pregunta:

1. ¿Está bajo `src/interceptor`? Entonces probablemente es código que mantenemos.
2. ¿Está bajo otro paquete de `src`? Entonces es una dependencia externa y hay que
   consultar su upstream y versión.
3. ¿Es `.msg` o `.srv`? Cambiarlo altera un contrato de comunicación.
4. ¿Es `CMakeLists.txt` o `package.xml`? Cambiarlo altera compilación o runtime.
5. ¿Está bajo `.github`? Puede alterar CI, permisos o releases.

Este mapa es el índice de lectura. Para comprender las líneas de los archivos
propios, continúa con [Lectura guiada del código](Code-walkthrough.md) y para
comprobar el sistema usa [Referencia rápida y solución de problemas](Quick-reference-and-troubleshooting.md).

---

🏠 [Inicio](Home.md) · ⬅️ Anterior: [Análisis detallado de cada archivo propio](Line-by-line-code-analysis.md) · ➡️ Siguiente: [Referencia y solución de problemas](Quick-reference-and-troubleshooting.md)
