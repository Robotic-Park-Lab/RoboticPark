# mars_supervisor_pkg

Paquete `ament_python` con un nodo de supervisión/orquestación para experimentos MARS (Multi-Agent Robotic Systems).

## Nodos

- **`supervisor_node`** (`supervisor_node.py`, nombre de nodo `supervisor_node`): `Supervisor`, un secuenciador de comandos guionizado genérico. Lee dos ficheros vía parámetros ROS — `config` (un `.yaml` de experiencia, usado aquí solo para comprobar `Data_Logging.enable` y copiarse junto a la carpeta de log de la ejecución) y `file` (un script de comandos: `cmd00`, `cmd01`, ... en orden, cada uno con `topic`/`type`/`value` y un `trigger`, ya sea un retardo fijo o esperar un valor concreto en un topic suscrito). Publica mensajes `String`/`PoseStamped`/`Float64` en los topics declarados en la sección `config.publisher` del fichero de comandos. El mismo patrón de secuencia guionizada que el nodo `sequencer` de `uned_crazyflie_missions`, desarrollado de forma independiente en este repositorio.

Este paquete tenía también una segunda copia, divergente, del nodo de control de pastoreo/formación afín (`affine_formation_node.py`), presente también en `multi_agent_pkg`. Eliminada el 2026-08-23 en favor de la versión más activa de `multi_agent_pkg` — ver el README de ese paquete.

## Dependencias

`rclpy`, `std_msgs`, `geometry_msgs`, `tf_transformations` — declaradas en `package.xml`/`setup.py`.

## Tests

Sin tests funcionales. `ament_pep257` pasa; `ament_flake8` está en rojo — verificado con un `colcon test` real y aislado: 46 avisos de estilo (ver `AUDIT.md` en la rama `doc` para el panorama completo de todos los paquetes de RoboticPark). `Supervisor` es un nodo `rclpy` completo con el cableado de topics/parámetros y la lógica de lectura de ficheros hecha directamente desde sus métodos; nada de ello se ha extraído a funciones puras testeables unitariamente.
