# roboticpark_config

Paquete `ament_cmake` con la configuración de lanzamiento y entorno compartida por el resto de repositorios del laboratorio. Es la plantilla original de la que `uned_crazyflie_config`, `uned_kheperaiv_config` y `uned_swarm_config` (en `uned_multi_agent_ros_pkg`) adaptaron cada uno su propio `experience.launch.py`/`generic.launch.py`.

## Estructura

- **`launch/experience.launch.py`**: el único archivo de lanzamiento parametrizado — `ros2 launch roboticpark_config experience.launch.py config_file:=<experiencia>.yaml`. Lee las secciones `Operation` (Webots/Gazebo, fichero de mundo, físico vs. simulado), `Architecture` (`centralized` lanza un único nodo de coordinación por `pkg`/`executable`/`name`, coincidiendo con los nodos de `mars_supervisor_pkg`/`multi_agent_pkg`; `distributed_ros2` no lanza nada extra aquí), `Robots` (`type`/`name` por robot, distinguidos por `dron*`/`khepera*` en el nombre), `CPU_Monitoring` (lanza el nodo de `measure_process_ros2_pkg`), e `Interface` (RQT/RViz) de un `.yaml` en `resources/`.
- **`resources/`**: cerca de 90 ficheros `.yaml` de experiencia (ver la tabla "Experiencias disponibles" del README raíz), el URDF de los dos robots (`crazyflie.urdf`, `kheperaiv.urdf`), y `models/RoboticLab/` — un modelo SDF nativo de Gazebo del espacio del laboratorio (`model.sdf`/`model.config`).
- **`rviz/`, `rqt/`**: configuraciones de RViz y perspectivas de RQT, un conjunto por familia de experiencia (`ControlFormation*`, `LagrangeMultipliers_*`, `IROS_AffineFormation_*`, ...).
- **`worlds/`**: mundos de Webots para las mismas familias de experiencia, más sus mallas compartidas.
- **`scripts/`**: utilidades de desarrollo (no instaladas por el paquete) para generar ficheros Webots/RViz/experiencia de una nueva variante con N robots, en vez de editarlos a mano. Se ejecutan manualmente, desde `scripts/`:
  - `generate_formation_assets.py` — mundos `.wbt` de Webots y configuraciones `.rviz`, a partir de una lista de robots (nombre/tipo/posición). Los ficheros RViz se generan clonando el bloque de Display por robot de un fichero ya existente y probado (`rviz/IROS_AffineFormation_N05.rviz`), en vez de escribir el YAML de RViz desde cero.
  - `generate_experience_yaml.py` — configs `.yaml` de experiencia, usando `resources/IROS_AffineFormation_config_N05.yaml` como plantilla estructural.
  Para añadir una N nueva, editar las listas de robots al final de cada script y volver a ejecutar; ambos son idempotentes (re-ejecutar sobrescribe sus propios ficheros de salida).

## Tests

Ninguno más allá de los tests de lint estándar `ament_lint_auto`/`ament_lint_common` (que en la práctica no producen ningún resultado de test — ver `AUDIT.md` en la rama `doc`) — este paquete no tiene nodos propios, solo archivos de lanzamiento y recursos.
