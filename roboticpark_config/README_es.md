# roboticpark_config

Paquete `ament_cmake` con la configuración de lanzamiento y entorno compartida por el resto de repositorios del laboratorio. Es la plantilla original de la que `uned_crazyflie_config`, `uned_kheperaiv_config` y `uned_swarm_config` (en `uned_multi_agent_ros_pkg`) adaptaron cada uno su propio `experience.launch.py`/`generic.launch.py`.

## Estructura

- **`launch/experience.launch.py`**: el único archivo de lanzamiento parametrizado — `ros2 launch roboticpark_config experience.launch.py config_file:=<experiencia>.yaml`. Lee las secciones `Operation` (Webots/Gazebo, fichero de mundo, físico vs. simulado), `Architecture` (`centralized` lanza un único nodo de coordinación por `pkg`/`executable`/`name`, coincidiendo con los nodos de `mars_supervisor_pkg`/`multi_agent_pkg`; `distributed_ros2` no lanza nada extra aquí), `Robots` (`type`/`name` por robot, distinguidos por `dron*`/`khepera*` en el nombre), `CPU_Monitoring` (lanza el nodo de `measure_process_ros2_pkg`), e `Interface` (RQT/RViz) de un `.yaml` en `resources/`.
- **`resources/`**: cerca de 90 ficheros `.yaml` de experiencia (ver la tabla "Experiencias disponibles" del README raíz), el URDF de los dos robots (`crazyflie.urdf`, `kheperaiv.urdf`), y `models/RoboticLab/` — un modelo SDF nativo de Gazebo del espacio del laboratorio (`model.sdf`/`model.config`).
- **`rviz/`, `rqt/`**: configuraciones de RViz y perspectivas de RQT, un conjunto por familia de experiencia (`ControlFormation*`, `LagrangeMultipliers_*`, `IROS_AffineFormation_*`, ...).
- **`worlds/`**: mundos de Webots para las mismas familias de experiencia, más sus mallas compartidas.

## Tests

Ninguno más allá de los tests de lint estándar `ament_lint_auto`/`ament_lint_common` (que en la práctica no producen ningún resultado de test — ver `AUDIT.md` en la rama `doc`) — este paquete no tiene nodos propios, solo archivos de lanzamiento y recursos.
