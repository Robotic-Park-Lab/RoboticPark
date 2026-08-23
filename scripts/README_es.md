# scripts

Utilidades auxiliares que no forman parte de ningún paquete ROS: el pipeline de Matlab detrás del benchmark del capítulo de libro (rama `benchmark`) y su script de post-procesado de bags.

## Pipeline de Matlab del benchmark (`RP_benchmark.m`, `problem1.m`, `benchmark_evaluation.m`)

Estos tres ficheros juntos son el pipeline real de generación/evaluación detrás del benchmark publicado en el capítulo de libro, reproducido desde la rama `benchmark` (ver la sección "Reproducir un experimento publicado" del README raíz) — merece la pena enlazarlo desde la tabla "Experiencias disponibles" de `roboticpark_config` en cuanto se documenten allí los ficheros `ControlFormation*` correspondientes.

- **`problem1.m`** (función): dado un número de robots `N` y un radio `R`, calcula los drones necesarios para formar una semiesfera usando la máxima distancia horizontal permitida entre drones — devuelve `D` (distancias objetivo entre robots), `E` (las aristas del grafo resultante), `P` (posiciones 3D).
- **`RP_benchmark.m`** (script): llama a `problem1(N, R, 0)` y, a partir de su salida, **genera desde cero una configuración de experiencia completa** — `default_config.yaml` (el mismo esquema `Operation`/`Experience`/`Architecture`/`CPU_Monitoring`/`Interface`/`Data_Logging`/`Robots`/`Supervisor`/`Other` que lee `roboticpark_config/launch/experience.launch.py`, con una entrada `Robots.RobotNN` por dron, su pose calculada, y sus restricciones de distancia `task.relationship` a partir de `E`/`D`), más un `default_world.wbt` a juego (mundo de Webots con `N` Crazyflies en las posiciones calculadas) y un `default_config.rviz`, todo escrito en una nueva carpeta `ControlFormationNN_files/`. Esto es lo que generó la familia de ficheros de experiencia `ControlFormation*` en `roboticpark_config/resources/`.
- **`benchmark_evaluation.m`** (script): relee el `default_config.yaml` de un experimento grabado (desde una carpeta de dataset con fecha, p. ej. `2024-01-20-22-09/`) para reconstruir la topología de formación, y luego evalúa los datos registrados contra ella — este es el paso real de evaluación/métricas del benchmark, ejecutado después de grabar un `ros2 bag` de una ejecución real o simulada.

## `bag2csv_benchmark.py`

Convierte cada topic de un `ros2 bag` en un fichero `.csv`, uno por topic, según el tipo de mensaje — el original del que se adaptó `bag2csv.py` de `uned_crazyflie_ros_pkg/scripts/Python/` en este mismo laboratorio (esa versión añadió 2 tipos de mensaje más soportados; esta es la fuente original del post-procesado de bags del propio benchmark).

## Tests

Ninguno — son scripts de análisis independientes (2 lenguajes, ninguno es un paquete ROS), algo que `colcon test` no cubre.
