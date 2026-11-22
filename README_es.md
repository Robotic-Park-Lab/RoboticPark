# RoboticPark

> 📖 Para entender las ramas de este repo y la guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/doc).

Repositorio principal de [Robotic Park Lab](https://github.com/Robotic-Park-Lab). Desde aquí se instala la infraestructura completa del laboratorio en un equipo nuevo — el resto de paquetes UNED, el sistema Vicon y las dependencias de simulación (Webots) — y se reproduce exactamente la configuración usada en publicaciones concretas del laboratorio.

#### Estructura
- **[mars_supervisor_pkg](mars_supervisor_pkg/README_es.md)**. Supervisión del sistema experimental (MARS).
- **[measure_process_ros2_pkg](measure_process_ros2_pkg/README_es.md)**. Medición y registro de procesos del sistema (CPU/memoria) durante los experimentos.
- **[multi_agent_pkg](multi_agent_pkg/README_es.md)**. Utilidades comunes para experimentos multi-agente que combinan varias plataformas del laboratorio — matemáticas de control de formación consumidas por `uned_crazyflie_driver`/`uned_kheperaiv_webots`, y la ley de control de pastoreo/formación afín activa del laboratorio.
- **[roboticpark_config](roboticpark_config/README_es.md)**. Configuración compartida por el resto de repositorios del laboratorio (lanzada como dependencia desde `uned_*_ros_pkg`).
- **[scripts](scripts/README_es.md)**. Utilidades auxiliares que no forman parte de ningún paquete ROS.

## Instalación :book:

Desarrollo activo, sobre ROS 2 Humble:
```
mkdir -p ~/roboticpark_ws/src
cd ~/roboticpark_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark
chmod +x install.sh
./install.sh
```

`install.sh` detecta el modo (desarrollo o benchmark) a partir de la rama en la que clonaste RoboticPark, pero también admite argumentos explícitos:
```
./install.sh [--ros-distro humble] [--benchmark|--dev] [--workspace DIR]
```
- `--ros-distro`: hoy solo `humble` tiene rama `*-dev` activa en los repos del laboratorio.
- `--benchmark` / `--dev`: fuerza el modo sin depender de la rama actual.
- `--workspace`: raíz del workspace, si no quieres `~/roboticpark_ws`.

## Uso 🔧

### Lanzar una experiencia

Igual que los repositorios de Crazyflie, Khepera IV y multi-agente, `roboticpark_config` tiene un **único archivo de lanzamiento parametrizado**, `roboticpark_config/launch/experience.launch.py`. Cada experiencia es un fichero `.yaml` en `roboticpark_config/resources/`:
```
ros2 launch roboticpark_config experience.launch.py config_file:=<experiencia>.yaml
```
Ver [roboticpark_config/README_es.md](roboticpark_config/README_es.md) para el esquema completo de secciones (`Operation`/`Architecture`/`Robots`/`CPU_Monitoring`/`Interface`) y cómo se comparte con los propios archivos de lanzamiento de experiencia del resto de repositorios.

### Experiencias disponibles

`roboticpark_config/resources/` tiene 65 ficheros `.yaml` de experiencia (los `*_topics.yaml` que los acompañan son guiones del supervisor, no experiencias). El catálogo completo — una fila por experiencia con su modo, robots, ley de formación y nodos adicionales, agrupado por línea de investigación — está en **[LAUNCHES_es.md](LAUNCHES_es.md)**:

| Línea de investigación | Prefijo de `config_file` |
|---|---|
| Formación triangulada con líderes (malla, 12 drones) | `AdaptiveFormation_N12_*` |
| Formación por distancias con referencia al origen (7–20 drones) | `ControlFormation*` |
| Formaciones sobre superficies — multiplicadores de Lagrange (esfera/cono) | `LagrangeMultipliers_*` |
| Formación afín y pastoreo de un Khepera IV | `IROS_AffineFormation_*`, `Herding_*` |
| Morphing con flota mixta (esfera ↔ elipsoide) | `Morphing_*` |
| Formación esférica (real/híbrida) | `SphereFormation*` |
| Benchmark de escalabilidad (50 drones) | `CoverageBenchmark_config` |
| Demos de Crazyflie / Khepera IV, gimbal e identificación | `Crazyflie_*`, `KheperaIV_*`, `Khepera_rele` |
| Plantillas y pruebas | `default_config`, `demo_benchmark*`, `test` |

Las experiencias nuevas se pueden generar con el [configurador web](https://robotic-park-lab.github.io/configurador/).

### Scripts de benchmark en Matlab

Ver [scripts/README_es.md](scripts/README_es.md) para `RP_benchmark.m`/`benchmark_evaluation.m`/`problem1.m` y el script de post-procesado `bag2csv_benchmark.py`.

## Reproducir un experimento publicado :paperclip:

La rama `benchmark` respalda el capítulo de libro sobre control de Francisco Mañas y **no se modifica ni se renombra**. Para reproducir exactamente esa configuración:
```
mkdir -p ~/roboticpark_benchmark_ws/src
cd ~/roboticpark_benchmark_ws/src
git clone -b benchmark https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark
chmod +x install.sh
./install.sh
```
`install.sh` detecta que RoboticPark está en `benchmark` y clona automáticamente las ramas `benchmark` de `uned_crazyflie_ros_pkg`, `uned_kheperaIV_ros_pkg` y `uned_multi_agent_ros_pkg` (no de `ros2-vicon-receiver`, que no tiene rama `benchmark` propia).

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones relacionadas :paperclip:

Cada entrada de abajo debería listar, junto a la cita, el fichero de lanzamiento/configuración concreto de `roboticpark_config` usado en los experimentos de esa publicación (p. ej. `experience.launch.py config_file:=<fichero>.yaml`, o `benchmark` para la rama congelada) — añade entradas conforme se vayan documentando.

| Publicación | Lanzamiento / configuración usada |
|---|---|
| Capítulo de libro sobre control — Francisco Mañas | rama `benchmark` (ver "Reproducir un experimento publicado" arriba) |
| _(por rellenar)_ | |
