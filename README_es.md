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

`roboticpark_config/resources/` tiene actualmente cerca de 90 ficheros `.yaml` de experiencia, agrupados por línea de investigación (`ControlFormation*`, `LagrangeMultipliers_{cone,sphere}*`, `IROS_AffineFormation*`, `Morphing_*`, configuraciones de identificación y ajuste por relé `Gimbal*`/`gimbal_*`, `CoverageBenchmark*`, `OpenLoop_Crazyflie_*`, `SphereFormation*`, `Crazyflie_Demo*`, `Khepera_rele*`, más `default_config.yaml`/`test.yaml`). Esta tabla está pensada para que Francisco la vaya rellenando de forma incremental a medida que (re)documente cada experiencia — añade una fila por cada experiencia que quieras documentar aquí:

| `config_file` | Descripción |
|---|---|
| _(por rellenar)_ | |

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
