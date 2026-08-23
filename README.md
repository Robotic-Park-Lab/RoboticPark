# RoboticPark

> 📖 Para entender las ramas de este repo y la guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/doc).

Repositorio principal de [Robotic Park Lab](https://github.com/Robotic-Park-Lab). Desde aquí se instala la infraestructura completa del laboratorio en un equipo nuevo — el resto de paquetes UNED, el sistema Vicon y las dependencias de simulación (Webots) — y se reproduce exactamente la configuración usada en publicaciones concretas del laboratorio.

#### Estructura
- **mars_supervisor_pkg**. Supervisión del sistema experimental (MARS).
- **measure_process_ros2_pkg**. Medición y registro de procesos durante los experimentos.
- **multi_agent_pkg**. Utilidades comunes para experimentos multi-agente que combinan varias plataformas del laboratorio.
- **roboticpark_config**. Configuración compartida por el resto de repositorios del laboratorio (lanzada como dependencia desde `uned_*_ros_pkg`).
- **scripts**. Utilidades auxiliares que no forman parte de ningún paquete ROS.

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

## Publicaciones asociadas :paperclip:
- Capítulo de libro sobre control — configuración reproducible desde la rama `benchmark` (ver arriba).
