# RoboticPark

> 📖 Para entender las ramas de este repo y la guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/doc).

Repositorio principal de [Robotic Park Lab](https://github.com/Robotic-Park-Lab). Desde aquí se instala la infraestructura completa del laboratorio en un equipo nuevo — el resto de paquetes UNED, el sistema Vicon y las dependencias de simulación (Webots) — y se reproduce exactamente la configuración usada en publicaciones concretas del laboratorio.

#### Estructura
- **mars_supervisor_pkg**. Supervisión del sistema experimental (MARS).
- **measure_process_ros2_pkg**. Medición y registro de procesos durante los experimentos.
- **multi_agent_pkg**. Utilidades comunes para experimentos multi-agente que combinan varias plataformas del laboratorio.
- **roboticpark_config**. Configuración compartida por el resto de repositorios del laboratorio (lanzada como dependencia desde `uned_*_ros_pkg`).
- **roboticpark_interfaces**. Mensajes, servicios y acciones ROS 2 comunes a varias plataformas.
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

## Reproducir un experimento publicado :paperclip:

La rama `benchmark` respalda el capítulo de libro sobre control de Francisco Mañas y **no se modifica ni se renombra**. Para clonar exactamente esa configuración de RoboticPark:
```
mkdir -p ~/roboticpark_benchmark_ws/src
cd ~/roboticpark_benchmark_ws/src
git clone -b benchmark https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark
chmod +x install.sh
./install.sh
```
> **TO-DO**: `install.sh` todavía no es consciente de la rama desde la que se ejecuta — hoy siempre clona las ramas `humble-dev` de `uned_crazyflie_ros_pkg`, `ros2-vicon-receiver` y `uned_swarm_ros_pkg`, independientemente de si RoboticPark se clonó en `humble-dev` o en `benchmark`. Hasta que el instalador se parametrice (clonar también las ramas `benchmark` de esos tres repos cuando corresponda), reproducir el benchmark completo requiere clonar manualmente las ramas `benchmark` de `uned_kheperaIV_ros_pkg`, `uned_crazyflie_ros_pkg` y `uned_swarm_ros_pkg` en lugar de dejar que `install.sh` las traiga.

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones asociadas :paperclip:
- Capítulo de libro sobre control — configuración reproducible desde la rama `benchmark` (ver arriba).
