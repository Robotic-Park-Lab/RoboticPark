# RoboticPark

Repositorio principal de [Robotic Park Lab](https://github.com/Robotic-Park-Lab). Desde aquí se instala la infraestructura completa del laboratorio en un equipo nuevo, y se reproduce la configuración exacta usada en publicaciones concretas.

Esta rama (`main`) es solo un índice — no contiene el código del workspace. Para trabajar con el laboratorio, ve a:

- 🛠️ **[`humble-dev`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/humble-dev)** — desarrollo activo, ROS 2 Humble. Aquí están `install.sh` y los paquetes del workspace.
- 📕 **[`benchmark`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/benchmark)** — configuración congelada de un capítulo de libro publicado sobre control. No se modifica ni se renombra.
- 📖 **[`doc`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/doc)** — qué es cada rama de cada repo del laboratorio, y la guía de contribución.

## Instalación rápida
```
mkdir -p ~/roboticpark_ws/src && cd ~/roboticpark_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark && ./install.sh
```
Ver el README de [`humble-dev`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/humble-dev) para las opciones del instalador (distro, modo benchmark) y la lista completa de paquetes del laboratorio.

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es
