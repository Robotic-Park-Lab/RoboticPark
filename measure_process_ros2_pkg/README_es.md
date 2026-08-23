# measure_process_ros2_pkg

Paquete `ament_python` con un único nodo para medir y registrar el uso de recursos del sistema/procesos durante los experimentos.

## Nodos

- **`measure_process`** (`measure_process.py`, nombre de nodo `measure_process`): `MeasureProcess`, publica periódicamente (parámetro `process_period`, 0.5s por defecto) estadísticas generales del sistema — número de CPUs, %CPU, %memoria, y las medias de carga a 1/5/15 minutos como porcentaje del número de CPUs — en `cpu_stats` (`Float64MultiArray`), vía [`psutil`](https://psutil.readthedocs.io/). También sigue procesos concretos nombrados en su parámetro `process_name` (separados por comas, p. ej. `gzserver, webots`) y publica el %CPU de cada uno en `<process_name>_cpu` (los guiones del nombre se sustituyen por guiones bajos para el topic). Se usa para correlacionar la carga de CPU del simulador/driver con los resultados del experimento (ver el uso de `measure_process_ros2_pkg` desde la sección `CPU_Monitoring` de experiencia de `roboticpark_config`).

## Dependencias

`rclpy`, `std_msgs` — declaradas en `package.xml`. También hace falta la librería de terceros `psutil` (no es un paquete ROS resoluble por rosdep — instalar aparte, p. ej. `pip install psutil`).

## Tests

Sin tests funcionales. `ament_pep257` pasa; `ament_flake8` está en rojo — verificado con un `colcon test` real y aislado: 11 avisos de estilo (ver `AUDIT.md` en la rama `doc` para el panorama completo de todos los paquetes de RoboticPark). `MeasureProcess.do_measure()` depende de la tabla de procesos real y en vivo (`psutil.process_iter()`) y de contadores de CPU/memoria de todo el sistema — no testeable unitariamente de forma significativa sin simular el propio `psutil`, no intentado aquí.
