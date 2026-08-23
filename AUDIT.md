# Auditoría — RoboticPark (2026-08-23)

Checklist fundamentada en inspección real de `humble-dev` y `main`: lectura completa de los 5 paquetes de código (`mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_config`, `roboticpark_interfaces`) más `scripts/`, sus `package.xml`/`setup.py`/`.gitignore`, y un `colcon build`/`colcon test` real y aislado de los 5 paquetes ROS. Primera auditoría de este repositorio dentro del pase repo-by-repo — es el repositorio flagship del laboratorio y el más grande, así que esta pasada es más extensa que la de `uned_multi_agent_ros_pkg`/`uned_tello_ros_pkg`. Es solo documentación bilingüe + hallazgos, sin tocar código; los puntos de abajo están **sin resolver**, a la espera de que decidas cuáles abordar.

`benchmark` está bloqueada y fuera de alcance — no se ha tocado ni leído nada de ella en esta pasada (solo se ha leído, desde `scripts/`, el pipeline de Matlab que genera su configuración — ver `scripts/README.md`).

## 1 — Documentación (resuelto en esta pasada)

- [x] **README raíz solo en español, sin secciones de Uso ni Publicaciones**: ahora en inglés con `README_es.md` en español, más las nuevas secciones "Usage"/"Uso" y "Related publications"/"Publicaciones relacionadas" (estructura y una tabla lista para que las vayas rellenando tú — ver ambos README).
- [x] **`roboticpark_interfaces` mal descrito en el README raíz**: decía "Mensajes, servicios y acciones ROS 2 comunes a varias plataformas" — falso, ver punto 6 más abajo. Corregido en ambos README raíz.
- [x] **Ningún paquete tenía README propio**: los 6 (`mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_config`, `roboticpark_interfaces`, `scripts`) tienen ahora `README.md`/`README_es.md` por primera vez.

## 2 — `main` (rama por defecto) no es solo un índice, y está desincronizada de `humble-dev`

- [ ] La rama `doc` (esta) documenta `main` como "índice del repo — sin código de workspace, solo enlaza a las demás ramas". **Eso ya no es cierto**: `main` se creó como una rama normal (no huérfana) a partir de la punta de `humble-dev`, así que su árbol todavía contiene los 6 paquetes completos, no solo un README. Desde entonces, `humble-dev` ha seguido avanzando (p. ej. el commit "Update AffineFormation" en `multi_agent_pkg`) y `main` **no se ha vuelto a sincronizar** — comparando ambas ramas: a `main` le falta `roboticpark_config/worlds/IROS_AffineFormation_N05_hybrid.wbt` y tiene versiones distintas (más antiguas) de varios `.yaml` de `LagrangeMultipliers_*`, además de una versión más simple de `install.sh` que la documentada en el propio README de `humble-dev`.
- [ ] Quien clone `RoboticPark` sin especificar rama (`git clone https://github.com/Robotic-Park-Lab/RoboticPark.git`, sin `-b`) se lleva por tanto una copia de código **desactualizada**, no un simple índice — contradice la intención original de la Fase 3 de la reestructuración general (documentada en `project-roboticpark-repo-restructuring`).
- [ ] **Decisión pendiente tuya**: o bien `main` se vuelve realmente huérfana (solo el índice, sin árbol de paquetes) y se sincroniza una vez, o bien se adopta un proceso para mantenerla al día con `humble-dev` (p. ej. fusionar `humble-dev` en `main` periódicamente, o hacer que `main` sea literalmente un merge/mirror). No se ha tocado `main` en esta pasada.

## 3 — Sin `.gitignore`, y 12 ficheros `.pyc` versionados en git

- [ ] Este repositorio **no tiene ningún `.gitignore`** (a diferencia de `uned_multi_agent_ros_pkg`, que sí tiene uno, aunque incompleto). Como consecuencia, hay **12 ficheros `__pycache__/*.pyc`** commiteados en `humble-dev`: 2 en `mars_supervisor_pkg`, 2 en `measure_process_ros2_pkg`, 5 en `multi_agent_pkg` (incluyendo `larange_multipliers.cpython-310.pyc` — el `.pyc` de un módulo con un typo en el nombre, de cuando el fichero se llamaba así antes de renombrarse a `lagrange_multipliers.py`; el compilado viejo se quedó atrás), y 3 en `roboticpark_interfaces/src/`.
- [ ] El propio directorio de trabajo usado para esta auditoría ya tiene bytecode `.pyc` sin trackear listo para colarse en el próximo `git add -A` — root cause directo de que sigan apareciendo.

## 4 — Metadatos sin rellenar y maintainer inconsistente

- [ ] **`TODO: Package description`** / **`TODO: License declaration`** sin rellenar en `mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_interfaces` (los 4 paquetes `ament_python`). `roboticpark_config` sí tiene descripción/licencia pendientes también (`TODO:` en ambos campos).
- [ ] **Maintainer inconsistente entre paquetes, y con el resto del laboratorio**: `mars_supervisor_pkg`/`multi_agent_pkg`/`roboticpark_interfaces` declaran `<maintainer email="...">kiko</maintainer>`; `measure_process_ros2_pkg` usa el email `fma527@ual.es` (distinto de `fjmanas@dia.uned.es`, usado en el resto del laboratorio) con nombre `kiko`; `roboticpark_config` usa `Francisco Jose Manas` (sin tildes, formato distinto) con `fjmanas@dia.uned.es`. Cuatro formas distintas de identificar a la misma persona en 5 paquetes.
- [ ] **`roboticpark_config`** además declara `rosidl_default_generators`/`rosidl_default_runtime` y `<member_of_group>rosidl_interface_packages</member_of_group>` sin tener ningún `msg/`/`srv/` — boilerplate de una plantilla de paquete de interfaces nunca recortado (ver su propio README).

## 5 — Dependencias no declaradas

- [ ] **`measure_process_ros2_pkg`**: `package.xml` no declara ni siquiera `rclpy`/`std_msgs` (solo `test_depend` de lint), pese a importarlos de verdad; tampoco declara `psutil` (que ni siquiera es resoluble por rosdep — instalación aparte, no documentada en ningún sitio hasta el nuevo README de este paquete).
- [ ] **`mars_supervisor_pkg`**, **`multi_agent_pkg`**: mismo patrón — `package.xml` sin `<depend>` real pese a usar `rclpy`/`std_msgs`/`geometry_msgs`/`visualization_msgs`/`tf2_ros`/`nav_msgs`/`scipy` (este último tampoco resoluble por rosdep).
- [ ] **`roboticpark_interfaces`**: no declara nada real tampoco (`matplotlib`/`numpy`/PyQt5/`sensor_msgs`/`geometry_msgs`/`visualization_msgs`), aunque esas dependencias solo las usa el código de `src/` que ni siquiera se empaqueta — ver el punto 6.

## 6 — `roboticpark_interfaces`: nombre engañoso y código real sin empaquetar

- [ ] El nombre sugiere definiciones ROS `.msg`/`.srv`/`.action`; no hay ninguna. El único punto de entrada ROS 2 real es un stub de plantilla (`roboticpark_interface.py`, `print('Hi from roboticpark_interfaces.')`).
- [ ] El código funcional real — `src/ExperimentalDesign_app.py` (909 líneas) + `src/ExperimentDesign.py` + `.ui`: una aplicación PyQt5 para diseñar visualmente ficheros `.yaml` de experiencia (genera el mismo esquema `Operation`/`Robots`/`Architecture`/... que lee `roboticpark_config/launch/experience.launch.py`) — **nunca se instala**: `setup.py` usa `packages=find_packages(exclude=['test'])`, que no incluye `src/`. Ningún `ros2 run` ni script de consola lo expone. 909 líneas de una herramienta con un propósito real, invisible para `colcon build`.
- [ ] **Decisión pendiente tuya**: renombrar el paquete a algo que refleje lo que hace, reestructurarlo para que `src/` se instale y tenga un entry point real, o ambas cosas — no decidido ni tocado aquí.

## 7 — Compila limpio, pero el lint está en rojo en 4 de 5 paquetes (verificado con `colcon build`/`colcon test` reales)

Los 5 paquetes ROS **compilan limpio** (`colcon build` real, sin errores). `colcon test` real y aislado muestra:

| Paquete | `ament_flake8` | `ament_pep257` |
|---|---|---|
| `mars_supervisor_pkg` | ❌ 100 avisos | ❌ falla |
| `measure_process_ros2_pkg` | ❌ 11 avisos | ✅ pasa |
| `multi_agent_pkg` | ❌ 153 avisos | ❌ falla |
| `roboticpark_interfaces` | ❌ 445 avisos (el peor de todo el repositorio) | ✅ pasa |
| `roboticpark_config` | — no genera ningún resultado de test, pese a declarar `ament_lint_auto`/`ament_lint_common` | |

(`ament_copyright` no produjo un resultado claro de pasa/falla en ninguno de los 4 paquetes `ament_python` — aparece como "skipped" en la salida de `colcon test-result`, no investigado más a fondo en esta pasada).

## 8 — Código muerto/divergente en `affine_formation_node.py` (dos copias, dos estados distintos)

- [ ] **`mars_supervisor_pkg/affine_formation_node.py`**: `self.test` está fijado a `False` y nunca se cambia en ningún otro sitio del fichero — la rama `if self.test:` de `update()` (`generate_arc`) es código muerto en la práctica. La llamada a su variante `new_distributed_formation_control`/`find_t_gains_2d` está además envuelta en una cadena de comillas triples justo al lado de la llamada real, es decir, comentada — doblemente inalcanzable.
- [ ] **`multi_agent_pkg/affine_formation_node.py`**: versión más reciente y en desarrollo activo de la misma ley de control de pastoreo — máquina de estados distinta, un interruptor `control_type` real entre dos leyes de control (ambas alcanzables), zona de seguridad, publicador `dist_sp`. Coincide con los ficheros `IROS_AffineFormation_*` de `roboticpark_config/resources/` y es lo único de estas dos copias que el commit "Update AffineFormation" (solo en `humble-dev`, ver punto 2) tocó.
- [ ] No está claro si `mars_supervisor_pkg/affine_formation_node.py` es la versión que se dejó de mantener a propósito cuando se creó la de `multi_agent_pkg`, o si sigue teniendo algún uso. **Decisión pendiente tuya** — no se ha tocado ninguna de las dos copias.

## No verificado en este sandbox

- El submódulo/paquete de mensajes `tello_msgs` y el resto de dependencias cruzadas de otros repositorios (`multi_agent_pkg` es en sí mismo una de esas dependencias, consumida por `uned_crazyflie_driver`/`uned_kheperaiv_webots`) no se han vuelto a verificar desde el otro lado en esta pasada — solo se ha auditado este repositorio en aislamiento.
- `install.sh` no se ha ejecutado de verdad en esta pasada (instalaría el resto del laboratorio completo) — su contenido solo se ha leído para el hallazgo del punto 2.
