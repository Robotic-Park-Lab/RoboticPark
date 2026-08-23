# Auditoría — RoboticPark (2026-08-23, revisada el mismo día)

**Segunda pasada el mismo día, tras una decisión tuya sobre la estructura de ramas.** Diste por buena la lectura del punto 2 de abajo (rama por defecto desincronizada) y decidiste: eliminar `humble-dev`, renombrar `main` a `humble-dev` (ahora rama única y por defecto), y en ella resolver directamente los puntos 3 (`.gitignore`), 4/5 (metadatos y dependencias) y 6 (`roboticpark_interfaces`). Esta revisión documenta ese trabajo y vuelve a comprobar el estado real con `colcon build`/`colcon test` aislados sobre la rama resultante. La primera pasada (más abajo) queda como registro histórico de lo encontrado antes de esta decisión.

`benchmark` sigue bloqueada y fuera de alcance — no se ha tocado en ninguna de las dos pasadas.

## 0 — Cirugía de ramas: qué se hizo y qué se recuperó

- [x] **`humble-dev` (antigua) eliminada, `main` renombrada a `humble-dev`** (local + remoto), y cambiada la rama por defecto de GitHub de `main` a `humble-dev` antes de borrar `main` (GitHub no permite borrar la rama por defecto). `origin/main` ya no existe.
- [x] **Recuperado el commit `"Update AffineFormation"`** (2026-08-22, autor `FranciscoJManasAlvarez`): antes de borrar la antigua `humble-dev`, se comprobó que ambas ramas divergían de un mismo punto (`4d67bed`) con exactamente un commit propio cada una — `main` solo tenía el commit `"main: convertir en índice del repo"` (el que causaba el punto 2 de la primera pasada), y la antigua `humble-dev` tenía este `"Update AffineFormation"`: ajustes reales de ganancias/lógica de zona de seguridad en `multi_agent_pkg/affine_formation_node.py`, más un fichero que **faltaba por completo**, `mars_supervisor_pkg/affine_formation_node.py` (608 líneas) — que el propio `setup.py` de `mars_supervisor_pkg` ya declaraba como entry point (`affine_formation_node = mars_supervisor_pkg.affine_formation_node:main`) **sin que el fichero existiera** en `main`. Se aplicó con `git cherry-pick` sobre la nueva `humble-dev`, limpio, sin conflictos, y verificado con `colcon build` aislado. Sin este paso se habría perdido trabajo real y quedado un entry point roto.
- [ ] **La rama `docs/bilingual-readmes-audit`** (READMEs bilingües del punto 1 de abajo) sigue existiendo pero **no se ha tocado**: se creó a partir de la punta de la antigua `humble-dev`, que ya no es la base de la nueva `humble-dev` — fusionarla directamente arrastraría de vuelta el estado antiguo. **Decisión pendiente tuya**: descartarla, o volver a aplicar/rehacer ese trabajo de documentación sobre la `humble-dev` actual.

## 1 — Documentación (de la primera pasada — sigue sin fusionar, ver punto 0)

- [x] **README raíz solo en español, sin secciones de Uso ni Publicaciones**: ahora en inglés con `README_es.md` en español, más las nuevas secciones "Usage"/"Uso" y "Related publications"/"Publicaciones relacionadas" (estructura y una tabla lista para que las vayas rellenando tú — ver ambos README). **Este trabajo vive solo en `docs/bilingual-readmes-audit`, no en `humble-dev`** — ver punto 0.
- [x] **`roboticpark_interfaces` mal descrito en el README raíz**: ya no aplica — el paquete se ha eliminado (punto 6).
- [x] **Ningún paquete tenía README propio**: los 6 originales (`mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_config`, `roboticpark_interfaces`, `scripts`) tienen `README.md`/`README_es.md` en `docs/bilingual-readmes-audit`, no en `humble-dev` (y ese branch documentaba un paquete, `roboticpark_interfaces`, que ya no existe).

## 2 — `main` (rama por defecto) no era solo un índice, y estaba desincronizada de `humble-dev` — RESUELTO (punto 0)

- [x] Resuelto eliminando la ambigüedad de raíz: ya no hay dos ramas de código, solo `humble-dev`. Ver punto 0 para el detalle de qué se recuperó al hacerlo.

## 3 — Sin `.gitignore`, y 12 ficheros `.pyc`/1 `.wbproj` versionados en git — RESUELTO

- [x] Añadido `.gitignore` en la raíz (`__pycache__/`, `*.pyc`, `*.egg-info/`, `build/`/`install/`/`log/` por si acaso, `*.wbproj`, editores/SO). Desindexados los 9 `.pyc` que quedaban en `mars_supervisor_pkg`/`measure_process_ros2_pkg`/`multi_agent_pkg` (los 3 de `roboticpark_interfaces` desaparecieron con el paquete, punto 6) y el `.wbproj` de `roboticpark_config/worlds/` — quedan en disco pero no en git.

## 4 — Metadatos sin rellenar y maintainer inconsistente — RESUELTO

- [x] `description`/`license` rellenados en `mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_config` (en `package.xml` y, donde aplica, `setup.py`). Licencia: `BSD-3-Clause`, coherente con el `LICENSE` de la raíz.
- [x] Maintainer unificado a `Francisco Jose Manas` / `fjmanas@dia.uned.es` en los 4 paquetes (antes: `kiko`/`fma527@ual.es` en 3 de ellos, y una variante acentuada distinta en el `setup.py` de `multi_agent_pkg`).
- [ ] **`roboticpark_config` sigue declarando** `rosidl_default_generators`/`rosidl_default_runtime`/`<member_of_group>rosidl_interface_packages</member_of_group>` sin tener ningún `msg/`/`srv/` real — no se ha tocado, es un cambio de build config más allá de "rellenar metadatos que faltan". **Decisión pendiente tuya** si quieres que se recorte.

## 5 — Dependencias no declaradas — RESUELTO para los paquetes que quedan

- [x] Añadidas las dependencias ROS reales que faltaban en `package.xml`: `rclpy`/`geometry_msgs`/`std_msgs`/`tf_transformations` en `mars_supervisor_pkg`; `rclpy`/`std_msgs` en `measure_process_ros2_pkg`; `rclpy`/`geometry_msgs`/`std_msgs`/`nav_msgs`/`tf2_ros`/`visualization_msgs`/`builtin_interfaces` en `multi_agent_pkg`; `urdf` en `roboticpark_config` (ya estaba en su `CMakeLists.txt` vía `find_package`, pero nunca declarado en `package.xml`).
- [ ] Las dependencias Python puras no resolubles por rosdep (`psutil` en `measure_process_ros2_pkg`, `scipy` en `multi_agent_pkg`) siguen sin declararse formalmente en `package.xml` — están documentadas en el README de cada paquete (de la pasada anterior en `docs/bilingual-readmes-audit`, ver punto 0/1) pero no en metadatos de build. No se ha tocado.

## 6 — `roboticpark_interfaces`: eliminado — RESUELTO

- [x] Confirmaste que no es útil ahora mismo. Se comprobó primero que ningún otro paquete lo referenciaba (`grep` en todo el repo) y se eliminó por completo (`git rm -r`): el nombre era engañoso (nada de `.msg`/`.srv`/`.action`; el único punto de entrada ROS 2 real era un stub de plantilla) y su código funcional real, una app PyQt5 de 909 líneas para diseñar `.yaml` de experiencia, nunca se instalaba (`setup.py` la dejaba fuera de `find_packages()`).

## 7 — Compila limpio; el lint sigue en rojo en los 3 paquetes Python (no se ha tocado, fuera del alcance de esta pasada)

Los 4 paquetes que quedan **compilan limpio** (`colcon build` aislado, sin errores; el commit recuperado del punto 0 no rompió nada). `colcon test` aislado, ya sobre la `humble-dev` actual:

| Paquete | `ament_flake8` | `ament_pep257` |
|---|---|---|
| `mars_supervisor_pkg` | ❌ falla (creció tras recuperar `affine_formation_node.py`, punto 0) | ❌ falla |
| `measure_process_ros2_pkg` | ❌ falla | ✅ pasa |
| `multi_agent_pkg` | ❌ falla | ❌ falla |
| `roboticpark_config` | — sigue sin generar ningún resultado de test, pese a declarar `ament_lint_auto`/`ament_lint_common` (mismo hallazgo que la primera pasada, no investigado) | |

(`ament_copyright` sigue sin resultado claro de pasa/falla en los 3 paquetes `ament_python` — "skipped" en `colcon test-result`, no investigado.)

## 8 — Código muerto/divergente en `affine_formation_node.py` (dos copias, dos estados distintos) — sin tocar

- [ ] **`mars_supervisor_pkg/affine_formation_node.py`**: existe de nuevo tras el cherry-pick del punto 0 (antes de esta pasada, faltaba en `main`/nueva `humble-dev` pese a que su `setup.py` lo declaraba como entry point — un bug real que este mismo cherry-pick corrige de rebote). `self.test` sigue fijado a `False` y nunca se cambia — la rama `if self.test:` de `update()` (`generate_arc`) es código muerto en la práctica. La llamada a su variante `new_distributed_formation_control`/`find_t_gains_2d` sigue comentada (envuelta en comillas triples) junto a la llamada real.
- [ ] **`multi_agent_pkg/affine_formation_node.py`**: versión más reciente y en desarrollo activo de la misma ley de control de pastoreo — máquina de estados distinta, interruptor `control_type` real entre dos leyes de control (ambas alcanzables), zona de seguridad, publicador `dist_sp`. Coincide con los ficheros `IROS_AffineFormation_*` de `roboticpark_config/resources/` y es la que tocó el commit `"Update AffineFormation"` recuperado en el punto 0.
- [ ] No está claro si `mars_supervisor_pkg/affine_formation_node.py` es la versión que se dejó de mantener a propósito cuando se creó la de `multi_agent_pkg`, o si sigue teniendo algún uso. **Decisión pendiente tuya** — no se ha tocado ninguna de las dos copias.

## Otros cabos sueltos de esta pasada

- [ ] Había cambios locales sin commitear en el working tree usado para esta auditoría (un `.ui` de `roboticpark_interfaces` ya eliminado, y ficheros generados de `roboticpark_config`/`multi_agent_pkg`) — se guardaron con `git stash` antes de tocar ramas, sin aplicar ni descartar. Están en el stash local de esa máquina si los necesitas; si no, se pueden descartar sin más.

## No verificado en este sandbox (de la primera pasada, sigue aplicando)

- El submódulo/paquete de mensajes `tello_msgs` y el resto de dependencias cruzadas de otros repositorios (`multi_agent_pkg` es en sí mismo una de esas dependencias, consumida por `uned_crazyflie_driver`/`uned_kheperaiv_webots`) no se han vuelto a verificar desde el otro lado en esta pasada — solo se ha auditado este repositorio en aislamiento.
- `install.sh` no se ha ejecutado de verdad en esta pasada (instalaría el resto del laboratorio completo).

---

## Primera pasada (2026-08-23, antes de la decisión sobre ramas) — registro histórico

Checklist fundamentada en inspección real de `humble-dev` y `main`: lectura completa de los 5 paquetes de código (`mars_supervisor_pkg`, `measure_process_ros2_pkg`, `multi_agent_pkg`, `roboticpark_config`, `roboticpark_interfaces`) más `scripts/`, sus `package.xml`/`setup.py`/`.gitignore`, y un `colcon build`/`colcon test` real y aislado de los 5 paquetes ROS. Primera auditoría de este repositorio dentro del pase repo-by-repo — es el repositorio flagship del laboratorio y el más grande, así que esta pasada fue más extensa que la de `uned_multi_agent_ros_pkg`/`uned_tello_ros_pkg`. En su momento fue solo documentación bilingüe + hallazgos, sin tocar código.

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
