# Ideas y trabajo futuro — RoboticPark

Documento vivo para anotar ideas, mejoras y líneas de trabajo futuras que no son bugs ni pendientes de la auditoría actual (eso vive en `AUDIT.md`, en esta misma rama). Añade entradas libremente, con fecha, según vayan surgiendo.

## Ideas abiertas

- **(2026-08-23) Decidir el papel real de `main`.** Ahora mismo es una rama por defecto que se suponía índice-only pero que en realidad carga todo el árbol de paquetes y ha quedado desincronizada de `humble-dev` (ver `AUDIT.md`, punto 2). Dos caminos razonables: convertirla de verdad en una rama huérfana ligera, o adoptar un proceso de sincronización periódica (merge automático o manual desde `humble-dev`).
- **(2026-08-23) `roboticpark_interfaces`: renombrar y/o reestructurar.** El nombre no coincide con el contenido (es una app PyQt de diseño de experimentos, no interfaces ROS), y su código real (`src/ExperimentalDesign_app.py`) no se instala con `colcon build`. Ver `AUDIT.md`, punto 6, para el detalle completo.
- **(2026-08-23) Documentar la tabla de experiencias de `roboticpark_config`.** El README raíz y el de `roboticpark_config` ya tienen la tabla/estructura lista — falta ir rellenándola fichero a fichero según se vayan revisitando las casi 90 experiencias de `resources/`.
- **(2026-08-23) Enlazar el pipeline de Matlab del benchmark con la tabla de publicaciones.** `scripts/RP_benchmark.m` genera literalmente la familia `ControlFormation*` de `roboticpark_config/resources/` para el capítulo de libro — buen primer caso concreto para la tabla de "Publicaciones relacionadas" del README raíz.
- **(2026-08-23) Decidir el destino de `mars_supervisor_pkg/affine_formation_node.py`.** Tiene una rama de código muerta (`generate_arc`, nunca alcanzada) y una llamada comentada (`new_distributed_formation_control`), mientras `multi_agent_pkg/affine_formation_node.py` sigue evolucionando activamente para el mismo problema. Si la primera ya no se usa, candidata a retirar o marcar explícitamente como legacy.
- **(2026-08-23) Añadir un `.gitignore` real.** El repositorio no tiene ninguno — root cause de los 12 `.pyc` commiteados encontrados en la auditoría. Un `.gitignore` estándar de `ament_python` (`__pycache__/`, `*.pyc`, `build/`, `install/`, `log/`) evitaría que se repita.

## Explorado y descartado

_(sin entradas todavía)_
