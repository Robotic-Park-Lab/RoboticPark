# Lanzamientos disponibles — RoboticPark

English version: [LAUNCHES.md](LAUNCHES.md)

`roboticpark_config` tiene **un único fichero de lanzamiento**, `roboticpark_config/launch/experience.launch.py`,
y cada experiencia es un `.yaml` de `roboticpark_config/resources/`:

```bash
ros2 launch roboticpark_config experience.launch.py config_file:=<experiencia>.yaml
```

El `.yaml` decide qué se lanza: `Operation` (virtual/real/híbrido, Webots o Gazebo y mundo), `Architecture`
(`centralized` lanza el nodo indicado; `distributed` y `distributed_ros2` no lanzan nodo central), `Robots`,
`CPU_Monitoring` (`measure_process_ros2_pkg`), `Interface` (RViz/RQT), `Data_Logging` (rosbag), `Supervisor`
(`mars_supervisor_pkg` con un `*_topics.yaml` de órdenes temporizadas) y `Other` (nodos adicionales). Los ficheros
`*_topics.yaml` no son experiencias: son los guiones del supervisor, y `OpenLoop_Crazyflie_*.yaml`
son parámetros del nodo `open_loop_signal`.

Para crear una experiencia nueva sin editar YAML a mano: [configurador web](https://robotic-park-lab.github.io/configurador/)
(genera el `.yaml`, el `_topics.yaml`, el mundo `.wbt` y el `.rviz`).

**Tabla generada a partir del contenido de cada `.yaml`** (modo, robots, ley de la tarea, nodos). "Tarea" es la de los
robots con `task.enable` u `Onboard`; si no hay ninguna, se indica el nodo central si existe.

## Formación triangulada con líderes (malla)

12 Crazyflie en una malla triangulada guiada por dos líderes (`multi_agent_pkg/triangulated_formation_node`). Variantes: estática, reconfiguración, incertidumbre, barrido de ganancia baja y vuelo real.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `AdaptiveFormation_N12_gainsweep_low.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | formación por posiciones relativas; `affine_p`; continua; líder-seguidor | nodo central `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); monitor de CPU |
| `AdaptiveFormation_N12_real.yaml` | real | 12 Crazyflie | formación por posiciones relativas; `affine_p`; continua; líder-seguidor | nodo central `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); monitor de CPU |
| `AdaptiveFormation_N12_reconfig.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | formación por posiciones relativas; `affine_p`; continua; líder-seguidor | nodo central `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_reconfig.yaml`); monitor de CPU |
| `AdaptiveFormation_N12_static.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | formación por posiciones relativas; `affine_p`; continua; líder-seguidor | nodo central `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); monitor de CPU |
| `AdaptiveFormation_N12_uncertainty.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | formación por posiciones relativas; `affine_p`; continua; líder-seguidor | nodo central `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); monitor de CPU |

## Formación por distancias con referencia al origen

Formación de N Crazyflie (7–20) por distancias a vecinos más la distancia a un punto de referencia, en virtual, real e híbrido; incluye experimentos de reconfiguración.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `ControlFormation07.yaml` | virtual · Webots · `default_world.wbt` | 7 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation07_R10.yaml` | virtual · Webots · `ControlFormation07_R10.wbt` | 7 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation07_reconfiguration.yaml` | real | 7 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | supervisor (`reconfiguration_topics.yaml`); nodos extra: `swarm_reconfiguration`; monitor de CPU; grabación rosbag |
| `ControlFormation07_reconfiguration_Webots.yaml` | virtual · Webots · `ControlFormation07.wbt` | 7 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | supervisor (`reconfiguration_topics.yaml`); nodos extra: `swarm_reconfiguration`; monitor de CPU |
| `ControlFormation08.yaml` | real | 8 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation08_hybrid.yaml` | híbrido · Webots · `ControlFormation08_hybrid.wbt` | 8 Crazyflie (1 virtual, 7 real) | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation08_virtual.yaml` | virtual · Webots · `ControlFormation08.wbt` | 8 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation12_hybrid.yaml` | híbrido · Webots · `ControlFormation12_Hybrid.wbt` | 12 Crazyflie (6 virtual, 6 real) | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation12_virtual.yaml` | virtual · Webots · `ControlFormation12.wbt` | 12 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation15.yaml` | virtual · Webots · `ControlFormation15.wbt` | 15 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `ControlFormation20.yaml` | virtual · Webots · `ControlFormation20.wbt` | 20 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU; grabación rosbag |

## Benchmark de escalabilidad

50 Crazyflie virtuales para medir carga de CPU y escalado.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `CoverageBenchmark_config.yaml` | virtual · Webots · `CoverageBenchmark.wbt` | 50 Crazyflie | formación por distancias sobre esfera; `gradient_p`; continua | supervisor (`CoverageBenchmark_topics.yaml`); monitor de CPU |

## Demos con Crazyflie

Demos de uno a cinco drones: trayectorias en lazo abierto (círculo, polígono), formaciones pequeñas, gimbal (identificación PRBS y ajuste por relé) e identificación.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `Crazyflie_DemoCircle.yaml` | real | 1 Crazyflie | sin formación | nodos extra: `open_loop_signal`; monitor de CPU |
| `Crazyflie_DemoCircle_webots.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | sin formación | nodos extra: `open_loop_signal`; monitor de CPU |
| `Crazyflie_DemoFormation03.yaml` | real | 3 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation03_Offboard.yaml` | real | 3 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation03r.yaml` | real | 3 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation03r_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation04.yaml` | real | 4 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation04r.yaml` | real | 4 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation04r_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation05.yaml` | real | 5 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation05_Offboard.yaml` | real | 5 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation05_Webots.yaml` | virtual · Webots · `ControlFormation05.wbt` | 5 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoFormation05r.yaml` | real | 5 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | supervisor (`Crazyflie_DemoFormation05r_topics.yaml`); monitor de CPU |
| `Crazyflie_DemoPolygon.yaml` | real | 1 Crazyflie | sin formación | nodos extra: `open_loop_signal`; monitor de CPU |
| `Crazyflie_DemoPolygon_webots.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | sin formación | nodos extra: `open_loop_signal`; monitor de CPU |
| `Crazyflie_Gimbal_demo.yaml` | real | 1 Crazyflie | sin formación | supervisor (`gimbal_PRBS_pitch_rate_topics.yaml`) |
| `Crazyflie_Gimbal_rele.yaml` | real | 1 Crazyflie | sin formación | supervisor (`gimbal_rele_range_pitch_topics.yaml`) |
| `Crazyflie_Identification.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | sin formación | supervisor (`Crazyflie_Identification_topics.yaml`); monitor de CPU; grabación rosbag |
| `Crazyflie_Individual.yaml` | real | 1 Crazyflie | sin formación | monitor de CPU |

## Pastoreo con formación afín

N Crazyflie guían a un Khepera IV con una formación afín (`multi_agent_pkg/affine_formation_node`), variantes de escala.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `Herding_N07_scale.yaml` | virtual · Webots · `Herding_N07.wbt` | 7 Crazyflie, 1 Khepera IV | formación por posiciones relativas; `affine_p`/`reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |
| `Herding_N09_scale.yaml` | virtual · Webots · `Herding_N09.wbt` | 9 Crazyflie, 1 Khepera IV | formación por posiciones relativas; `affine_p`/`reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |
| `Herding_N13_scale.yaml` | virtual · Webots · `Herding_N13.wbt` | 13 Crazyflie, 1 Khepera IV | formación por posiciones relativas; `affine_p`/`reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |

## Formación afín (IROS)

Configuraciones de la formación afín con Khepera IV: 5 y 13 drones, virtual e híbrida.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `IROS_AffineFormation_config.yaml` | virtual · Webots · `IROS_AffineFormation_N13.wbt` | 13 Crazyflie, 1 Khepera IV | formación por posiciones relativas; `affine_p`/`reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |
| `IROS_AffineFormation_config_N05.yaml` | virtual · Webots · `IROS_AffineFormation_N05.wbt` | 5 Crazyflie, 1 Khepera IV | formación por posiciones relativas; `affine_p`/`reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |
| `IROS_AffineFormation_config_N05_real.yaml` | virtual · Webots · `IROS_AffineFormation_N05_hybrid.wbt` | 5 Crazyflie, 1 Khepera IV (5 real, 1 virtual) | formación por posiciones relativas; `reactive`; continua | nodo central `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); monitor de CPU |

## Khepera IV

Teleoperación y ensayo de relé con un Khepera IV.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `KheperaIV_Teleop_webots.yaml` | virtual · Webots · `KheperaIV_Individual.wbt` | 1 Khepera IV | sin formación | monitor de CPU |
| `Khepera_rele.yaml` | virtual · Webots · `ControlFormation05.wbt` | 1 Khepera IV | sin formación | supervisor (`LagrangeMultiplier_topics.yaml`); monitor de CPU; grabación rosbag; ⚠️ falta en el repo: `LagrangeMultiplier_topics.yaml` |

## Formaciones sobre superficies (multiplicadores de Lagrange, ECC 2026)

Formación por distancias restringida a una esfera o un cono, con las leyes `gradient`, `ML1` (RF+C) y `ML2` (PRF), de 5 a 20 drones.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `LagrangeMultipliers_N05_config.yaml` | virtual · Webots · `LagrangeMultipliers_N05.wbt` | 5 Crazyflie | formación por distancias sobre esfera; `ML1_v`; continua | supervisor (`LagrangeMultipliers_N05_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_N08_config.yaml` | virtual · Webots · `LagrangeMultipliers_N08.wbt` | 8 Crazyflie | formación por distancias sobre esfera; `gradient_v`; continua | supervisor (`LagrangeMultipliers_N08_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_N12.wbt` | 12 Crazyflie | formación por distancias sobre esfera; `gradient_v`; continua | supervisor (`LagrangeMultipliers_N12_topics.yaml`); monitor de CPU; grabación rosbag |
| `LagrangeMultipliers_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_N15.wbt` | 15 Crazyflie | formación por distancias sobre esfera; `ML2_v`; continua | supervisor (`LagrangeMultipliers_N15_topics.yaml`); monitor de CPU; grabación rosbag |
| `LagrangeMultipliers_N20_config.yaml` | virtual · Webots · `LagrangeMultipliers_N20.wbt` | 20 Crazyflie | formación por distancias sobre esfera; `gradient_v`; continua | supervisor (`LagrangeMultipliers_N20_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_cone_N06_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N06.wbt` | 6 Crazyflie | formación por distancias sobre cono; `gradient_v`; continua | supervisor (`LagrangeMultipliers_cone_N06_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_cone_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N12.wbt` | 12 Crazyflie | formación por distancias sobre cono; `ML2_v`; continua | supervisor (`LagrangeMultipliers_cone_N12_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_cone_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N15.wbt` | 15 Crazyflie | formación por distancias sobre cono; `ML2_v`; continua | supervisor (`LagrangeMultipliers_cone_N15_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_sphere_N05_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N05.wbt` | 5 Crazyflie | formación por distancias sobre esfera; `gradient_v`; continua | supervisor (`LagrangeMultipliers_sphere_N05_topics.yaml`); monitor de CPU |
| `LagrangeMultipliers_sphere_N08_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N08.wbt` | 8 Crazyflie | formación por distancias sobre esfera; `ML1_v`; continua | supervisor (`LagrangeMultipliers_sphere_N08_topics.yaml`); monitor de CPU; grabación rosbag |
| `LagrangeMultipliers_sphere_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N12.wbt` | 12 Crazyflie | formación por distancias sobre esfera; `ML1_v`; continua | supervisor (`LagrangeMultipliers_sphere_N12_topics.yaml`); monitor de CPU; grabación rosbag |
| `LagrangeMultipliers_sphere_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N15.wbt` | 15 Crazyflie | formación por distancias sobre esfera; `gradient_v`; continua | supervisor (`LagrangeMultipliers_sphere_N15_topics.yaml`); monitor de CPU; grabación rosbag |
| `LagrangeMultipliers_sphere_N20_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N20.wbt` | 20 Crazyflie | formación por distancias sobre esfera; `ML2_v`; continua | supervisor (`LagrangeMultipliers_sphere_N20_topics.yaml`); monitor de CPU |

## Morphing

Flota mixta (20 Crazyflie + 10–13 Khepera IV) que cambia entre esfera y elipsoide.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `Morphing_ellipsoid.yaml` | virtual · Webots · `Morphing.wbt` | 13 Khepera IV, 20 Crazyflie | formación por distancias sobre elipsoide; `gradient`/`gradient_v`; continua | supervisor (`Morphing_ellipsoid_topics.yaml`); monitor de CPU |
| `Morphing_sphere.yaml` | virtual · Webots · `Morphing.wbt` | 20 Crazyflie, 10 Khepera IV | formación por distancias sobre esfera; `ML2_v`; continua | supervisor (`Morphing_sphere_topics.yaml`); monitor de CPU |

## Formación esférica

Formación esférica real (8) e híbrida (12).

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `SphereFormation08.yaml` | real | 8 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`SphereFormation_topics.yaml`); monitor de CPU |
| `SphereFormation12.yaml` | híbrido · Webots · `ControlFormation12.wbt` | 12 Crazyflie (8 real, 4 virtual) | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`SphereFormation_topics.yaml`); monitor de CPU |

## Plantillas y pruebas

Configuración por defecto, benchmark y pruebas.

| `config_file` | Modo · herramienta · mundo | Robots | Tarea | Otros nodos y avisos |
|---|---|---|---|---|
| `default_config.yaml` | virtual · Webots · `default_world.wbt` | 15 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | supervisor (`default_topics.yaml`); monitor de CPU; grabación rosbag |
| `demo_benchmark.yaml` | virtual · Webots | 15 Crazyflie | sin formación | comunicación por eventos; supervisor (`test_file`); monitor de CPU; grabación rosbag; ⚠️ falta en el repo: `test_file` |
| `demo_benchmark_webots.yaml` | virtual · Webots · `ControlFormation15.wbt` | 15 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; supervisor (`default_topics.yaml`); monitor de CPU |
| `test.yaml` | real | 3 Crazyflie | formación por distancias sobre esfera; `gradient`; continua | comunicación por eventos; monitor de CPU |

## Notas de compatibilidad (2026-11-22)

- Todos los mundos `.wbt` incluyen el LED `led_ring` en cada Crazyflie: `uned_crazyflie_driver/webots_driver.py`
  lo usa al arrancar y sin él el driver no se inicializa.
- Las formaciones por distancias en Webots (`ControlFormation*`, `LagrangeMultipliers_*`, `SphereFormation*`,
  `Morphing_*`, …) necesitan `uned_crazyflie_ros_pkg` en `humble-dev` desde `a49b93e` (migración de
  formaciones, ver su `AUDIT.md` en la rama `doc`). Con versiones anteriores el driver falla en
  `load_formation_params()`.
- Las relaciones con el formato antiguo `origin_R` (sin centro) se interpretan con centro en (0, 0, 0).
- No se han vuelto a ejecutar una por una todas las experiencias de esta tabla; sí las seis plantillas del
  configurador web (cúpula ML2, anillo por eventos, flota híbrida, pastoreo, escudo triangulado y un dron).
