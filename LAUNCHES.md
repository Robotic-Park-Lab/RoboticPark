# Available launches — RoboticPark

Versión en español: [LAUNCHES_es.md](LAUNCHES_es.md)

`roboticpark_config` has **a single launch file**, `roboticpark_config/launch/experience.launch.py`,
and every experience is a `.yaml` in `roboticpark_config/resources/`:

```bash
ros2 launch roboticpark_config experience.launch.py config_file:=<experience>.yaml
```

The `.yaml` decides what is launched: `Operation` (virtual/real/hybrid, Webots or Gazebo and world), `Architecture`
(`centralized` launches the given node; `distributed` and `distributed_ros2` launch no central node), `Robots`,
`CPU_Monitoring` (`measure_process_ros2_pkg`), `Interface` (RViz/RQT), `Data_Logging` (rosbag), `Supervisor`
(`mars_supervisor_pkg` with a `*_topics.yaml` of timed commands) and `Other` (extra nodes). The `*_topics.yaml` files
are not experiences: they are the supervisor scripts, and `OpenLoop_Crazyflie_*.yaml`
are parameters for the `open_loop_signal` node.

To create a new experience without hand-editing YAML: [web experiment builder](https://robotic-park-lab.github.io/configurador/)
(it generates the `.yaml`, the `_topics.yaml`, the `.wbt` world and the `.rviz`).

**Table generated from the content of each `.yaml`** (mode, robots, task law, nodes). "Task" is the one of the robots
with `task.enable` or `Onboard`; when there is none, the central node is shown if any.

## Leader-based triangulated formation (mesh)

12 Crazyflie in a triangulated mesh steered by two leaders (`multi_agent_pkg/triangulated_formation_node`). Variants: static, reconfiguration, uncertainty, low-gain sweep and real flight.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `AdaptiveFormation_N12_gainsweep_low.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | relative-position formation; `affine_p`; continuous; leader-follower | central node `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); CPU monitor |
| `AdaptiveFormation_N12_real.yaml` | real | 12 Crazyflie | relative-position formation; `affine_p`; continuous; leader-follower | central node `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); CPU monitor |
| `AdaptiveFormation_N12_reconfig.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | relative-position formation; `affine_p`; continuous; leader-follower | central node `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_reconfig.yaml`); CPU monitor |
| `AdaptiveFormation_N12_static.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | relative-position formation; `affine_p`; continuous; leader-follower | central node `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); CPU monitor |
| `AdaptiveFormation_N12_uncertainty.yaml` | virtual · Webots · `AdaptiveFormation_N12.wbt` | 12 Crazyflie | relative-position formation; `affine_p`; continuous; leader-follower | central node `triangulated_formation_node`; supervisor (`AdaptiveFormation_topics_static.yaml`); CPU monitor |

## Distance-based formation with an origin reference

Formation of N Crazyflie (7–20) from neighbour distances plus the distance to a reference point, virtual, real and hybrid; includes reconfiguration experiments.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `ControlFormation07.yaml` | virtual · Webots · `default_world.wbt` | 7 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation07_R10.yaml` | virtual · Webots · `ControlFormation07_R10.wbt` | 7 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation07_reconfiguration.yaml` | real | 7 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | supervisor (`reconfiguration_topics.yaml`); extra nodes: `swarm_reconfiguration`; CPU monitor; rosbag recording |
| `ControlFormation07_reconfiguration_Webots.yaml` | virtual · Webots · `ControlFormation07.wbt` | 7 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | supervisor (`reconfiguration_topics.yaml`); extra nodes: `swarm_reconfiguration`; CPU monitor |
| `ControlFormation08.yaml` | real | 8 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation08_hybrid.yaml` | hybrid · Webots · `ControlFormation08_hybrid.wbt` | 8 Crazyflie (1 virtual, 7 real) | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation08_virtual.yaml` | virtual · Webots · `ControlFormation08.wbt` | 8 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation12_hybrid.yaml` | hybrid · Webots · `ControlFormation12_Hybrid.wbt` | 12 Crazyflie (6 virtual, 6 real) | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation12_virtual.yaml` | virtual · Webots · `ControlFormation12.wbt` | 12 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation15.yaml` | virtual · Webots · `ControlFormation15.wbt` | 15 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `ControlFormation20.yaml` | virtual · Webots · `ControlFormation20.wbt` | 20 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor; rosbag recording |

## Scalability benchmark

50 virtual Crazyflie to measure CPU load and scaling.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `CoverageBenchmark_config.yaml` | virtual · Webots · `CoverageBenchmark.wbt` | 50 Crazyflie | distance-based formation on a sphere; `gradient_p`; continuous | supervisor (`CoverageBenchmark_topics.yaml`); CPU monitor |

## Crazyflie demos

One- to five-drone demos: open-loop trajectories (circle, polygon), small formations, gimbal (PRBS identification and relay tuning) and identification.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `Crazyflie_DemoCircle.yaml` | real | 1 Crazyflie | no formation | extra nodes: `open_loop_signal`; CPU monitor |
| `Crazyflie_DemoCircle_webots.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | no formation | extra nodes: `open_loop_signal`; CPU monitor |
| `Crazyflie_DemoFormation03.yaml` | real | 3 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation03_Offboard.yaml` | real | 3 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation03r.yaml` | real | 3 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation03r_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation04.yaml` | real | 4 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation04r.yaml` | real | 4 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation04r_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation05.yaml` | real | 5 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation05_Offboard.yaml` | real | 5 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation05_Webots.yaml` | virtual · Webots · `ControlFormation05.wbt` | 5 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`Crazyflie_DemoFormation05_topics.yaml`); CPU monitor |
| `Crazyflie_DemoFormation05r.yaml` | real | 5 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | supervisor (`Crazyflie_DemoFormation05r_topics.yaml`); CPU monitor |
| `Crazyflie_DemoPolygon.yaml` | real | 1 Crazyflie | no formation | extra nodes: `open_loop_signal`; CPU monitor |
| `Crazyflie_DemoPolygon_webots.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | no formation | extra nodes: `open_loop_signal`; CPU monitor |
| `Crazyflie_Gimbal_demo.yaml` | real | 1 Crazyflie | no formation | supervisor (`gimbal_PRBS_pitch_rate_topics.yaml`) |
| `Crazyflie_Gimbal_rele.yaml` | real | 1 Crazyflie | no formation | supervisor (`gimbal_rele_range_pitch_topics.yaml`) |
| `Crazyflie_Identification.yaml` | virtual · Webots · `Crazyflie_Individual.wbt` | 1 Crazyflie | no formation | supervisor (`Crazyflie_Identification_topics.yaml`); CPU monitor; rosbag recording |
| `Crazyflie_Individual.yaml` | real | 1 Crazyflie | no formation | CPU monitor |

## Herding with an affine formation

N Crazyflie herd a Khepera IV with an affine formation (`multi_agent_pkg/affine_formation_node`), scale variants.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `Herding_N07_scale.yaml` | virtual · Webots · `Herding_N07.wbt` | 7 Crazyflie, 1 Khepera IV | relative-position formation; `affine_p`/`reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |
| `Herding_N09_scale.yaml` | virtual · Webots · `Herding_N09.wbt` | 9 Crazyflie, 1 Khepera IV | relative-position formation; `affine_p`/`reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |
| `Herding_N13_scale.yaml` | virtual · Webots · `Herding_N13.wbt` | 13 Crazyflie, 1 Khepera IV | relative-position formation; `affine_p`/`reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |

## Affine formation (IROS)

Affine-formation configurations with a Khepera IV: 5 and 13 drones, virtual and hybrid.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `IROS_AffineFormation_config.yaml` | virtual · Webots · `IROS_AffineFormation_N13.wbt` | 13 Crazyflie, 1 Khepera IV | relative-position formation; `affine_p`/`reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |
| `IROS_AffineFormation_config_N05.yaml` | virtual · Webots · `IROS_AffineFormation_N05.wbt` | 5 Crazyflie, 1 Khepera IV | relative-position formation; `affine_p`/`reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |
| `IROS_AffineFormation_config_N05_real.yaml` | virtual · Webots · `IROS_AffineFormation_N05_hybrid.wbt` | 5 Crazyflie, 1 Khepera IV (5 real, 1 virtual) | relative-position formation; `reactive`; continuous | central node `affine_formation_node`; supervisor (`IROS_AffineFormation_topics.yaml`); CPU monitor |

## Khepera IV

Teleoperation and relay test with one Khepera IV.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `KheperaIV_Teleop_webots.yaml` | virtual · Webots · `KheperaIV_Individual.wbt` | 1 Khepera IV | no formation | CPU monitor |
| `Khepera_rele.yaml` | virtual · Webots · `ControlFormation05.wbt` | 1 Khepera IV | no formation | supervisor (`LagrangeMultiplier_topics.yaml`); CPU monitor; rosbag recording; ⚠️ missing from the repo: `LagrangeMultiplier_topics.yaml` |

## Formations on surfaces (Lagrange multipliers, ECC 2026)

Distance-based formation constrained to a sphere or a cone, with the `gradient`, `ML1` (RF+C) and `ML2` (PRF) laws, 5 to 20 drones.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `LagrangeMultipliers_N05_config.yaml` | virtual · Webots · `LagrangeMultipliers_N05.wbt` | 5 Crazyflie | distance-based formation on a sphere; `ML1_v`; continuous | supervisor (`LagrangeMultipliers_N05_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_N08_config.yaml` | virtual · Webots · `LagrangeMultipliers_N08.wbt` | 8 Crazyflie | distance-based formation on a sphere; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_N08_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_N12.wbt` | 12 Crazyflie | distance-based formation on a sphere; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_N12_topics.yaml`); CPU monitor; rosbag recording |
| `LagrangeMultipliers_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_N15.wbt` | 15 Crazyflie | distance-based formation on a sphere; `ML2_v`; continuous | supervisor (`LagrangeMultipliers_N15_topics.yaml`); CPU monitor; rosbag recording |
| `LagrangeMultipliers_N20_config.yaml` | virtual · Webots · `LagrangeMultipliers_N20.wbt` | 20 Crazyflie | distance-based formation on a sphere; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_N20_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_cone_N06_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N06.wbt` | 6 Crazyflie | distance-based formation on a cone; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_cone_N06_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_cone_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N12.wbt` | 12 Crazyflie | distance-based formation on a cone; `ML2_v`; continuous | supervisor (`LagrangeMultipliers_cone_N12_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_cone_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_cone_N15.wbt` | 15 Crazyflie | distance-based formation on a cone; `ML2_v`; continuous | supervisor (`LagrangeMultipliers_cone_N15_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_sphere_N05_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N05.wbt` | 5 Crazyflie | distance-based formation on a sphere; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_sphere_N05_topics.yaml`); CPU monitor |
| `LagrangeMultipliers_sphere_N08_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N08.wbt` | 8 Crazyflie | distance-based formation on a sphere; `ML1_v`; continuous | supervisor (`LagrangeMultipliers_sphere_N08_topics.yaml`); CPU monitor; rosbag recording |
| `LagrangeMultipliers_sphere_N12_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N12.wbt` | 12 Crazyflie | distance-based formation on a sphere; `ML1_v`; continuous | supervisor (`LagrangeMultipliers_sphere_N12_topics.yaml`); CPU monitor; rosbag recording |
| `LagrangeMultipliers_sphere_N15_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N15.wbt` | 15 Crazyflie | distance-based formation on a sphere; `gradient_v`; continuous | supervisor (`LagrangeMultipliers_sphere_N15_topics.yaml`); CPU monitor; rosbag recording |
| `LagrangeMultipliers_sphere_N20_config.yaml` | virtual · Webots · `LagrangeMultipliers_sphere_N20.wbt` | 20 Crazyflie | distance-based formation on a sphere; `ML2_v`; continuous | supervisor (`LagrangeMultipliers_sphere_N20_topics.yaml`); CPU monitor |

## Morphing

Mixed fleet (20 Crazyflie + 10–13 Khepera IV) switching between sphere and ellipsoid.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `Morphing_ellipsoid.yaml` | virtual · Webots · `Morphing.wbt` | 13 Khepera IV, 20 Crazyflie | distance-based formation on a ellipsoid; `gradient`/`gradient_v`; continuous | supervisor (`Morphing_ellipsoid_topics.yaml`); CPU monitor |
| `Morphing_sphere.yaml` | virtual · Webots · `Morphing.wbt` | 20 Crazyflie, 10 Khepera IV | distance-based formation on a sphere; `ML2_v`; continuous | supervisor (`Morphing_sphere_topics.yaml`); CPU monitor |

## Sphere formation

Real (8) and hybrid (12) sphere formation.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `SphereFormation08.yaml` | real | 8 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`SphereFormation_topics.yaml`); CPU monitor |
| `SphereFormation12.yaml` | hybrid · Webots · `ControlFormation12.wbt` | 12 Crazyflie (8 real, 4 virtual) | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`SphereFormation_topics.yaml`); CPU monitor |

## Templates and tests

Default configuration, benchmark and tests.

| `config_file` | Mode · tool · world | Robots | Task | Other nodes and warnings |
|---|---|---|---|---|
| `default_config.yaml` | virtual · Webots · `default_world.wbt` | 15 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | supervisor (`default_topics.yaml`); CPU monitor; rosbag recording |
| `demo_benchmark.yaml` | virtual · Webots | 15 Crazyflie | no formation | event-based communication; supervisor (`test_file`); CPU monitor; rosbag recording; ⚠️ missing from the repo: `test_file` |
| `demo_benchmark_webots.yaml` | virtual · Webots · `ControlFormation15.wbt` | 15 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; supervisor (`default_topics.yaml`); CPU monitor |
| `test.yaml` | real | 3 Crazyflie | distance-based formation on a sphere; `gradient`; continuous | event-based communication; CPU monitor |

## Compatibility notes (2026-11-22)

- Every `.wbt` world includes the `led_ring` LED on each Crazyflie: `uned_crazyflie_driver/webots_driver.py` uses
  it at start-up and the driver does not initialise without it.
- Distance-based formations in Webots (`ControlFormation*`, `LagrangeMultipliers_*`, `SphereFormation*`,
  `Morphing_*`, …) need `uned_crazyflie_ros_pkg` with the formation migration (branch `fix/formaciones-webots`,
  see its `AUDIT.md` on the `doc` branch). With the earlier `humble-dev` the driver fails in `load_formation_params()`.
- Relationships in the old `origin_R` format (no centre) are read with the centre at (0, 0, 0).
- Not every experience in this table has been re-run one by one; the six web-builder templates have (ML2 dome,
  event-based ring, hybrid fleet, herding, triangulated shield and single drone).
