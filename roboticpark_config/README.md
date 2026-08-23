# roboticpark_config

`ament_cmake` package with the launch and environment configuration shared by the rest of the lab's repositories. This is the original template that `uned_crazyflie_config`, `uned_kheperaiv_config` and `uned_swarm_config` (in `uned_multi_agent_ros_pkg`) each adapted their own `experience.launch.py`/`generic.launch.py` from.

## Structure

- **`launch/experience.launch.py`**: the single parametrized launch file — `ros2 launch roboticpark_config experience.launch.py config_file:=<experience>.yaml`. Reads `Operation` (Webots/Gazebo, world file, physical vs. simulated), `Architecture` (`centralized` launches a single coordination node by `pkg`/`executable`/`name`, matching `mars_supervisor_pkg`'s/`multi_agent_pkg`'s nodes; `distributed_ros2` launches nothing extra here), `Robots` (per-robot `type`/`name`, told apart by `dron*`/`khepera*` in the name), `CPU_Monitoring` (launches `measure_process_ros2_pkg`'s node), and `Interface` (RQT/RViz) sections from a `.yaml` in `resources/`.
- **`resources/`**: close to 90 experience `.yaml` files (see the root README's "Available experiences" table), the two robots' URDF (`crazyflie.urdf`, `kheperaiv.urdf`), and `models/RoboticLab/` — a Gazebo-native SDF model of the lab space (`model.sdf`/`model.config`).
- **`rviz/`, `rqt/`**: RViz configs and RQT perspectives, one set per experience family (`ControlFormation*`, `LagrangeMultipliers_*`, `IROS_AffineFormation_*`, ...).
- **`worlds/`**: Webots worlds for the same experience families, plus their shared meshes.

## Tests

None beyond the standard `ament_lint_auto`/`ament_lint_common` lint tests (not actually producing any test results — see `AUDIT.md` on the `doc` branch) — this package has no nodes of its own, only launch files and resources.
