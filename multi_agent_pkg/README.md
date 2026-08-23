# multi_agent_pkg

`ament_python` package with formation-control geometry math shared across the lab's multi-agent experiments, plus the lab's active herding/affine-formation control node. Its `lagrange_multipliers` module also acts as a **library dependency**: `uned_crazyflie_driver` and `uned_kheperaiv_webots` import `multi_agent_pkg.lagrange_multipliers` directly for its geometry classes, rather than running any node from this package.

## Structure

- **`lagrange_multipliers.py`**: `Line`, `Curve`, `Sphere`, `Cone`, `Ellipsoid` — geometry primitives (each with `distance()`/`value()`/`projection()`, used by gradient-based formation controllers to keep a swarm's shape constrained to a target surface) via Lagrange-multiplier optimization (`scipy.optimize.minimize`, see `Cone.cone_constraint()`). This is what the Crazyflie/Khepera IV Webots drivers' "sphere/cone/ellipsoid formation geometries" (documented in their own READMEs) actually resolve to. Also defines `LagrangeMultipliers(Node)` (console script `lagrange_multipliers`), a thin example/demo node (`example_callback`) — not what the other repos actually use; they import the geometry classes as a plain Python module.
- **`affine_formation_node.py`** (console script `affine_formation_node`, node name `affine_herding_node`): `AffineHerdingNode`, the lab's active control law for herding a group of "herder" robots (Crazyflies, matched by `dron` in the robot name from a `Robots` config section) around a moving "sheep" agent (hardcoded to `khepera01`'s `local_pose`/`target_pose` topics) — state machine with `formation`/`check_zone`/`check_order` stages, a `control_type` toggle between two live control laws, a safety-zone distance switch, and a `dist_sp` publisher. Matches the `IROS_AffineFormation_*` experience files in `roboticpark_config/resources/`. A diverging, unmaintained copy of an earlier version of this node used to also live in `mars_supervisor_pkg` — removed on 2026-08-23 in favor of this one, which is the more recently developed and actively used.
- **`basic_node.py`** (console script `basic_node`): a `ros2 pkg create` template stub (`print('Hi from multi_agent_pkg.')`), never filled in.

## Dependencies

`rclpy`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `nav_msgs`, `builtin_interfaces` — declared in `package.xml`. Also `scipy` (for `Cone.cone_constraint()`'s `minimize`), a third-party dependency not resolvable by rosdep — install separately, e.g. `pip install scipy`.

## Tests

No functional tests, and the standard `ament_flake8`/`ament_pep257` lint tests are actually **red** — verified with a real, isolated `colcon test`: 153 flake8 style errors/warnings and a `pep257` failure (see `AUDIT.md` on the `doc` branch for the full picture across all of RoboticPark's packages). None of `lagrange_multipliers.py`'s geometry math (`Sphere`/`Cone`/`Ellipsoid` `distance()`/`value()`/`projection()`) has been extracted into anything testable independently of a running node — despite being pure math with no ROS dependency, making it a strong candidate for the same kind of unit testing already applied to `PIDController` in `uned_crazyflie_driver`/`uned_kheperaiv_driver`.
