# scripts

Auxiliary utilities that are not part of any ROS package: the Matlab pipeline behind the book-chapter benchmark (`benchmark` branch) and its bag post-processing script.

## Matlab benchmark pipeline (`RP_benchmark.m`, `problem1.m`, `benchmark_evaluation.m`)

These three files together are the actual generation/evaluation pipeline behind the published book-chapter benchmark reproduced from the `benchmark` branch (see the root README's "Reproducing a published experiment" section) — worth linking from `roboticpark_config`'s "Available experiences" table once the corresponding `ControlFormation*` config files are documented there.

- **`problem1.m`** (function): given a robot count `N` and radius `R`, computes the drones needed to form a hemisphere using the maximum allowed horizontal distance between drones — returns `D` (target inter-robot distances), `E` (the resulting graph edges), `P` (3D positions).
- **`RP_benchmark.m`** (script): calls `problem1(N, R, 0)` and, from its output, **generates a full experience config from scratch** — `default_config.yaml` (the same `Operation`/`Experience`/`Architecture`/`CPU_Monitoring`/`Interface`/`Data_Logging`/`Robots`/`Supervisor`/`Other` schema read by `roboticpark_config/launch/experience.launch.py`, with one `Robots.RobotNN` entry per drone, its computed pose, and its `task.relationship` distance constraints from `E`/`D`), plus a matching `default_world.wbt` (Webots world with `N` Crazyflies at the computed positions) and `default_config.rviz`, all written into a new `ControlFormationNN_files/` folder. This is what produced the `ControlFormation*` family of experience files in `roboticpark_config/resources/`.
- **`benchmark_evaluation.m`** (script): reads back a recorded experiment's `default_config.yaml` (from a dated dataset folder, e.g. `2024-01-20-22-09/`) to reconstruct the formation topology, then evaluates the logged data against it — this is the benchmark's actual evaluation/metrics step, run after a `ros2 bag` recording from a real or simulated run.

## `bag2csv_benchmark.py`

Converts every topic in a `ros2 bag` to a `.csv` file, one per topic, based on the message type — the original this repo's `uned_crazyflie_ros_pkg/scripts/Python/bag2csv.py` was adapted from (that version added 2 more supported message types; this one is the source of record for the benchmark's own bag post-processing).

## Tests

None — these are standalone analysis scripts (2 languages, neither is a ROS package), not something `colcon test` covers.
