# measure_process_ros2_pkg

`ament_python` package with a single node for measuring and logging system/process resource usage during experiments.

## Nodes

- **`measure_process`** (`measure_process.py`, node name `measure_process`): `MeasureProcess`, periodically (`process_period` parameter, default 0.5s) publishes overall system stats — CPU count, CPU%, memory%, and the 1/5/15-minute load averages as a percentage of CPU count — on `cpu_stats` (`Float64MultiArray`), via [`psutil`](https://psutil.readthedocs.io/). It also tracks specific processes named in its `process_name` parameter (comma-separated, e.g. `gzserver, webots`) and publishes each one's own CPU% on `<process_name>_cpu` (dashes in the name replaced with underscores for the topic). Used to correlate simulator/driver CPU load with experiment results (see `measure_process_ros2_pkg` usage from `roboticpark_config`'s `CPU_Monitoring` experience section).

## Dependencies

`rclpy`, `std_msgs` — declared in `package.xml`. The third-party `psutil` is also required (not a rosdep-resolvable ROS package — install separately, e.g. `pip install psutil`).

## Tests

No functional tests. `ament_pep257` passes; `ament_flake8` is red — verified with a real, isolated `colcon test`: 11 style errors/warnings (see `AUDIT.md` on the `doc` branch for the full picture across all of RoboticPark's packages). `MeasureProcess.do_measure()` depends on the real, live process table (`psutil.process_iter()`) and system-wide CPU/memory counters — not meaningfully unit-testable without mocking `psutil` itself, not attempted here.
