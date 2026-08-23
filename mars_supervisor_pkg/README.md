# mars_supervisor_pkg

`ament_python` package with a supervision/orchestration node for MARS (Multi-Agent Robotic Systems) experiments.

## Nodes

- **`supervisor_node`** (`supervisor_node.py`, node name `supervisor_node`): `Supervisor`, a generic scripted command sequencer. Reads two files via ROS parameters — `config` (an experience `.yaml`, only used here to check `Data_Logging.enable` and copy itself alongside the run's log folder) and `file` (a command script: `cmd00`, `cmd01`, ... in order, each with a `topic`/`type`/`value` and a `trigger`, either a fixed delay or waiting for a matching value on a subscribed topic). Publishes `String`/`PoseStamped`/`Float64` messages on the topics declared in the command file's `config.publisher` section. The same kind of scripted-sequence pattern as `uned_crazyflie_missions`' `sequencer` node, developed independently in this repo.

This package used to also carry a second, diverging copy of the herding/affine-formation control node (`affine_formation_node.py`) also present in `multi_agent_pkg`. Removed on 2026-08-23 in favor of `multi_agent_pkg`'s more actively-developed version — see that package's README.

## Dependencies

`rclpy`, `std_msgs`, `geometry_msgs`, `tf_transformations` — declared in `package.xml`/`setup.py`.

## Tests

No functional tests. `ament_pep257` passes; `ament_flake8` is red — verified with a real, isolated `colcon test`: 46 style errors/warnings (see `AUDIT.md` on the `doc` branch for the full picture across all of RoboticPark's packages). `Supervisor` is a full `rclpy` node with topic/parameter wiring and file-reading logic done straight from its methods; none of it has been extracted into pure, unit-testable functions.
