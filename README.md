# RoboticPark

> 📖 To understand this repo's branches and its contribution guide, see the [`doc`](https://github.com/Robotic-Park-Lab/RoboticPark/tree/doc) branch.

Main repository of [Robotic Park Lab](https://github.com/Robotic-Park-Lab). This is where the lab's complete infrastructure gets installed on a new machine — the rest of the UNED packages, the Vicon system, and simulation dependencies (Webots) — and where the exact configuration used in specific lab publications gets reproduced.

#### Structure
- **[mars_supervisor_pkg](mars_supervisor_pkg/README.md)**. Supervision of the experimental system (MARS).
- **[measure_process_ros2_pkg](measure_process_ros2_pkg/README.md)**. Measurement and logging of system processes (CPU/memory) during experiments.
- **[multi_agent_pkg](multi_agent_pkg/README.md)**. Shared utilities for multi-agent experiments combining several lab platforms — formation-control geometry math consumed by `uned_crazyflie_driver`/`uned_kheperaiv_webots`, and the lab's active herding/affine-formation control law.
- **[roboticpark_config](roboticpark_config/README.md)**. Configuration shared by the rest of the lab's repositories (launched as a dependency from `uned_*_ros_pkg`).
- **[scripts](scripts/README.md)**. Auxiliary utilities that are not part of any ROS package.

## Installation :book:

Active development, on ROS 2 Humble:
```
mkdir -p ~/roboticpark_ws/src
cd ~/roboticpark_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark
chmod +x install.sh
./install.sh
```

`install.sh` detects the mode (development or benchmark) from the branch you cloned RoboticPark on, but also accepts explicit arguments:
```
./install.sh [--ros-distro humble] [--benchmark|--dev] [--workspace DIR]
```
- `--ros-distro`: only `humble` has an active `*-dev` branch across the lab's repos today.
- `--benchmark` / `--dev`: forces the mode regardless of the current branch.
- `--workspace`: workspace root, if you don't want `~/roboticpark_ws`.

## Usage 🔧

### Launching an experience

Like the Crazyflie, Khepera IV and multi-agent repos, `roboticpark_config` has a **single parametrized launch file**, `roboticpark_config/launch/experience.launch.py`. Each experience is a `.yaml` file in `roboticpark_config/resources/`:
```
ros2 launch roboticpark_config experience.launch.py config_file:=<experience>.yaml
```
See [roboticpark_config/README.md](roboticpark_config/README.md) for the full section schema (`Operation`/`Architecture`/`Robots`/`CPU_Monitoring`/`Interface`) and how it's shared with the other repos' own experience launch files.

### Available experiences

`roboticpark_config/resources/` currently holds close to 90 experience `.yaml` files, grouped by research line (`ControlFormation*`, `LagrangeMultipliers_{cone,sphere}*`, `IROS_AffineFormation*`, `Morphing_*`, `Gimbal*`/`gimbal_*` identification and relay-tuning configs, `CoverageBenchmark*`, `OpenLoop_Crazyflie_*`, `SphereFormation*`, `Crazyflie_Demo*`, `Khepera_rele*`, plus `default_config.yaml`/`test.yaml`). This table is meant to be filled in incrementally by Francisco as each experience gets (re)documented — add a row per experience you want documented here:

| `config_file` | Description |
|---|---|
| _(to be filled in)_ | |

### Matlab benchmark scripts

See [scripts/README.md](scripts/README.md) for `RP_benchmark.m`/`benchmark_evaluation.m`/`problem1.m` and the `bag2csv_benchmark.py` post-processing script.

## Reproducing a published experiment :paperclip:

The `benchmark` branch backs Francisco Mañas's book chapter on control and **is not modified or renamed**. To reproduce that exact configuration:
```
mkdir -p ~/roboticpark_benchmark_ws/src
cd ~/roboticpark_benchmark_ws/src
git clone -b benchmark https://github.com/Robotic-Park-Lab/RoboticPark.git
cd RoboticPark
chmod +x install.sh
./install.sh
```
`install.sh` detects that RoboticPark is on `benchmark` and automatically clones the `benchmark` branches of `uned_crazyflie_ros_pkg`, `uned_kheperaIV_ros_pkg` and `uned_multi_agent_ros_pkg` (not `ros2-vicon-receiver`, which has no `benchmark` branch of its own).

## Authors ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Related publications :paperclip:

Each entry below should list, alongside the citation, the specific `roboticpark_config` launch/config file used for that publication's experiments (e.g. `experience.launch.py config_file:=<file>.yaml`, or `benchmark` for the frozen branch) — add entries as they get documented.

| Publication | Launch / config used |
|---|---|
| Book chapter on control — Francisco Mañas | `benchmark` branch (see "Reproducing a published experiment" above) |
| _(to be filled in)_ | |
