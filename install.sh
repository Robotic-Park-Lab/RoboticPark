#! /bin/bash
set -uo pipefail

# ---------------------------------------------------------------------------
# Robotic Park Lab -- instalador del workspace
#
# Uso:
#   ./install.sh [--ros-distro humble] [--benchmark|--dev] [--workspace DIR]
#
#   --ros-distro  Distro ROS 2. Hoy solo "humble" tiene rama *-dev activa en
#                 los repos del laboratorio; pide una rama <distro>-dev antes
#                 de usar otro valor.
#   --benchmark   Instala la configuración exacta del capítulo de libro sobre
#                 control: clona la rama congelada `benchmark` de cada repo
#                 del laboratorio que la tiene (uned_crazyflie_ros_pkg,
#                 uned_kheperaIV_ros_pkg, uned_multi_agent_ros_pkg) en vez de
#                 la rama de desarrollo activo.
#   --dev         Fuerza el modo de desarrollo activo aunque este propio
#                 RoboticPark se haya clonado en `benchmark` (para pruebas).
#   --workspace   Raíz del workspace. Por defecto, el directorio padre de
#                 donde vive este script (se espera <workspace>/src/RoboticPark),
#                 o ~/roboticpark_ws si no se puede determinar.
#
# Sin argumentos, el modo se detecta según la rama actual de este propio
# RoboticPark: `benchmark` -> --benchmark, cualquier otra -> --dev.
# ---------------------------------------------------------------------------

ROS_DISTRO_ARG="humble"
MODE=""
WORKSPACE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --ros-distro) ROS_DISTRO_ARG="$2"; shift 2 ;;
    --benchmark)  MODE="benchmark"; shift ;;
    --dev)        MODE="dev"; shift ;;
    --workspace)  WORKSPACE="$2"; shift 2 ;;
    *) echo "Opción desconocida: $1" >&2; exit 1 ;;
  esac
done

if [ "$ROS_DISTRO_ARG" != "humble" ]; then
  echo "Solo humble tiene rama *-dev activa en los repos del laboratorio hoy." >&2
  echo "Pide (o crea) una rama <distro>-dev antes de usar --ros-distro $ROS_DISTRO_ARG." >&2
  exit 1
fi
DEV_BRANCH="humble-dev"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$MODE" ]; then
  CURRENT_BRANCH="$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")"
  if [ "$CURRENT_BRANCH" = "benchmark" ]; then
    MODE="benchmark"
  else
    MODE="dev"
  fi
fi

if [ "$MODE" = "benchmark" ]; then
  BRANCH="benchmark"
  echo -e "Modo: reproducir benchmark (capítulo de libro sobre control)\n"
else
  BRANCH="$DEV_BRANCH"
  echo -e "Modo: desarrollo activo ($DEV_BRANCH)\n"
fi

if [ -z "$WORKSPACE" ]; then
  # SCRIPT_DIR es normalmente <workspace>/src/RoboticPark
  if [ -d "$SCRIPT_DIR/../.." ]; then
    WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"
  else
    WORKSPACE="$HOME/roboticpark_ws"
  fi
fi
SRC_DIR="$WORKSPACE/src"
mkdir -p "$SRC_DIR"

echo -e "Robotic Park install\n"
echo -e "Workspace: $WORKSPACE"
echo -e "Checking and installing dependencies to build ..."

sudo apt install -y "ros-${ROS_DISTRO_ARG}-webots-ros2"

echo -e "\nWebots:"
if ! apt list --installed 2>/dev/null | grep -q "^webots/"; then
    sudo mkdir -p /etc/apt/keyrings
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc -O /etc/apt/keyrings/Cyberbotics.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
    sudo apt update
    sudo apt install -y webots
else
    echo -e "\tInstalled."
fi

clone_if_missing() {
  # clone_if_missing <paquete-a-comprobar> <rama> <url-repo> <carpeta-destino>
  local pkg="$1" branch="$2" url="$3" dest="$4"
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo -e "\tInstalled."
  elif [ -d "$SRC_DIR/$dest" ]; then
    echo -e "\tYa clonado en $SRC_DIR/$dest (pendiente de compilar)."
  else
    echo -e "\tInstalling ..."
    git clone -b "$branch" "$url" "$SRC_DIR/$dest"
  fi
}

echo -e "\nCrazyflie package:"
clone_if_missing uned_crazyflie_config "$BRANCH" https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git uned_crazyflie_ros_pkg

echo -e "\nVicon package:"
clone_if_missing vicon_receiver main https://github.com/Robotic-Park-Lab/ros2-vicon-receiver.git ros2-vicon-receiver

echo -e "\nMulti-agent package:"
# El repo se llama uned_multi_agent_ros_pkg (antes uned_swarm_ros_pkg); sus
# paquetes ROS internos siguen llamándose uned_swarm_* por ahora.
clone_if_missing uned_swarm_config "$BRANCH" https://github.com/Robotic-Park-Lab/uned_multi_agent_ros_pkg.git uned_multi_agent_ros_pkg

echo -e "\nKhepera IV package:"
clone_if_missing uned_kheperaiv_config "$BRANCH" https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg.git uned_kheperaIV_ros_pkg

echo -e "\nCrazyflie-lib-python:"
CODE_DIR="$HOME/Code"
mkdir -p "$CODE_DIR"
if [ -d "$CODE_DIR/crazyflie-lib-python" ]; then
    echo -e "\tInstalled."
else
    git clone https://github.com/Robotic-Park-Lab/crazyflie-lib-python.git "$CODE_DIR/crazyflie-lib-python"
    pip install -e "$CODE_DIR/crazyflie-lib-python"
fi

echo -e "\nResolviendo dependencias con rosdep ..."
cd "$WORKSPACE"
rosdep update
rosdep install --from-paths src -y --ignore-src
# colcon build --symlink-install

echo -e "\nListo. Compila con:\n  cd $WORKSPACE && colcon build --symlink-install && source install/setup.bash"
