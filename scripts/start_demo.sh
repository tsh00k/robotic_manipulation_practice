#!/usr/bin/env bash
# Build the workspace, source its environment, and start the complete demo.
#
# Examples:
#   scripts/start_demo.sh
#   scripts/start_demo.sh --no-build
#   scripts/start_demo.sh --packages-select mujoco_bridge task_executor
#   scripts/start_demo.sh --no-build -- enable_debug_viewer:=true

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/start_demo.sh [options] [-- launch-arguments]

Builds the workspace (unless --no-build is given), sources ROS 2 and the local
install space, then runs mujoco_bridge's demo launch file.

Options:
  --no-build              Reuse the existing build/install spaces.
  --no-symlink-install    Do not pass --symlink-install to colcon build.
  --packages-select PKG   Build only selected packages (may be repeated).
  -h, --help              Show this help.

Everything after '--' is passed unchanged to demo.launch.py, for example:
  -- enable_debug_viewer:=true joint_state_rate_hz:=200
EOF
}

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
DO_BUILD=1
SYMLINK_INSTALL=1
BUILD_ARGS=()
LAUNCH_ARGS=()

while (($# > 0)); do
  case "$1" in
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    --no-build)
      DO_BUILD=0
      shift
      ;;
    --no-symlink-install)
      SYMLINK_INSTALL=0
      shift
      ;;
    --packages-select)
      [[ $# -ge 2 ]] || { echo "--packages-select requires a package name" >&2; exit 2; }
      BUILD_ARGS+=("--packages-select" "$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1 (use -- to pass launch arguments)" >&2
      exit 2
      ;;
  esac
done

cd "$REPO_ROOT"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 Humble setup not found: $ROS_SETUP" >&2
  echo "Run this script inside the robotics-dev container." >&2
  exit 1
fi

# ROS setup scripts intentionally read optional variables that may be unset.
# Keep strict checking for this script, but do not apply nounset while sourcing
# external environment setup files.
source_environment() {
  set +u
  # shellcheck disable=SC1091
  source "$1"
  set -u
}

source_environment "$ROS_SETUP"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available in PATH; enter the robotics-dev container first." >&2
  exit 1
fi

if ((DO_BUILD)); then
  build_command=(colcon build)
  if ((SYMLINK_INSTALL)); then
    build_command+=(--symlink-install)
  fi
  build_command+=("${BUILD_ARGS[@]}")
  echo "+ ${build_command[*]}"
  "${build_command[@]}"
fi

if [[ ! -f "$REPO_ROOT/install/local_setup.bash" ]]; then
  echo "install/local_setup.bash is missing; run without --no-build first." >&2
  exit 1
fi

source_environment "$REPO_ROOT/install/local_setup.bash"

echo "+ ros2 launch mujoco_bridge demo.launch.py ${LAUNCH_ARGS[*]}"
exec ros2 launch mujoco_bridge demo.launch.py "${LAUNCH_ARGS[@]}"
