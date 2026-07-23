#!/usr/bin/env bash

# Read-only inventory for the isolated YOPO runtime. Run this from the same
# host environment or container that will execute YOPO.

set -u

yopo_repo="${YOPO_REPO:-/home/nvidia/catkin_ws/src/YOPO_ROS2}"

section()
{
  printf '\n========== %s ==========\n' "$1"
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'yopo_repo=%s\n' "$yopo_repo"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"
printf 'CONDA_DEFAULT_ENV=%s\n' "${CONDA_DEFAULT_ENV:-UNSET}"
printf 'CONDA_PREFIX=%s\n' "${CONDA_PREFIX:-UNSET}"
if command -v conda >/dev/null 2>&1; then
  conda info --envs 2>&1 || true
fi

section "YOPO repository"
if [ ! -d "${yopo_repo}/.git" ]; then
  echo "status=MISSING"
else
  echo "status=FOUND"
  printf 'branch='
  git -C "$yopo_repo" rev-parse --abbrev-ref HEAD 2>&1 || true
  printf 'commit='
  git -C "$yopo_repo" rev-parse HEAD 2>&1 || true
  echo "remotes:"
  git -C "$yopo_repo" remote -v 2>&1 || true
  echo "working_tree:"
  if [ -z "$(git -C "$yopo_repo" status --porcelain 2>/dev/null)" ]; then
    echo "CLEAN"
  else
    git -C "$yopo_repo" status --short 2>&1 || true
  fi
fi

section "YOPO immutable files"
for file in \
  "${yopo_repo}/YOPO/saved/YOPO_1/epoch50.pth" \
  "${yopo_repo}/YOPO/config/traj_opt.yaml" \
  "${yopo_repo}/YOPO/requirements.txt"
do
  if [ -f "$file" ]; then
    sha256sum "$file"
  else
    echo "MISSING $file"
  fi
done

section "YOPO Python runtime"
python3 --version 2>&1 || true
python3 -c 'import torch; print("torch_version=" + torch.__version__); print("torch_cuda_available=" + str(torch.cuda.is_available()))' 2>&1 || true
python3 -c 'import numpy; print("numpy_version=" + numpy.__version__)' 2>&1 || true

section "YOPO ROS installation"
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix yopo_planner 2>&1 || true
  ros2 pkg prefix quadrotor_msgs 2>&1 || true
else
  echo "ros2=UNAVAILABLE"
fi

section "Completion"
echo "scope=yopo_runtime"
echo "analysis=READ_ONLY_INVENTORY_COMPLETE"
echo "authorization=NONE"
