#!/usr/bin/env bash

# Read-only inventory for the YOPO Jetson/PX4 baseline. This script does not
# start nodes, change parameters, arm the vehicle, or write to the flight stack.

set -u

workspace="${WORKSPACE:-/workspaces/isaac_ros-dev}"
yopo_repo="${YOPO_REPO:-${workspace}/src/YOPO_ROS2}"

section()
{
  printf '\n========== %s ==========\n' "$1"
}

repo_snapshot()
{
  label="$1"
  path="$2"

  printf '\n[%s]\npath=%s\n' "$label" "$path"
  if [ ! -d "${path}/.git" ]; then
    echo "status=MISSING"
    return
  fi

  echo "status=FOUND"
  printf 'branch='
  git -C "$path" rev-parse --abbrev-ref HEAD 2>&1 || true
  printf 'commit='
  git -C "$path" rev-parse HEAD 2>&1 || true
  echo "remotes:"
  git -C "$path" remote -v 2>&1 || true
  echo "working_tree:"
  if [ -z "$(git -C "$path" status --porcelain 2>/dev/null)" ]; then
    echo "CLEAN"
  else
    git -C "$path" status --short 2>&1 || true
  fi
}

package_snapshot()
{
  package="$1"
  echo "[${package}]"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2=UNAVAILABLE"
    return
  fi
  ros2 pkg prefix "$package" 2>&1 || true
  ros2 pkg xml "$package" 2>&1 | sed -n '/<name>/p; /<version>/p' || true
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'workspace=%s\n' "$workspace"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"

section "Operating system"
uname -a 2>&1 || true
cat /etc/os-release 2>&1 || true
if [ -r /etc/nv_tegra_release ]; then
  cat /etc/nv_tegra_release
fi
dpkg-query -W nvidia-jetpack 2>&1 || true
if command -v nvcc >/dev/null 2>&1; then
  nvcc --version 2>&1 || true
elif [ -x /usr/local/cuda/bin/nvcc ]; then
  /usr/local/cuda/bin/nvcc --version 2>&1 || true
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi 2>&1 || true
fi

section "ROS environment"
set +u
if [ -r /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -r "${workspace}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${workspace}/install/setup.bash"
fi
set -u
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-UNSET}"
printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-UNSET}"

section "Git repositories"
repo_snapshot "TF_ROS_CUsalm" "${workspace}/src/TF_ROS_CUsalm"
repo_snapshot "cuVSLAM bringup" "${workspace}/src/cuvslam-yopo-adapter"
repo_snapshot "isaac_ros_visual_slam" "${workspace}/src/isaac_ros_visual_slam"
repo_snapshot "vrpn_client_ros2" "${workspace}/src/vrpn_client_ros2"
repo_snapshot "YOPO_ROS2" "$yopo_repo"

section "ROS and Debian packages"
package_snapshot "isaac_ros_yopo_bringup"
package_snapshot "isaac_ros_visual_slam"
package_snapshot "mavros"
package_snapshot "mavros_msgs"
package_snapshot "realsense2_camera"
package_snapshot "vrpn_client_ros"
dpkg-query -W \
  ros-humble-mavros \
  ros-humble-mavros-msgs \
  ros-humble-realsense2-camera 2>&1 || true

section "Python and YOPO runtime"
python3 -c 'import torch; print("torch_version=" + torch.__version__); print("torch_cuda_available=" + str(torch.cuda.is_available()))' 2>&1 || true

section "Container runtime"
docker version 2>&1 || true
docker image inspect \
  isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-jp6.2 \
  isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-vslam3.2-dev-20260717 \
  --format 'tags={{json .RepoTags}} id={{.Id}}' 2>&1 || true

section "RealSense device"
lsusb -d 8086:0b3a 2>&1 || true
lsusb -t 2>&1 || true
pkg-config --modversion realsense2 2>&1 || true
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  timeout 10s rs-enumerate-devices -s 2>&1 || true
else
  echo "rs-enumerate-devices=UNAVAILABLE"
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

section "MAVROS connection"
if command -v ros2 >/dev/null 2>&1; then
  timeout 5s ros2 topic echo /mavros/state --once 2>&1 || true
  vehicle_info_type="$(timeout 5s ros2 service type \
    /mavros/vehicle_info_get 2>/dev/null || true)"
  printf 'vehicle_info_service_type=%s\n' "${vehicle_info_type:-UNAVAILABLE}"
  if [ -n "$vehicle_info_type" ]; then
    timeout 10s ros2 service call /mavros/vehicle_info_get \
      "$vehicle_info_type" '{}' 2>&1 || true
  fi
else
  echo "ros2=UNAVAILABLE"
fi

section "PX4 critical parameters"
if command -v ros2 >/dev/null 2>&1; then
  for parameter in \
    MPC_THR_HOVER \
    EKF2_EV_CTRL \
    EKF2_AID_MASK \
    EKF2_EV_DELAY \
    EKF2_HGT_REF \
    COM_OF_LOSS_T \
    COM_OBL_RC_ACT
  do
    echo "[${parameter}]"
    timeout 5s ros2 service call /mavros/param/get \
      mavros_msgs/srv/ParamGet "{param_id: '${parameter}'}" 2>&1 || true
  done
else
  echo "ros2=UNAVAILABLE"
fi

section "Completion"
echo "analysis=READ_ONLY_INVENTORY_COMPLETE"
echo "authorization=NONE"
