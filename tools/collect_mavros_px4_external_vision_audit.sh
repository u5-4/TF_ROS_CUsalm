#!/usr/bin/env bash

# Read-only audit of the installed MAVROS external-vision endpoints and the
# minimal PX4 EKF2 external-vision parameter set. It never creates external-
# vision or control topic publishers, writes parameters, changes mode, arms,
# or enters OFFBOARD.

set -u

error_count=0
warning_count=0

section()
{
  printf '\n========== %s ==========\n' "$1"
}

show_topic_endpoint()
{
  topic="$1"
  requirement="$2"
  expected_type="$3"
  expected_endpoint="$4"
  expected_node="$5"
  printf '\n----- %s -----\n' "$topic"
  endpoint_output="$(timeout 5s ros2 topic info --verbose "$topic" 2>&1)"
  endpoint_rc=$?
  printf '%s\n' "$endpoint_output"
  if [ "$endpoint_rc" -ne 0 ]; then
    printf 'endpoint_status=MISSING requirement=%s\n' "$requirement"
    if [ "$requirement" = "required" ]; then
      error_count=$((error_count + 1))
    else
      warning_count=$((warning_count + 1))
    fi
  elif printf '%s\n' "$endpoint_output" | awk \
    -v expected_type="$expected_type" \
    -v expected_endpoint="$expected_endpoint" \
    -v expected_node="$expected_node" '
      /^Node name:/ {name = $3; node_namespace = ""; type = ""; endpoint = ""}
      /^Node namespace:/ {node_namespace = $3}
      /^Topic type:/ {type = $3}
      /^Endpoint type:/ {endpoint = $3}
      /^QoS profile:/ {
        if (name == expected_node && node_namespace == "/mavros" &&
            type == expected_type && endpoint == expected_endpoint) {
          found = 1
        }
      }
      END {exit found ? 0 : 1}
    '
  then
    echo "endpoint_contract=VERIFIED"
  else
    echo "endpoint_contract=NOT_VERIFIED"
    if [ "$requirement" = "required" ]; then
      error_count=$((error_count + 1))
    else
      warning_count=$((warning_count + 1))
    fi
  fi
}

read_px4_parameter()
{
  parameter="$1"
  printf '\n----- %s -----\n' "$parameter"
  parameter_output="$(timeout 10s ros2 service call \
    /mavros/param/get mavros_msgs/srv/ParamGet \
    "{param_id: '${parameter}'}" 2>&1)"
  parameter_rc=$?
  printf '%s\n' "$parameter_output"
  if [ "$parameter_rc" -ne 0 ] || \
    { [[ "$parameter_output" != *"success=True"* ]] && \
      [[ "$parameter_output" != *"success: true"* ]]; }
  then
    error_count=$((error_count + 1))
  fi
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"
if [ "${ROS_DOMAIN_ID:-UNSET}" != "42" ]; then
  error_count=$((error_count + 1))
  echo "ros_domain_contract=MISMATCH_EXPECTED_42"
fi
if [ "${RMW_IMPLEMENTATION:-UNSET}" != "rmw_fastrtps_cpp" ]; then
  error_count=$((error_count + 1))
  echo "rmw_contract=MISMATCH_EXPECTED_rmw_fastrtps_cpp"
fi

section "Installed MAVROS identity"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2=UNAVAILABLE"
  exit 2
fi
if ! dpkg-query -W \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs 2>&1
then
  error_count=$((error_count + 1))
fi
apt-cache policy \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs 2>&1 || true
printf 'mavros_prefix='
if ! ros2 pkg prefix mavros 2>&1; then
  error_count=$((error_count + 1))
fi
printf 'mavros_extras_prefix='
if ! ros2 pkg prefix mavros_extras 2>&1; then
  error_count=$((error_count + 1))
fi
printf 'mavros_msgs_prefix='
if ! ros2 pkg prefix mavros_msgs 2>&1; then
  error_count=$((error_count + 1))
fi

section "Relevant ROS message contracts"
for interface in \
  geometry_msgs/msg/PoseStamped \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  nav_msgs/msg/Odometry \
  mavros_msgs/msg/TimesyncStatus \
  mavros_msgs/srv/ParamGet
do
  printf '\n----- %s -----\n' "$interface"
  if ! ros2 interface show "$interface" 2>&1; then
    error_count=$((error_count + 1))
  fi
done

section "MAVROS node and plugin parameters"
ros2 node list 2>&1 || true
for node in /mavros/mavros_node /mavros/mavros /mavros/vision_pose
do
  printf '\n----- %s -----\n' "$node"
  if ! timeout 10s ros2 node info "$node" 2>&1; then
    error_count=$((error_count + 1))
  fi
  if ! timeout 10s ros2 param list "$node" 2>&1; then
    error_count=$((error_count + 1))
  fi
done
printf '\n----- vision_pose tf.listen -----\n'
tf_listen_output="$(timeout 5s ros2 param get \
  /mavros/vision_pose tf.listen 2>&1)"
tf_listen_rc=$?
printf '%s\n' "$tf_listen_output"
if [ "$tf_listen_rc" -ne 0 ] || \
  [[ "$tf_listen_output" != *"Boolean value is: False"* ]]
then
  error_count=$((error_count + 1))
  echo "vision_pose_tf_listener=NOT_VERIFIED_DISABLED"
else
  echo "vision_pose_tf_listener=DISABLED"
fi
for parameter in \
  fcu_url \
  gcs_url \
  tgt_system \
  tgt_component \
  base_link_frame \
  map_frame \
  odom_frame
do
  printf '\n----- %s -----\n' "$parameter"
  if ! timeout 5s ros2 param get \
    /mavros/mavros_node "$parameter" 2>&1
  then
    error_count=$((error_count + 1))
  fi
done

section "External-vision endpoint graph"
show_topic_endpoint \
  /mavros/state required mavros_msgs/msg/State PUBLISHER sys
show_topic_endpoint \
  /mavros/local_position/odom required nav_msgs/msg/Odometry PUBLISHER \
  local_position
show_topic_endpoint \
  /mavros/timesync_status required mavros_msgs/msg/TimesyncStatus PUBLISHER time
show_topic_endpoint \
  /mavros/vision_pose/pose_cov required \
  geometry_msgs/msg/PoseWithCovarianceStamped SUBSCRIPTION vision_pose
show_topic_endpoint \
  /mavros/vision_pose/pose optional geometry_msgs/msg/PoseStamped SUBSCRIPTION \
  vision_pose
show_topic_endpoint \
  /mavros/odometry/out optional nav_msgs/msg/Odometry SUBSCRIPTION odometry
show_topic_endpoint \
  /mavros/mocap/pose optional geometry_msgs/msg/PoseStamped SUBSCRIPTION mocap
show_topic_endpoint \
  /mavros/odometry/in optional nav_msgs/msg/Odometry PUBLISHER odometry

section "External-vision interface decision"
echo "selected_input=/mavros/vision_pose/pose_cov"
echo "selected_type=geometry_msgs/msg/PoseWithCovarianceStamped"
echo "selected_covariance=ALL_NAN_UNKNOWN_PENDING_BENCH_PASS_THROUGH"
echo "pose_input=REJECTED_ZERO_INITIALIZED_COVARIANCE"
echo "odometry_out_input=REJECTED_FOR_POSE_ONLY_SOURCES"
echo "reason=NO_FAKE_ZERO_COVARIANCE_OR_TWIST"

section "MAVROS and PX4 state"
state_output="$(timeout 5s ros2 topic echo /mavros/state \
  mavros_msgs/msg/State --once 2>&1)"
state_rc=$?
printf '%s\n' "$state_output"
if [ "$state_rc" -ne 0 ]; then
  error_count=$((error_count + 1))
fi

section "Read-only runtime samples"
if ! timeout 5s ros2 topic echo /mavros/timesync_status \
  mavros_msgs/msg/TimesyncStatus --once 2>&1
then
  error_count=$((error_count + 1))
fi
if ! timeout 5s ros2 topic echo /mavros/local_position/odom \
  nav_msgs/msg/Odometry --once --no-arr 2>&1
then
  error_count=$((error_count + 1))
fi

section "Minimal PX4 external-vision parameters"
param_get_type="$(
  timeout 5s ros2 service type /mavros/param/get 2>/dev/null || true
)"
printf 'param_get_service_type=%s\n' "${param_get_type:-UNAVAILABLE}"
if [[ "$state_output" == *"connected: true"* ]] && \
  [[ "$state_output" == *"armed: false"* ]] && \
  [[ "$param_get_type" == "mavros_msgs/srv/ParamGet" ]]
then
  for parameter in \
    EKF2_EV_CTRL \
    EKF2_EV_DELAY \
    EKF2_EV_NOISE_MD \
    EKF2_EV_QMIN \
    EKF2_EV_POS_X \
    EKF2_EV_POS_Y \
    EKF2_EV_POS_Z \
    EKF2_EVP_NOISE \
    EKF2_EVP_GATE \
    EKF2_EVV_NOISE \
    EKF2_EVV_GATE \
    EKF2_EVA_NOISE \
    EKF2_HGT_REF \
    EKF2_GPS_CTRL \
    EKF2_BARO_CTRL \
    EKF2_RNG_CTRL \
    EKF2_MAG_TYPE \
    EKF2_NOAID_TOUT
  do
    read_px4_parameter "$parameter"
  done
else
  echo "parameter_reads=SKIPPED_REQUIRES_CONNECTED_AND_DISARMED_PX4"
  error_count=$((error_count + 1))
fi

section "Completion"
echo "scope=mavros_px4_external_vision"
printf 'errors=%d\n' "$error_count"
printf 'warnings=%d\n' "$warning_count"
echo "external_vision_topic_publishers_created=0"
echo "parameter_writes=0"
echo "mode_changes=0"
echo "arming_commands=0"
echo "offboard_commands=0"
echo "authorization=NONE"
if [ "$error_count" -eq 0 ]; then
  echo "analysis=READ_ONLY_AUDIT_COMPLETE"
  exit 0
fi
echo "analysis=INCOMPLETE"
exit 2
