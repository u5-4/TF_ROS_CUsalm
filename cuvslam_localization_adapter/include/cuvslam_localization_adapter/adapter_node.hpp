// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
#define CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cuvslam_localization_adapter/contract_config.hpp"
#include "localization_contracts/gate.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/validation.hpp"

namespace cuvslam_localization_adapter
{

class CuvslamLocalizationAdapter final : public rclcpp::Node
{
public:
  explicit CuvslamLocalizationAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct UpstreamDiagnosticObservation
  {
    std::uint8_t level;
    std::int64_t stamp_ns;
    SteadyTime received_at;
  };

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void OnVisualSlamStatus(
    const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::SharedPtr message);
  void OnUpstreamDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnDiagnosticTimer();

  bool InputsAreHealthyLocked(const SteadyTime & now, std::string * reason) const;
  void UpdatePublisherEvidenceLocked();
  void UpdateOdometryRateLocked(std::int64_t stamp_ns);
  bool ClockDomainMatches(std::int64_t stamp_ns, double * residual_sec) const;
  void MaybeAdvanceHealthLocked(const SteadyTime & now);
  void PublishDiagnosticsLocked(const SteadyTime & now);
  std::optional<double> AgeSeconds(
    const std::optional<SteadyTime> & received_at,
    const SteadyTime & now) const;

  std::string mode_;
  std::string contract_file_;
  ContractConfig contract_;
  std::string diagnostic_status_name_;
  localization_contracts::RuntimeHealthGate health_gate_;
  localization_contracts::StrictStampGuard odometry_sequence_guard_;
  localization_contracts::StrictStampGuard stamp_guard_;
  localization_contracts::StrictStampGuard status_sequence_guard_;
  localization_contracts::StrictStampGuard status_stamp_guard_;
  std::unordered_map<std::string, localization_contracts::StrictStampGuard>
    diagnostic_sequence_guards_;

  std::chrono::steady_clock::time_point started_at_;
  std::optional<SteadyTime> last_odometry_received_at_;
  std::optional<SteadyTime> last_odometry_observed_at_;
  std::optional<SteadyTime> last_status_received_at_;
  std::optional<std::int64_t> last_health_observed_stamp_ns_;
  bool last_odometry_valid_{false};
  bool last_status_valid_{false};
  std::uint8_t last_vo_state_{0U};
  std::string last_actual_parent_frame_;
  std::string last_actual_child_frame_;
  std::unordered_map<std::string, UpstreamDiagnosticObservation> upstream_diagnostics_;
  std::deque<std::int64_t> recent_odometry_stamps_ns_;
  std::optional<double> observed_odometry_rate_hz_;
  std::optional<double> maximum_recent_stamp_gap_sec_;
  std::optional<double> last_odometry_receive_gap_sec_;
  std::optional<double> last_odometry_clock_residual_sec_;
  std::optional<double> last_status_clock_residual_sec_;
  std::optional<double> last_diagnostic_clock_residual_sec_;
  std::optional<localization_contracts::RigidTransform> last_input_pose_;
  std::optional<localization_contracts::RigidTransform> last_shadow_pose_;

  std::size_t odometry_publisher_count_{0U};
  std::size_t status_publisher_count_{0U};
  bool odometry_publisher_identity_valid_{false};
  bool status_publisher_identity_valid_{false};

  std::uint64_t received_{0U};
  std::uint64_t accepted_{0U};
  std::uint64_t rejected_{0U};
  std::uint64_t duplicate_{0U};
  std::uint64_t nonmonotonic_{0U};
  std::uint64_t status_received_{0U};
  std::uint64_t status_rejected_{0U};
  std::uint64_t status_duplicate_{0U};
  std::uint64_t status_nonmonotonic_{0U};
  std::uint64_t diagnostic_evidence_received_{0U};
  std::uint64_t diagnostic_evidence_rejected_{0U};
  std::uint64_t diagnostic_duplicate_{0U};
  std::uint64_t diagnostic_nonmonotonic_{0U};
  std::uint64_t clock_domain_mismatch_{0U};
  std::uint64_t odometry_stamp_gap_violation_{0U};
  std::uint64_t odometry_receive_gap_violation_{0U};
  std::uint64_t odometry_rate_violation_{0U};
  std::uint64_t pose_jump_violation_{0U};
  std::uint64_t shadow_processed_{0U};
  std::uint64_t shadow_pose_computed_{0U};
  static constexpr std::uint64_t kPublished = 0U;

  mutable std::mutex mutex_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<
    isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    upstream_diagnostics_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace cuvslam_localization_adapter

#endif  // CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
