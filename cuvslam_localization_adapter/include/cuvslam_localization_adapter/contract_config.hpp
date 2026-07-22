// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef CUVSLAM_LOCALIZATION_ADAPTER__CONTRACT_CONFIG_HPP_
#define CUVSLAM_LOCALIZATION_ADAPTER__CONTRACT_CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "localization_contracts/gate.hpp"
#include "localization_contracts/rigid_transform.hpp"

namespace cuvslam_localization_adapter
{

struct RequiredDiagnosticConfig
{
  std::string name;
  std::string hardware_id;
  std::string expected_publisher;
  std::uint8_t maximum_level;
  std::map<std::string, std::string> required_values;
};

struct InputContractConfig
{
  std::string odometry_topic;
  std::string visual_slam_status_topic;
  std::string diagnostics_topic;
  std::string expected_odometry_publisher;
  std::string expected_status_publisher;
  std::string expected_parent_frame;
  std::string expected_child_frame;
  std::string expected_status_frame;
  std::vector<std::uint8_t> accepted_vo_states;
  std::vector<RequiredDiagnosticConfig> required_diagnostics;
};

struct HealthContractConfig
{
  double diagnostic_period_sec;
  double odometry_stale_after_sec;
  double tracking_stale_after_sec;
  double diagnostic_stale_after_sec;
  double startup_timeout_sec;
  std::size_t recovery_consecutive_samples;
  double minimum_odometry_rate_hz;
  double maximum_odometry_stamp_gap_sec;
  double maximum_odometry_receive_gap_sec;
  double maximum_clock_residual_sec;
  double maximum_status_odometry_skew_sec;
  double maximum_position_step_m;
  double maximum_rotation_step_rad;
  std::size_t odometry_rate_window_samples;
};

struct EvidenceGateConfig
{
  std::string status;
  std::string policy;
  std::optional<std::string> approval_id;
  std::string reason;

  bool Approved() const noexcept;
};

struct ExtrinsicGateConfig
{
  std::string status;
  std::string parent_frame;
  std::string child_frame;
  std::optional<localization_contracts::RigidTransform> transform;
  std::optional<std::string> provenance;
  std::string reason;

  bool Approved() const noexcept;
};

struct ContractConfig
{
  int schema_version;
  std::string contract_id;
  std::string status;
  std::string source_sha256;
  InputContractConfig input;
  HealthContractConfig health;
  ExtrinsicGateConfig extrinsic;
  EvidenceGateConfig twist;
  EvidenceGateConfig covariance;
  localization_contracts::AuthorizationState authorization;
  std::string output_topic;
  std::string output_parent_frame;
  std::string output_child_frame;

  localization_contracts::ContractApprovals Approvals() const noexcept;
};

std::string CalculateFileSha256(const std::string & path);
ContractConfig LoadContractConfig(const std::string & path);
std::optional<std::string> CheckDiagnosticEvidence(
  const RequiredDiagnosticConfig & required,
  std::uint8_t actual_level,
  const std::string & actual_hardware_id,
  const std::vector<std::pair<std::string, std::string>> & actual_values);

}  // namespace cuvslam_localization_adapter

#endif  // CUVSLAM_LOCALIZATION_ADAPTER__CONTRACT_CONFIG_HPP_
