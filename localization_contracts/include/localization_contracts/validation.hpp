// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_CONTRACTS__VALIDATION_HPP_
#define LOCALIZATION_CONTRACTS__VALIDATION_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "localization_contracts/rigid_transform.hpp"

namespace localization_contracts
{

using Covariance6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

enum class StampOrder
{
  kAccept,
  kDuplicate,
  kNonmonotonic,
};

class StrictStampGuard
{
public:
  StampOrder Classify(std::int64_t stamp_ns) const;
  void Commit(std::int64_t stamp_ns);
  const std::optional<std::int64_t> & LastAcceptedNanoseconds() const noexcept;

private:
  std::optional<std::int64_t> last_accepted_ns_;
};

struct OdometrySample
{
  std::string parent_frame;
  std::string child_frame;
  std::int64_t stamp_ns;
  Eigen::Vector3d position;
  QuaternionXyzw orientation_xyzw;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
  std::array<double, 36> pose_covariance;
  std::array<double, 36> twist_covariance;
};

struct ValidationIssue
{
  std::string code;
  std::string detail;
  bool latched;
};

struct StampWindowMetrics
{
  double rate_hz;
  double maximum_gap_sec;
};

struct PoseStep
{
  double translation_m;
  double rotation_rad;
};

double AbsoluteStampDifferenceSeconds(std::int64_t first_ns, std::int64_t second_ns);
std::optional<StampWindowMetrics> ComputeStampWindowMetrics(
  const std::vector<std::int64_t> & stamps_ns);
PoseStep MeasurePoseStep(
  const RigidTransform & previous,
  const RigidTransform & current);

bool CovarianceIsSymmetricPositiveSemidefinite(
  const std::array<double, 36> & covariance,
  double tolerance = 1.0e-10);

std::vector<ValidationIssue> ValidateOdometrySample(
  const OdometrySample & sample,
  const std::string & expected_parent_frame,
  const std::string & expected_child_frame);

}  // namespace localization_contracts

#endif  // LOCALIZATION_CONTRACTS__VALIDATION_HPP_
