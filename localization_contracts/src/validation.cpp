// Copyright 2026 u5-4
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "localization_contracts/validation.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "localization_contracts/errors.hpp"

namespace localization_contracts
{

namespace
{

constexpr long double kNanosecondsPerSecond = 1000000000.0L;

}  // namespace

StampOrder StrictStampGuard::Classify(const std::int64_t stamp_ns) const
{
  if (stamp_ns <= 0) {
    throw NumericContractViolation("timestamp must be positive");
  }
  if (!last_accepted_ns_.has_value() || stamp_ns > last_accepted_ns_.value()) {
    return StampOrder::kAccept;
  }
  if (stamp_ns == last_accepted_ns_.value()) {
    return StampOrder::kDuplicate;
  }
  return StampOrder::kNonmonotonic;
}

void StrictStampGuard::Commit(const std::int64_t stamp_ns)
{
  if (Classify(stamp_ns) != StampOrder::kAccept) {
    throw NumericContractViolation("cannot commit a non-increasing timestamp");
  }
  last_accepted_ns_ = stamp_ns;
}

const std::optional<std::int64_t> & StrictStampGuard::LastAcceptedNanoseconds() const noexcept
{
  return last_accepted_ns_;
}

double AbsoluteStampDifferenceSeconds(
  const std::int64_t first_ns,
  const std::int64_t second_ns)
{
  if (first_ns <= 0 || second_ns <= 0) {
    throw NumericContractViolation("timestamps must be positive");
  }
  const long double delta_ns =
    static_cast<long double>(first_ns) - static_cast<long double>(second_ns);
  return static_cast<double>(std::abs(delta_ns) / kNanosecondsPerSecond);
}

std::optional<StampWindowMetrics> ComputeStampWindowMetrics(
  const std::vector<std::int64_t> & stamps_ns)
{
  if (stamps_ns.size() < 2U) {
    return std::nullopt;
  }
  double maximum_gap_sec = 0.0;
  for (std::size_t index = 0U; index < stamps_ns.size(); ++index) {
    if (stamps_ns[index] <= 0) {
      throw NumericContractViolation("timestamps must be positive");
    }
    if (index == 0U) {
      continue;
    }
    if (stamps_ns[index] <= stamps_ns[index - 1U]) {
      throw NumericContractViolation("timestamp window must be strictly increasing");
    }
    const double gap_sec = static_cast<double>(
      stamps_ns[index] - stamps_ns[index - 1U]) /
      static_cast<double>(kNanosecondsPerSecond);
    maximum_gap_sec = std::max(maximum_gap_sec, gap_sec);
  }
  const double duration_sec = static_cast<double>(
    stamps_ns.back() - stamps_ns.front()) /
    static_cast<double>(kNanosecondsPerSecond);
  return StampWindowMetrics{
    static_cast<double>(stamps_ns.size() - 1U) / duration_sec,
    maximum_gap_sec};
}

PoseStep MeasurePoseStep(
  const RigidTransform & previous,
  const RigidTransform & current)
{
  if (previous.ParentFrame() != current.ParentFrame() ||
    previous.ChildFrame() != current.ChildFrame())
  {
    throw FrameContractViolation("pose step requires identical transform frame roles");
  }
  const double quaternion_dot = std::abs(
    previous.Rotation().dot(current.Rotation()));
  return PoseStep{
    (current.Translation() - previous.Translation()).norm(),
    2.0 * std::acos(std::clamp(quaternion_dot, 0.0, 1.0))};
}

bool CovarianceIsSymmetricPositiveSemidefinite(
  const std::array<double, 36> & covariance,
  const double tolerance)
{
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  Covariance6d matrix;
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      matrix(row, column) = covariance[
        static_cast<std::size_t>(row * 6 + column)];
    }
  }
  if (!matrix.allFinite()) {
    return false;
  }
  if (!matrix.isApprox(matrix.transpose(), tolerance)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<Covariance6d> solver(matrix);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

std::vector<ValidationIssue> ValidateOdometrySample(
  const OdometrySample & sample,
  const std::string & expected_parent_frame,
  const std::string & expected_child_frame)
{
  const std::string parent = ValidateFrameId(expected_parent_frame);
  const std::string child = ValidateFrameId(expected_child_frame);
  std::vector<ValidationIssue> issues;
  if (sample.parent_frame != parent) {
    issues.push_back(
      ValidationIssue{
        "PARENT_FRAME_MISMATCH",
        "expected '" + parent + "', got '" + sample.parent_frame + "'",
        true});
  }
  if (sample.child_frame != child) {
    issues.push_back(
      ValidationIssue{
        "CHILD_FRAME_MISMATCH",
        "expected '" + child + "', got '" + sample.child_frame + "'",
        true});
  }
  if (sample.stamp_ns <= 0) {
    issues.push_back(
      ValidationIssue{
        "INVALID_TIMESTAMP", "timestamp must be positive", false});
  }
  if (!sample.position.allFinite() ||
    !sample.linear_velocity.allFinite() || !sample.angular_velocity.allFinite())
  {
    issues.push_back(
      ValidationIssue{
        "NONFINITE_MEASUREMENT",
        "position and twist must contain only finite values",
        false});
  }
  try {
    (void)QuaternionFromRosXyzw(sample.orientation_xyzw);
  } catch (const NumericContractViolation & error) {
    issues.push_back(ValidationIssue{"INVALID_QUATERNION", error.what(), false});
  }
  if (!CovarianceIsSymmetricPositiveSemidefinite(sample.pose_covariance)) {
    issues.push_back(
      ValidationIssue{
        "INVALID_POSE_COVARIANCE",
        "pose covariance must be finite, symmetric, and positive semidefinite",
        false});
  }
  if (!CovarianceIsSymmetricPositiveSemidefinite(sample.twist_covariance)) {
    issues.push_back(
      ValidationIssue{
        "INVALID_TWIST_COVARIANCE",
        "twist covariance must be finite, symmetric, and positive semidefinite",
        false});
  }
  return issues;
}

}  // namespace localization_contracts
