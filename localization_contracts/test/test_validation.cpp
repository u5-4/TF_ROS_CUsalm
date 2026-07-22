// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/validation.hpp"

namespace localization_contracts
{
namespace
{

std::array<double, 36> DiagonalCovariance(const double value = 1.0)
{
  std::array<double, 36> covariance{};
  for (std::size_t index = 0; index < 6U; ++index) {
    covariance[index * 6U + index] = value;
  }
  return covariance;
}

OdometrySample ValidSample()
{
  return OdometrySample{
    "odom",
    "camera_link",
    100,
    Eigen::Vector3d(1.0, 2.0, 3.0),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0},
    Eigen::Vector3d(0.1, 0.2, 0.3),
    Eigen::Vector3d(0.01, 0.02, 0.03),
    DiagonalCovariance(),
    DiagonalCovariance()};
}

TEST(Validation, AcceptsPositiveSemidefiniteAndZeroCovariance)
{
  EXPECT_TRUE(CovarianceIsSymmetricPositiveSemidefinite(DiagonalCovariance()));
  EXPECT_TRUE(CovarianceIsSymmetricPositiveSemidefinite(std::array<double, 36>{}));
  EXPECT_TRUE(ValidateOdometrySample(
      ValidSample(), "odom", "camera_link").empty());
}

TEST(Validation, RejectsAsymmetricNegativeAndNonfiniteCovariance)
{
  auto asymmetric = DiagonalCovariance();
  asymmetric[1] = 1.0;
  EXPECT_FALSE(CovarianceIsSymmetricPositiveSemidefinite(asymmetric));
  auto negative = DiagonalCovariance();
  negative[0] = -1.0;
  EXPECT_FALSE(CovarianceIsSymmetricPositiveSemidefinite(negative));
  auto nonfinite = DiagonalCovariance();
  nonfinite[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(CovarianceIsSymmetricPositiveSemidefinite(nonfinite));
  EXPECT_FALSE(CovarianceIsSymmetricPositiveSemidefinite(
      DiagonalCovariance(), -1.0));
  EXPECT_FALSE(CovarianceIsSymmetricPositiveSemidefinite(
      DiagonalCovariance(), std::numeric_limits<double>::infinity()));
}

TEST(Validation, RejectsNonfinitePoseAndTwistComponents)
{
  auto sample = ValidSample();
  sample.position.x() = std::numeric_limits<double>::quiet_NaN();
  sample.angular_velocity.z() = std::numeric_limits<double>::infinity();
  const auto issues = ValidateOdometrySample(sample, "odom", "camera_link");
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, "NONFINITE_MEASUREMENT");
}

TEST(Validation, ReportsLatchedFrameErrorsBeforeTransientQuaternionError)
{
  auto sample = ValidSample();
  sample.parent_frame = "map";
  sample.child_frame = "base_link";
  sample.orientation_xyzw = QuaternionXyzw{0.0, 0.0, 0.0, 0.0};
  const auto issues = ValidateOdometrySample(sample, "odom", "camera_link");
  ASSERT_EQ(issues.size(), 3U);
  EXPECT_EQ(issues[0].code, "PARENT_FRAME_MISMATCH");
  EXPECT_TRUE(issues[0].latched);
  EXPECT_EQ(issues[1].code, "CHILD_FRAME_MISMATCH");
  EXPECT_TRUE(issues[1].latched);
  EXPECT_EQ(issues[2].code, "INVALID_QUATERNION");
  EXPECT_FALSE(issues[2].latched);
}

TEST(Validation, StrictStampGuardDoesNotAdvanceOnRejectedSamples)
{
  StrictStampGuard guard;
  EXPECT_EQ(guard.Classify(100), StampOrder::kAccept);
  guard.Commit(100);
  EXPECT_EQ(guard.Classify(100), StampOrder::kDuplicate);
  EXPECT_EQ(guard.Classify(99), StampOrder::kNonmonotonic);
  ASSERT_TRUE(guard.LastAcceptedNanoseconds().has_value());
  EXPECT_EQ(guard.LastAcceptedNanoseconds().value(), 100);
  guard.Commit(101);
  EXPECT_EQ(guard.LastAcceptedNanoseconds().value(), 101);
  EXPECT_THROW(guard.Classify(0), NumericContractViolation);
  EXPECT_THROW(guard.Commit(101), NumericContractViolation);
  EXPECT_THROW(guard.Commit(100), NumericContractViolation);
}

TEST(Validation, SourceSequenceRejectsCorrectedEvidenceAtSameStamp)
{
  StrictStampGuard source_sequence;
  EXPECT_EQ(source_sequence.Classify(500), StampOrder::kAccept);
  source_sequence.Commit(500);
  EXPECT_EQ(source_sequence.Classify(500), StampOrder::kDuplicate);
}

TEST(Validation, ComputesStrictTimestampWindowMetrics)
{
  const std::vector<std::int64_t> stamps{
    1000000000LL,
    1010000000LL,
    1030000000LL};
  const auto metrics = ComputeStampWindowMetrics(stamps);
  ASSERT_TRUE(metrics.has_value());
  EXPECT_NEAR(metrics->rate_hz, 66.6666666667, 1.0e-9);
  EXPECT_NEAR(metrics->maximum_gap_sec, 0.02, 1.0e-12);
  EXPECT_NEAR(
    AbsoluteStampDifferenceSeconds(stamps.front(), stamps.back()),
    0.03,
    1.0e-12);

  EXPECT_FALSE(ComputeStampWindowMetrics({1000000000LL}).has_value());
  EXPECT_THROW(
    ComputeStampWindowMetrics({1000000000LL, 1000000000LL}),
    NumericContractViolation);
  EXPECT_THROW(
    ComputeStampWindowMetrics({0LL, 1000000000LL}),
    NumericContractViolation);
  EXPECT_THROW(
    AbsoluteStampDifferenceSeconds(0LL, 1LL),
    NumericContractViolation);
}

TEST(Validation, MeasuresPoseStepWithQuaternionSignInvariance)
{
  const double sine = std::sqrt(0.5);
  const auto previous = RigidTransform::Create(
    "odom", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto current = RigidTransform::Create(
    "odom", "camera_link", Eigen::Vector3d(0.3, 0.4, 0.0),
    QuaternionXyzw{0.0, 0.0, -sine, -sine});
  const PoseStep step = MeasurePoseStep(previous, current);
  EXPECT_NEAR(step.translation_m, 0.5, 1.0e-12);
  EXPECT_NEAR(step.rotation_rad, std::acos(-1.0) / 2.0, 1.0e-12);

  const auto wrong_frame = RigidTransform::Create(
    "map", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(
    MeasurePoseStep(previous, wrong_frame),
    FrameContractViolation);
}

}  // namespace
}  // namespace localization_contracts
