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

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/twist_contract.hpp"

namespace localization_contracts
{
namespace
{

TEST(TwistContract, RawCuvslamWindowVelocityIsAlwaysRejected)
{
  const double sine = std::sqrt(0.5);
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d(0.2, 0.0, 0.0),
    QuaternionXyzw{0.0, 0.0, sine, sine});
  SpatialVelocity velocity{
    Eigen::Vector3d(2.4, -1.0, 3.0),
    Eigen::Vector3d(0.0, 0.0, 2.0),
    "camera_link",
    "camera_link"};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(
      velocity, base_from_camera, CuvslamRawUnapproved()),
    UnapprovedTwistSemantics);

  velocity.linear.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(
      velocity, base_from_camera, CuvslamRawUnapproved()),
    UnapprovedTwistSemantics);
}

TEST(TwistContract, ApprovedSyntheticTwistIncludesLeverArm)
{
  const double sine = std::sqrt(0.5);
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d(0.2, 0.0, 0.0),
    QuaternionXyzw{0.0, 0.0, sine, sine});
  const SpatialVelocity velocity{
    Eigen::Vector3d(2.4, -1.0, 3.0),
    Eigen::Vector3d(0.0, 0.0, 2.0),
    "camera_link",
    "camera_link"};
  const TwistSemantics semantics{
    std::string("camera_link"),
    std::string("camera_link"),
    TwistTemporalModel::kInstantaneousAtHeaderStamp,
    std::string("unit-test/synthetic-instantaneous-v1")};

  const SpatialVelocity output =
    InstantaneousCameraTwistToBase(velocity, base_from_camera, semantics);
  EXPECT_TRUE(output.linear.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1.0e-9));
  EXPECT_TRUE(output.angular.isApprox(Eigen::Vector3d(0.0, 0.0, 2.0), 1.0e-9));
  EXPECT_EQ(output.reference_point_frame, "base_link");
  EXPECT_EQ(output.expressed_in_frame, "base_link");
}

TEST(TwistContract, ApprovedButWrongTemporalModelIsRejected)
{
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const SpatialVelocity velocity{
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    "camera_link", "camera_link"};
  const TwistSemantics semantics{
    std::string("camera_link"),
    std::string("camera_link"),
    TwistTemporalModel::kWindowPoseDifference,
    std::string("not-valid-for-this-function")};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(velocity, base_from_camera, semantics),
    UnapprovedTwistSemantics);
}

TEST(TwistContract, RejectsMissingApprovalAndWrongSpatialSemantics)
{
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const SpatialVelocity velocity{
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    "camera_link", "camera_link"};
  const TwistSemantics missing_approval{
    std::string("camera_link"),
    std::string("camera_link"),
    TwistTemporalModel::kInstantaneousAtHeaderStamp,
    std::nullopt};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(
      velocity, base_from_camera, missing_approval),
    UnapprovedTwistSemantics);

  const TwistSemantics wrong_reference{
    std::string("base_link"),
    std::string("camera_link"),
    TwistTemporalModel::kInstantaneousAtHeaderStamp,
    std::string("synthetic")};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(
      velocity, base_from_camera, wrong_reference),
    UnapprovedTwistSemantics);

  const TwistSemantics wrong_expression{
    std::string("camera_link"),
    std::string("base_link"),
    TwistTemporalModel::kInstantaneousAtHeaderStamp,
    std::string("synthetic")};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(
      velocity, base_from_camera, wrong_expression),
    UnapprovedTwistSemantics);
}

TEST(TwistContract, RejectsNonfiniteApprovedSyntheticTwist)
{
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const SpatialVelocity velocity{
    Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    "camera_link",
    "camera_link"};
  const TwistSemantics semantics{
    std::string("camera_link"),
    std::string("camera_link"),
    TwistTemporalModel::kInstantaneousAtHeaderStamp,
    std::string("synthetic")};
  EXPECT_THROW(
    InstantaneousCameraTwistToBase(velocity, base_from_camera, semantics),
    NumericContractViolation);
}

}  // namespace
}  // namespace localization_contracts
