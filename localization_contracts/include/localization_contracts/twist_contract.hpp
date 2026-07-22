// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_CONTRACTS__TWIST_CONTRACT_HPP_
#define LOCALIZATION_CONTRACTS__TWIST_CONTRACT_HPP_

#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "localization_contracts/rigid_transform.hpp"

namespace localization_contracts
{

enum class TwistTemporalModel
{
  kUnknown,
  kWindowPoseDifference,
  kInstantaneousAtHeaderStamp,
};

struct TwistSemantics
{
  std::optional<std::string> reference_point_frame;
  std::optional<std::string> expressed_in_frame;
  TwistTemporalModel temporal_model{TwistTemporalModel::kUnknown};
  std::optional<std::string> approval_id;

  bool Approved() const noexcept;
};

TwistSemantics CuvslamRawUnapproved();

struct SpatialVelocity
{
  SpatialVelocity(
    Eigen::Vector3d linear_value,
    Eigen::Vector3d angular_value,
    std::string reference_point_frame_value,
    std::string expressed_in_frame_value)
  : linear(std::move(linear_value)),
    angular(std::move(angular_value)),
    reference_point_frame(std::move(reference_point_frame_value)),
    expressed_in_frame(std::move(expressed_in_frame_value)) {}

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  std::string reference_point_frame;
  std::string expressed_in_frame;
};

SpatialVelocity InstantaneousCameraTwistToBase(
  const SpatialVelocity & camera_velocity,
  const RigidTransform & base_from_camera,
  const TwistSemantics & semantics);

}  // namespace localization_contracts

#endif  // LOCALIZATION_CONTRACTS__TWIST_CONTRACT_HPP_
