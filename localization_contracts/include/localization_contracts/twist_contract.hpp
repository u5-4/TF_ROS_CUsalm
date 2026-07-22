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

#ifndef LOCALIZATION_CONTRACTS__TWIST_CONTRACT_HPP_
#define LOCALIZATION_CONTRACTS__TWIST_CONTRACT_HPP_

#include <Eigen/Core>

#include <optional>
#include <string>
#include <utility>

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
