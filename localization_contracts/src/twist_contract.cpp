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

#include "localization_contracts/twist_contract.hpp"

#include "localization_contracts/errors.hpp"

namespace localization_contracts
{

bool TwistSemantics::Approved() const noexcept
{
  return approval_id.has_value() && !approval_id->empty();
}

TwistSemantics CuvslamRawUnapproved()
{
  return TwistSemantics{
    std::nullopt,
    std::nullopt,
    TwistTemporalModel::kWindowPoseDifference,
    std::nullopt};
}

SpatialVelocity InstantaneousCameraTwistToBase(
  const SpatialVelocity & camera_velocity,
  const RigidTransform & base_from_camera,
  const TwistSemantics & semantics)
{
  const std::string & camera_frame = base_from_camera.ChildFrame();
  const std::string & base_frame = base_from_camera.ParentFrame();
  if (!semantics.Approved()) {
    throw UnapprovedTwistSemantics(
            "TWIST_SEMANTICS_UNAPPROVED: approval evidence is required");
  }
  if (!semantics.reference_point_frame.has_value() ||
    semantics.reference_point_frame.value() != camera_frame)
  {
    throw UnapprovedTwistSemantics(
            "TWIST_REFERENCE_POINT_MISMATCH: camera origin is required");
  }
  if (!semantics.expressed_in_frame.has_value() ||
    semantics.expressed_in_frame.value() != camera_frame)
  {
    throw UnapprovedTwistSemantics(
            "TWIST_EXPRESSION_FRAME_MISMATCH: camera expression is required");
  }
  if (semantics.temporal_model != TwistTemporalModel::kInstantaneousAtHeaderStamp) {
    throw UnapprovedTwistSemantics(
            "TWIST_TEMPORAL_MODEL_MISMATCH: instantaneous header-stamp twist is required");
  }
  if (ValidateFrameId(camera_velocity.reference_point_frame) != camera_frame ||
    ValidateFrameId(camera_velocity.expressed_in_frame) != camera_frame)
  {
    throw FrameContractViolation("velocity frames do not match the approved semantics");
  }
  if (!camera_velocity.linear.allFinite() || !camera_velocity.angular.allFinite()) {
    throw NumericContractViolation("twist must contain only finite values");
  }

  const Eigen::Vector3d angular_in_base =
    base_from_camera.Rotation() * camera_velocity.angular;
  const Eigen::Vector3d camera_linear_in_base =
    base_from_camera.Rotation() * camera_velocity.linear;
  const Eigen::Vector3d base_linear =
    camera_linear_in_base - angular_in_base.cross(base_from_camera.Translation());
  return SpatialVelocity{
    base_linear,
    angular_in_base,
    base_frame,
    base_frame};
}

}  // namespace localization_contracts
