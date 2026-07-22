// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_CONTRACTS__RIGID_TRANSFORM_HPP_
#define LOCALIZATION_CONTRACTS__RIGID_TRANSFORM_HPP_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace localization_contracts
{

struct QuaternionXyzw
{
  constexpr QuaternionXyzw(
    const double x_value,
    const double y_value,
    const double z_value,
    const double w_value) noexcept
  : x(x_value), y(y_value), z(z_value), w(w_value) {}

  double x;
  double y;
  double z;
  double w;
};

std::string ValidateFrameId(const std::string & frame_id);
Eigen::Quaterniond QuaternionFromRosXyzw(const QuaternionXyzw & rotation_xyzw);
QuaternionXyzw QuaternionToRosXyzw(const Eigen::Quaterniond & rotation);

class RigidTransform
{
public:
  static RigidTransform Create(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Vector3d & translation,
    const QuaternionXyzw & rotation_xyzw);

  static RigidTransform Identity(const std::string & frame);

  RigidTransform Compose(const RigidTransform & right) const;
  RigidTransform Inverse() const;
  Eigen::Vector3d ApplyPoint(const Eigen::Vector3d & point_in_child) const;
  bool IsEquivalent(
    const RigidTransform & other,
    double translation_tolerance = 1.0e-9,
    double angular_tolerance_rad = 1.0e-9) const;

  const std::string & ParentFrame() const noexcept;
  const std::string & ChildFrame() const noexcept;
  const Eigen::Vector3d & Translation() const noexcept;
  const Eigen::Quaterniond & Rotation() const noexcept;
  QuaternionXyzw RotationXyzw() const;

private:
  RigidTransform(
    std::string parent_frame,
    std::string child_frame,
    Eigen::Vector3d translation,
    Eigen::Quaterniond rotation);

  std::string parent_frame_;
  std::string child_frame_;
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_;
};

RigidTransform CameraPoseToBasePose(
  const RigidTransform & odom_from_camera,
  const RigidTransform & base_from_camera);

}  // namespace localization_contracts

#endif  // LOCALIZATION_CONTRACTS__RIGID_TRANSFORM_HPP_
