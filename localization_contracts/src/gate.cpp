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

#include "localization_contracts/gate.hpp"

#include <stdexcept>
#include <utility>

namespace localization_contracts
{

GateDecision EvaluateLocalizationPublishGate(
  const std::string & mode,
  const HealthState health,
  const AuthorizationState authorization,
  const ContractApprovals & approvals)
{
  std::vector<std::string> reasons;
  if (mode != "shadow") {
    reasons.emplace_back("MODE_NOT_SHADOW");
  }
  if (health != HealthState::kHealthy) {
    reasons.emplace_back("HEALTH_NOT_OK");
  }
  (void)authorization;
  reasons.emplace_back("AUTHORIZATION_SHADOW_ONLY");
  if (!approvals.extrinsic) {
    reasons.emplace_back("EXTRINSIC_UNAPPROVED");
  }
  if (!approvals.twist) {
    reasons.emplace_back("TWIST_SEMANTICS_UNAPPROVED");
  }
  if (!approvals.covariance) {
    reasons.emplace_back("COVARIANCE_UNAPPROVED");
  }
  return GateDecision{reasons.empty(), std::move(reasons)};
}

RuntimeHealthGate::RuntimeHealthGate(const std::size_t recovery_consecutive_samples)
: recovery_required_(recovery_consecutive_samples)
{
  if (recovery_required_ == 0U) {
    throw std::invalid_argument("recovery_consecutive_samples must be positive");
  }
}

void RuntimeHealthGate::MarkStarting(const std::string & reason)
{
  if (state_ != HealthState::kStarting) {
    return;
  }
  reason_ = reason;
  recovery_progress_ = 0U;
}

void RuntimeHealthGate::MarkTransient(const std::string & reason)
{
  if (state_ == HealthState::kLatchedFault) {
    return;
  }
  state_ = HealthState::kTransientFault;
  reason_ = reason;
  recovery_progress_ = 0U;
}

void RuntimeHealthGate::MarkLatched(const std::string & reason)
{
  state_ = HealthState::kLatchedFault;
  reason_ = reason;
  recovery_progress_ = 0U;
}

void RuntimeHealthGate::ObserveHealthySample()
{
  if (state_ == HealthState::kLatchedFault) {
    return;
  }
  if (state_ == HealthState::kStarting) {
    state_ = HealthState::kHealthy;
    reason_ = "INPUT_HEALTHY";
    return;
  }
  if (state_ == HealthState::kTransientFault || state_ == HealthState::kRecovering) {
    state_ = HealthState::kRecovering;
    ++recovery_progress_;
    reason_ = "RECOVERY_IN_PROGRESS";
    if (recovery_progress_ >= recovery_required_) {
      state_ = HealthState::kHealthy;
      reason_ = "INPUT_HEALTHY";
      recovery_progress_ = 0U;
    }
  }
}

HealthState RuntimeHealthGate::State() const noexcept
{
  return state_;
}

const std::string & RuntimeHealthGate::Reason() const noexcept
{
  return reason_;
}

std::size_t RuntimeHealthGate::RecoveryProgress() const noexcept
{
  return recovery_progress_;
}

std::size_t RuntimeHealthGate::RecoveryRequired() const noexcept
{
  return recovery_required_;
}

std::string ToString(const HealthState state)
{
  switch (state) {
    case HealthState::kStarting:
      return "starting";
    case HealthState::kHealthy:
      return "healthy";
    case HealthState::kTransientFault:
      return "transient_fault";
    case HealthState::kRecovering:
      return "recovering";
    case HealthState::kLatchedFault:
      return "latched_fault";
  }
  throw std::logic_error("unhandled health state");
}

std::string ToString(const AuthorizationState state)
{
  switch (state) {
    case AuthorizationState::kShadowOnly:
      return "shadow_only";
  }
  throw std::logic_error("unhandled authorization state");
}

}  // namespace localization_contracts
