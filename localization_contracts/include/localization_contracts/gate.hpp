// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_CONTRACTS__GATE_HPP_
#define LOCALIZATION_CONTRACTS__GATE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace localization_contracts
{

enum class HealthState
{
  kStarting,
  kHealthy,
  kTransientFault,
  kRecovering,
  kLatchedFault,
};

enum class AuthorizationState
{
  kShadowOnly,
};

struct ContractApprovals
{
  bool extrinsic{false};
  bool twist{false};
  bool covariance{false};
};

struct GateDecision
{
  bool publish;
  std::vector<std::string> reasons;
};

GateDecision EvaluateLocalizationPublishGate(
  const std::string & mode,
  HealthState health,
  AuthorizationState authorization,
  const ContractApprovals & approvals);

class RuntimeHealthGate
{
public:
  explicit RuntimeHealthGate(std::size_t recovery_consecutive_samples);

  void MarkStarting(const std::string & reason);
  void MarkTransient(const std::string & reason);
  void MarkLatched(const std::string & reason);
  void ObserveHealthySample();

  HealthState State() const noexcept;
  const std::string & Reason() const noexcept;
  std::size_t RecoveryProgress() const noexcept;
  std::size_t RecoveryRequired() const noexcept;

private:
  std::size_t recovery_required_;
  HealthState state_{HealthState::kStarting};
  std::string reason_{"WAITING_FOR_INPUT"};
  std::size_t recovery_progress_{0};
};

std::string ToString(HealthState state);
std::string ToString(AuthorizationState state);

}  // namespace localization_contracts

#endif  // LOCALIZATION_CONTRACTS__GATE_HPP_
