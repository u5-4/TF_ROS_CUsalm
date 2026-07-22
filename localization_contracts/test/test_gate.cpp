// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "localization_contracts/gate.hpp"

namespace localization_contracts
{
namespace
{

TEST(PublishGate, ShadowDefaultReportsEveryBlocker)
{
  const auto decision = EvaluateLocalizationPublishGate(
    "shadow",
    HealthState::kStarting,
    AuthorizationState::kShadowOnly,
    ContractApprovals{false, false, false});
  EXPECT_FALSE(decision.publish);
  const std::vector<std::string> expected{
    "HEALTH_NOT_OK",
    "AUTHORIZATION_SHADOW_ONLY",
    "EXTRINSIC_UNAPPROVED",
    "TWIST_SEMANTICS_UNAPPROVED",
    "COVARIANCE_UNAPPROVED"};
  EXPECT_EQ(decision.reasons, expected);
}

TEST(PublishGate, StageOneCannotPublishEvenWithSyntheticApprovals)
{
  const auto decision = EvaluateLocalizationPublishGate(
    "shadow",
    HealthState::kHealthy,
    AuthorizationState::kShadowOnly,
    ContractApprovals{true, true, true});
  EXPECT_FALSE(decision.publish);
  const std::vector<std::string> expected{"AUTHORIZATION_SHADOW_ONLY"};
  EXPECT_EQ(decision.reasons, expected);
}

TEST(PublishGate, RejectsNonShadowModeBeforeAuthorization)
{
  const auto decision = EvaluateLocalizationPublishGate(
    "passive",
    HealthState::kHealthy,
    AuthorizationState::kShadowOnly,
    ContractApprovals{true, true, true});
  EXPECT_FALSE(decision.publish);
  const std::vector<std::string> expected{
    "MODE_NOT_SHADOW", "AUTHORIZATION_SHADOW_ONLY"};
  EXPECT_EQ(decision.reasons, expected);
}

TEST(RuntimeHealthGate, TransientFaultRequiresConsecutiveRecovery)
{
  RuntimeHealthGate gate(3U);
  gate.ObserveHealthySample();
  EXPECT_EQ(gate.State(), HealthState::kHealthy);
  gate.MarkTransient("ODOMETRY_STALE");
  gate.ObserveHealthySample();
  gate.ObserveHealthySample();
  EXPECT_EQ(gate.State(), HealthState::kRecovering);
  EXPECT_EQ(gate.RecoveryProgress(), 2U);
  gate.ObserveHealthySample();
  EXPECT_EQ(gate.State(), HealthState::kHealthy);
}

TEST(RuntimeHealthGate, LatchedFaultCannotBeCleared)
{
  RuntimeHealthGate gate(2U);
  gate.MarkLatched("FRAME_MISMATCH");
  gate.MarkTransient("STALE");
  gate.ObserveHealthySample();
  gate.MarkStarting("WAITING");
  EXPECT_EQ(gate.State(), HealthState::kLatchedFault);
  EXPECT_EQ(gate.Reason(), "FRAME_MISMATCH");
}

TEST(RuntimeHealthGate, StartingMarkerCannotBypassRecovery)
{
  RuntimeHealthGate gate(2U);
  gate.ObserveHealthySample();
  gate.MarkTransient("TEMPORARY_FAULT");
  gate.MarkStarting("WAITING_AGAIN");
  EXPECT_EQ(gate.State(), HealthState::kTransientFault);
  EXPECT_EQ(gate.Reason(), "TEMPORARY_FAULT");
}

TEST(RuntimeHealthGate, RejectsZeroRecoveryThreshold)
{
  EXPECT_THROW(RuntimeHealthGate(0U), std::invalid_argument);
}

}  // namespace
}  // namespace localization_contracts
