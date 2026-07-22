// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cuvslam_localization_adapter/contract_config.hpp"
#include "localization_contracts/gate.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

std::string Fixture(const std::string & name)
{
  return std::string(TEST_FIXTURE_DIR) + "/" + name;
}

TEST(ContractConfig, LoadsSyntheticExtrinsicInShadowOnlyContract)
{
  const ContractConfig contract =
    LoadContractConfig(Fixture("approved_synthetic_contract.yaml"));
  EXPECT_EQ(contract.status, "blocked");
  EXPECT_TRUE(contract.extrinsic.Approved());
  EXPECT_FALSE(contract.twist.Approved());
  EXPECT_FALSE(contract.covariance.Approved());
  EXPECT_EQ(
    contract.authorization,
    localization_contracts::AuthorizationState::kShadowOnly);
  EXPECT_EQ(contract.output_topic, "/localization/odometry");
  EXPECT_EQ(contract.output_parent_frame, "odom");
  EXPECT_EQ(contract.output_child_frame, "base_link");
  ASSERT_EQ(contract.input.required_diagnostics.size(), 1U);
  EXPECT_EQ(
    contract.input.required_diagnostics.front().hardware_id,
    "synthetic-hardware");
  EXPECT_EQ(
    contract.input.required_diagnostics.front().expected_publisher,
    "/synthetic_diagnostic_source");
  EXPECT_EQ(contract.source_sha256.size(), 64U);
}

TEST(ContractConfig, RejectsBlockedPlaceholderExtrinsic)
{
  try {
    (void)LoadContractConfig(Fixture("blocked_placeholder_contract.yaml"));
    FAIL() << "blocked placeholder extrinsic was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(
      std::string(error.what()).find("placeholder transform"),
      std::string::npos);
  }
}

TEST(ContractConfig, RejectsDuplicateYamlKeys)
{
  try {
    (void)LoadContractConfig(Fixture("duplicate_key_contract.yaml"));
    FAIL() << "duplicate YAML key was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("duplicate"), std::string::npos);
  }
}

TEST(ContractConfig, ChecksDiagnosticIdentityValuesAndDuplicateKeys)
{
  const ContractConfig contract =
    LoadContractConfig(Fixture("approved_synthetic_contract.yaml"));
  const auto & required = contract.input.required_diagnostics.front();
  const std::vector<std::pair<std::string, std::string>> valid{
    {"state", "ready"},
    {"extra", "allowed"}};
  EXPECT_FALSE(CheckDiagnosticEvidence(
      required, 0U, "synthetic-hardware", valid).has_value());

  EXPECT_EQ(
    CheckDiagnosticEvidence(required, 1U, "synthetic-hardware", valid).value(),
    "UPSTREAM_DIAGNOSTIC_IDENTITY_OR_LEVEL_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(required, 0U, "wrong-hardware", valid).value(),
    "UPSTREAM_DIAGNOSTIC_IDENTITY_OR_LEVEL_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(
      required, 0U, "synthetic-hardware", {{"state", "not-ready"}}).value(),
    "UPSTREAM_DIAGNOSTIC_VALUE_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(
      required,
      0U,
      "synthetic-hardware",
      {{"state", "ready"}, {"state", "ready"}}).value(),
    "UPSTREAM_DIAGNOSTIC_DUPLICATE_VALUE_KEY");
}

}  // namespace
}  // namespace cuvslam_localization_adapter
