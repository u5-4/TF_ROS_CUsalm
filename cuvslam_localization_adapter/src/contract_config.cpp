// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include "cuvslam_localization_adapter/contract_config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/rigid_transform.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

using localization_contracts::AuthorizationState;
using localization_contracts::ContractViolation;
using localization_contracts::QuaternionXyzw;
using localization_contracts::RigidTransform;

constexpr double kDegreesToRadians = 0.017453292519943295769;
constexpr const char * kOwnDiagnosticSuffix = ": localization contract";

std::string ReadFile(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open localization contract: " + path);
  }
  return std::string(
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>());
}

std::string CalculateSha256(const std::string & content)
{
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  if (SHA256(
      reinterpret_cast<const unsigned char *>(content.data()),
      content.size(), digest.data()) == nullptr)
  {
    throw std::runtime_error("OpenSSL failed to calculate contract SHA-256");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const unsigned char value : digest) {
    output << std::setw(2) << static_cast<unsigned int>(value);
  }
  return output.str();
}

void RequireMapping(const YAML::Node & node, const std::string & label)
{
  if (!node.IsMap()) {
    throw std::runtime_error(label + " must be a mapping");
  }
}

void RequireExactKeys(
  const YAML::Node & node,
  const std::set<std::string> & expected,
  const std::string & label)
{
  RequireMapping(node, label);
  std::set<std::string> actual;
  for (const auto & item : node) {
    if (!item.first.IsScalar()) {
      throw std::runtime_error(label + " contains a non-scalar key");
    }
    const std::string key = item.first.as<std::string>();
    if (!actual.insert(key).second) {
      throw std::runtime_error(label + " contains duplicate key '" + key + "'");
    }
  }
  if (actual != expected) {
    std::ostringstream message;
    message << label << " keys mismatch; expected={";
    bool first = true;
    for (const auto & key : expected) {
      message << (first ? "" : ",") << key;
      first = false;
    }
    message << "}, actual={";
    first = true;
    for (const auto & key : actual) {
      message << (first ? "" : ",") << key;
      first = false;
    }
    message << "}";
    throw std::runtime_error(message.str());
  }
}

std::string NonemptyString(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be a scalar string");
  }
  const std::string value = node.as<std::string>();
  if (value.empty() || value.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::runtime_error(label + " must be non-empty");
  }
  if (value.front() == ' ' || value.back() == ' ') {
    throw std::runtime_error(label + " must not contain surrounding spaces");
  }
  return value;
}

std::optional<std::string> OptionalString(
  const YAML::Node & node,
  const std::string & label)
{
  if (!node || node.IsNull()) {
    return std::nullopt;
  }
  return NonemptyString(node, label);
}

bool EndsWith(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::map<std::string, std::string> StringMap(
  const YAML::Node & node,
  const std::string & label)
{
  RequireMapping(node, label);
  if (node.size() == 0U) {
    throw std::runtime_error(label + " must not be empty");
  }
  std::map<std::string, std::string> result;
  for (const auto & item : node) {
    const std::string key = NonemptyString(item.first, label + " key");
    const std::string value = NonemptyString(item.second, label + "." + key);
    if (!result.emplace(key, value).second) {
      throw std::runtime_error(label + " contains duplicate key '" + key + "'");
    }
  }
  return result;
}

std::string Status(const YAML::Node & node, const std::string & label)
{
  const std::string value = NonemptyString(node, label);
  if (value != "blocked" && value != "approved") {
    throw std::runtime_error(label + " must be 'blocked' or 'approved'");
  }
  return value;
}

std::string AbsoluteTopic(const YAML::Node & node, const std::string & label)
{
  const std::string value = NonemptyString(node, label);
  if (value.size() < 2U || value.front() != '/' || value.find("//") != std::string::npos) {
    throw std::runtime_error(label + " must be a non-root absolute ROS topic");
  }
  if (std::any_of(value.begin(), value.end(), [](const unsigned char character) {
      return std::isspace(character) != 0;
    }))
  {
    throw std::runtime_error(label + " must not contain whitespace");
  }
  return value;
}

std::string AbsoluteNodeName(const YAML::Node & node, const std::string & label)
{
  const std::string value = NonemptyString(node, label);
  if (value.size() < 2U || value.front() != '/' || value.find("//") != std::string::npos) {
    throw std::runtime_error(label + " must be a non-root fully qualified ROS node name");
  }
  if (std::any_of(value.begin(), value.end(), [](const unsigned char character) {
      return std::isspace(character) != 0;
    }))
  {
    throw std::runtime_error(label + " must not contain whitespace");
  }
  return value;
}

double PositiveDouble(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be numeric");
  }
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(label + " must be finite and positive");
  }
  return value;
}

std::size_t PositiveSize(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be a positive integer");
  }
  const std::int64_t value = node.as<std::int64_t>();
  if (value <= 0) {
    throw std::runtime_error(label + " must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

Eigen::Vector3d Vector3(const YAML::Node & node, const std::string & label)
{
  if (!node.IsSequence() || node.size() != 3U) {
    throw std::runtime_error(label + " must contain exactly three values");
  }
  const Eigen::Vector3d value(
    node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::runtime_error(label + " must contain only finite values");
  }
  return value;
}

QuaternionXyzw Quaternion(const YAML::Node & node, const std::string & label)
{
  if (!node.IsSequence() || node.size() != 4U) {
    throw std::runtime_error(label + " must contain exactly four xyzw values");
  }
  return QuaternionXyzw{
    node[0].as<double>(), node[1].as<double>(),
    node[2].as<double>(), node[3].as<double>()};
}

InputContractConfig ParseInput(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"accepted_vo_states", "diagnostics_topic", "expected_child_frame",
      "expected_odometry_publisher", "expected_parent_frame",
      "expected_status_frame", "expected_status_publisher", "odometry_topic",
      "required_diagnostics", "visual_slam_status_topic"},
    "input");
  const YAML::Node states_node = node["accepted_vo_states"];
  if (!states_node.IsSequence() || states_node.size() == 0U) {
    throw std::runtime_error("accepted_vo_states must be a non-empty sequence");
  }
  std::vector<std::uint8_t> states;
  std::set<int> state_set;
  for (const auto & state_node : states_node) {
    const int state = state_node.as<int>();
    if (state < 0 || state > std::numeric_limits<std::uint8_t>::max()) {
      throw std::runtime_error("accepted_vo_states values must fit uint8");
    }
    if (!state_set.insert(state).second) {
      throw std::runtime_error("accepted_vo_states must not contain duplicates");
    }
    states.push_back(static_cast<std::uint8_t>(state));
  }

  const YAML::Node diagnostics_node = node["required_diagnostics"];
  if (!diagnostics_node.IsSequence() || diagnostics_node.size() == 0U) {
    throw std::runtime_error("required_diagnostics must be a non-empty sequence");
  }
  std::vector<RequiredDiagnosticConfig> diagnostics;
  std::set<std::string> names;
  for (std::size_t index = 0; index < diagnostics_node.size(); ++index) {
    const YAML::Node diagnostic_node = diagnostics_node[index];
    RequireExactKeys(
      diagnostic_node,
      {"expected_publisher", "hardware_id", "maximum_level", "name", "required_values"},
      "required_diagnostics[" + std::to_string(index) + "]");
    const std::string name = NonemptyString(diagnostic_node["name"], "diagnostic.name");
    if (!names.insert(name).second) {
      throw std::runtime_error("required diagnostic name is duplicated: " + name);
    }
    if (EndsWith(name, kOwnDiagnosticSuffix)) {
      throw std::runtime_error(
              "required diagnostics must not depend on this adapter's own status");
    }
    const int maximum_level = diagnostic_node["maximum_level"].as<int>();
    if (maximum_level != 0) {
      throw std::runtime_error("schema version one requires diagnostic maximum_level=0");
    }
    const std::string hardware_id = NonemptyString(
      diagnostic_node["hardware_id"], "diagnostic.hardware_id");
    const std::string expected_publisher = AbsoluteNodeName(
      diagnostic_node["expected_publisher"], "diagnostic.expected_publisher");
    auto required_values = StringMap(
      diagnostic_node["required_values"], "diagnostic.required_values");
    diagnostics.push_back(
      RequiredDiagnosticConfig{
        name,
        hardware_id,
        expected_publisher,
        static_cast<std::uint8_t>(maximum_level),
        std::move(required_values)});
  }

  return InputContractConfig{
    AbsoluteTopic(node["odometry_topic"], "input.odometry_topic"),
    AbsoluteTopic(
      node["visual_slam_status_topic"], "input.visual_slam_status_topic"),
    AbsoluteTopic(node["diagnostics_topic"], "input.diagnostics_topic"),
    AbsoluteNodeName(
      node["expected_odometry_publisher"], "input.expected_odometry_publisher"),
    AbsoluteNodeName(
      node["expected_status_publisher"], "input.expected_status_publisher"),
    localization_contracts::ValidateFrameId(
      NonemptyString(node["expected_parent_frame"], "input.expected_parent_frame")),
    localization_contracts::ValidateFrameId(
      NonemptyString(node["expected_child_frame"], "input.expected_child_frame")),
    localization_contracts::ValidateFrameId(
      NonemptyString(node["expected_status_frame"], "input.expected_status_frame")),
    std::move(states),
    std::move(diagnostics)};
}

HealthContractConfig ParseHealth(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"diagnostic_period_sec", "diagnostic_stale_after_sec",
      "maximum_clock_residual_sec", "maximum_odometry_receive_gap_sec",
      "maximum_odometry_stamp_gap_sec", "maximum_position_step_m",
      "maximum_rotation_step_deg", "maximum_status_odometry_skew_sec",
      "minimum_odometry_rate_hz", "odometry_rate_window_samples",
      "odometry_stale_after_sec", "recovery_consecutive_samples",
      "startup_timeout_sec", "tracking_stale_after_sec"},
    "health");
  const HealthContractConfig health{
    PositiveDouble(node["diagnostic_period_sec"], "health.diagnostic_period_sec"),
    PositiveDouble(
      node["odometry_stale_after_sec"], "health.odometry_stale_after_sec"),
    PositiveDouble(
      node["tracking_stale_after_sec"], "health.tracking_stale_after_sec"),
    PositiveDouble(
      node["diagnostic_stale_after_sec"], "health.diagnostic_stale_after_sec"),
    PositiveDouble(node["startup_timeout_sec"], "health.startup_timeout_sec"),
    PositiveSize(
      node["recovery_consecutive_samples"],
      "health.recovery_consecutive_samples"),
    PositiveDouble(
      node["minimum_odometry_rate_hz"], "health.minimum_odometry_rate_hz"),
    PositiveDouble(
      node["maximum_odometry_stamp_gap_sec"],
      "health.maximum_odometry_stamp_gap_sec"),
    PositiveDouble(
      node["maximum_odometry_receive_gap_sec"],
      "health.maximum_odometry_receive_gap_sec"),
    PositiveDouble(
      node["maximum_clock_residual_sec"], "health.maximum_clock_residual_sec"),
    PositiveDouble(
      node["maximum_status_odometry_skew_sec"],
      "health.maximum_status_odometry_skew_sec"),
    PositiveDouble(
      node["maximum_position_step_m"], "health.maximum_position_step_m"),
    PositiveDouble(
      node["maximum_rotation_step_deg"],
      "health.maximum_rotation_step_deg") * kDegreesToRadians,
    PositiveSize(
      node["odometry_rate_window_samples"],
      "health.odometry_rate_window_samples")};
  const double minimum_stale = std::min({
      health.odometry_stale_after_sec,
      health.tracking_stale_after_sec,
      health.diagnostic_stale_after_sec});
  const double maximum_stale = std::max({
      health.odometry_stale_after_sec,
      health.tracking_stale_after_sec,
      health.diagnostic_stale_after_sec});
  if (health.diagnostic_period_sec >= minimum_stale) {
    throw std::runtime_error(
            "diagnostic_period_sec must be smaller than every stale threshold");
  }
  if (health.startup_timeout_sec <= maximum_stale) {
    throw std::runtime_error("startup_timeout_sec must exceed every stale threshold");
  }
  if (health.maximum_odometry_stamp_gap_sec >= health.odometry_stale_after_sec ||
    health.maximum_odometry_receive_gap_sec >= health.odometry_stale_after_sec)
  {
    throw std::runtime_error(
            "odometry gap thresholds must be smaller than odometry_stale_after_sec");
  }
  if (health.maximum_status_odometry_skew_sec >= health.tracking_stale_after_sec) {
    throw std::runtime_error(
            "maximum_status_odometry_skew_sec must be smaller than tracking stale time");
  }
  if (health.maximum_rotation_step_rad > 180.0 * kDegreesToRadians) {
    throw std::runtime_error("maximum_rotation_step_deg must not exceed 180 degrees");
  }
  if (health.odometry_rate_window_samples < 2U) {
    throw std::runtime_error("odometry_rate_window_samples must be at least two");
  }
  return health;
}

ExtrinsicGateConfig ParseExtrinsic(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"child_frame", "parent_frame", "provenance", "reason", "rotation_xyzw",
      "status", "translation_m"},
    "extrinsic");
  const std::string status = Status(node["status"], "extrinsic.status");
  const std::string parent = localization_contracts::ValidateFrameId(
    NonemptyString(node["parent_frame"], "extrinsic.parent_frame"));
  const std::string child = localization_contracts::ValidateFrameId(
    NonemptyString(node["child_frame"], "extrinsic.child_frame"));
  if (parent == child) {
    throw std::runtime_error("extrinsic parent and child frames must differ");
  }
  const auto provenance = OptionalString(node["provenance"], "extrinsic.provenance");
  const std::string reason = NonemptyString(node["reason"], "extrinsic.reason");
  if (status == "blocked") {
    if (!node["translation_m"].IsNull() || !node["rotation_xyzw"].IsNull()) {
      throw std::runtime_error(
              "blocked extrinsic must not contain placeholder transform values");
    }
    if (provenance.has_value()) {
      throw std::runtime_error("blocked extrinsic must not claim provenance");
    }
    return ExtrinsicGateConfig{
      status, parent, child, std::nullopt, std::nullopt, reason};
  }
  if (!provenance.has_value()) {
    throw std::runtime_error("approved extrinsic requires provenance");
  }
  try {
    return ExtrinsicGateConfig{
      status,
      parent,
      child,
      RigidTransform::Create(
        parent,
        child,
        Vector3(node["translation_m"], "extrinsic.translation_m"),
        Quaternion(node["rotation_xyzw"], "extrinsic.rotation_xyzw")),
      provenance,
      reason};
  } catch (const ContractViolation & error) {
    throw std::runtime_error(std::string("invalid approved extrinsic: ") + error.what());
  }
}

EvidenceGateConfig ParseEvidenceGate(const YAML::Node & node, const std::string & label)
{
  RequireExactKeys(node, {"approval_id", "policy", "reason", "status"}, label);
  const std::string status = Status(node["status"], label + ".status");
  const std::string policy = NonemptyString(node["policy"], label + ".policy");
  const auto approval_id = OptionalString(node["approval_id"], label + ".approval_id");
  const std::string reason = NonemptyString(node["reason"], label + ".reason");
  if (status != "blocked") {
    throw std::runtime_error(
            "schema version one requires " + label + " status='blocked'");
  }
  if (policy != "reject" || approval_id.has_value()) {
    throw std::runtime_error(
            "blocked " + label + " must use policy='reject' and no approval_id");
  }
  return EvidenceGateConfig{status, policy, approval_id, reason};
}

struct PublicationConfig
{
  AuthorizationState authorization;
  std::string output_topic;
  std::string output_parent_frame;
  std::string output_child_frame;
};

PublicationConfig ParsePublication(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"authorization", "output_child_frame", "output_parent_frame", "output_topic"},
    "publication");
  const std::string value =
    NonemptyString(node["authorization"], "publication.authorization");
  if (value != "shadow_only") {
    throw std::runtime_error(
            "schema version one accepts only shadow_only authorization");
  }
  return PublicationConfig{
    AuthorizationState::kShadowOnly,
    AbsoluteTopic(node["output_topic"], "publication.output_topic"),
    localization_contracts::ValidateFrameId(
      NonemptyString(node["output_parent_frame"], "publication.output_parent_frame")),
    localization_contracts::ValidateFrameId(
      NonemptyString(node["output_child_frame"], "publication.output_child_frame"))};
}

}  // namespace

bool EvidenceGateConfig::Approved() const noexcept
{
  return status == "approved" && approval_id.has_value() && !approval_id->empty();
}

bool ExtrinsicGateConfig::Approved() const noexcept
{
  return status == "approved" && transform.has_value() && provenance.has_value() &&
         !provenance->empty();
}

localization_contracts::ContractApprovals ContractConfig::Approvals() const noexcept
{
  return localization_contracts::ContractApprovals{
    extrinsic.Approved(), twist.Approved(), covariance.Approved()};
}

std::string CalculateFileSha256(const std::string & path)
{
  return CalculateSha256(ReadFile(path));
}

ContractConfig LoadContractConfig(const std::string & path)
{
  const std::string content = ReadFile(path);
  YAML::Node root;
  try {
    const std::vector<YAML::Node> documents = YAML::LoadAll(content);
    if (documents.size() != 1U) {
      throw std::runtime_error(
              "localization contract must contain exactly one YAML document");
    }
    root = documents.front();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::string("invalid contract YAML: ") + error.what());
  }
  RequireExactKeys(
    root,
    {"contract_id", "covariance", "extrinsic", "health", "input", "publication",
      "schema_version", "status", "twist"},
    "contract");
  const int schema_version = root["schema_version"].as<int>();
  if (schema_version != 1) {
    throw std::runtime_error("schema_version must be exactly one");
  }
  const std::string contract_id = NonemptyString(root["contract_id"], "contract_id");
  const std::string status = Status(root["status"], "status");
  if (status != "blocked") {
    throw std::runtime_error("schema version one requires top-level status='blocked'");
  }
  InputContractConfig input = ParseInput(root["input"]);
  HealthContractConfig health = ParseHealth(root["health"]);
  ExtrinsicGateConfig extrinsic = ParseExtrinsic(root["extrinsic"]);
  EvidenceGateConfig twist = ParseEvidenceGate(root["twist"], "twist");
  EvidenceGateConfig covariance = ParseEvidenceGate(root["covariance"], "covariance");
  PublicationConfig publication = ParsePublication(root["publication"]);
  if (input.odometry_topic == input.visual_slam_status_topic ||
    input.odometry_topic == input.diagnostics_topic ||
    input.visual_slam_status_topic == input.diagnostics_topic)
  {
    throw std::runtime_error(
            "odometry, status, and diagnostics input topics must be distinct");
  }
  if (input.diagnostics_topic != "/diagnostics") {
    throw std::runtime_error(
            "schema version one diagnostics_topic must be exactly /diagnostics");
  }
  if (input.expected_parent_frame != "odom" ||
    input.expected_child_frame != "camera_link" ||
    input.expected_status_frame != "map")
  {
    throw std::runtime_error(
            "schema version one requires odom/camera_link input and map status frames");
  }
  if (input.accepted_vo_states.size() != 1U || input.accepted_vo_states.front() != 1U) {
    throw std::runtime_error("schema version one accepts only cuVSLAM vo_state=1");
  }
  if (extrinsic.parent_frame != "base_link" ||
    extrinsic.child_frame != "camera_link")
  {
    throw std::runtime_error(
            "schema version one extrinsic must be T[base_link,camera_link]");
  }
  if (publication.output_topic != "/localization/odometry" ||
    publication.output_parent_frame != "odom" ||
    publication.output_child_frame != "base_link")
  {
    throw std::runtime_error(
            "schema version one output contract must be /localization/odometry "
            "with odom/base_link frames");
  }
  if (publication.output_parent_frame != input.expected_parent_frame) {
    throw std::runtime_error(
            "publication output parent must equal the input odometry parent");
  }
  if (extrinsic.child_frame != input.expected_child_frame) {
    throw std::runtime_error(
            "extrinsic child must equal the tracked camera child frame");
  }
  if (publication.output_child_frame != extrinsic.parent_frame) {
    throw std::runtime_error(
            "publication output child must equal the extrinsic parent frame");
  }
  return ContractConfig{
    schema_version,
    contract_id,
    status,
    CalculateSha256(content),
    std::move(input),
    health,
    std::move(extrinsic),
    std::move(twist),
    std::move(covariance),
    publication.authorization,
    publication.output_topic,
    publication.output_parent_frame,
    publication.output_child_frame};
}

std::optional<std::string> CheckDiagnosticEvidence(
  const RequiredDiagnosticConfig & required,
  const std::uint8_t actual_level,
  const std::string & actual_hardware_id,
  const std::vector<std::pair<std::string, std::string>> & actual_values)
{
  if (actual_level > required.maximum_level ||
    actual_hardware_id != required.hardware_id)
  {
    return std::string("UPSTREAM_DIAGNOSTIC_IDENTITY_OR_LEVEL_MISMATCH");
  }
  std::map<std::string, std::string> values;
  for (const auto & value : actual_values) {
    if (!values.emplace(value.first, value.second).second) {
      return std::string("UPSTREAM_DIAGNOSTIC_DUPLICATE_VALUE_KEY");
    }
  }
  for (const auto & expected : required.required_values) {
    const auto actual = values.find(expected.first);
    if (actual == values.end() || actual->second != expected.second) {
      return std::string("UPSTREAM_DIAGNOSTIC_VALUE_MISMATCH");
    }
  }
  return std::nullopt;
}

}  // namespace cuvslam_localization_adapter
