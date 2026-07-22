// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_CONTRACTS__ERRORS_HPP_
#define LOCALIZATION_CONTRACTS__ERRORS_HPP_

#include <stdexcept>
#include <string>

namespace localization_contracts
{

class ContractViolation : public std::runtime_error
{
public:
  explicit ContractViolation(const std::string & message)
  : std::runtime_error(message) {}
};

class FrameContractViolation : public ContractViolation
{
public:
  explicit FrameContractViolation(const std::string & message)
  : ContractViolation(message) {}
};

class NumericContractViolation : public ContractViolation
{
public:
  explicit NumericContractViolation(const std::string & message)
  : ContractViolation(message) {}
};

class UnapprovedTwistSemantics : public ContractViolation
{
public:
  explicit UnapprovedTwistSemantics(const std::string & message)
  : ContractViolation(message) {}
};

}  // namespace localization_contracts

#endif  // LOCALIZATION_CONTRACTS__ERRORS_HPP_
