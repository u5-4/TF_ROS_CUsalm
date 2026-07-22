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
