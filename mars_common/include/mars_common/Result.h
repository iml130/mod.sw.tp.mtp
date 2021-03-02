//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef MARS_COMMON_RESULT_H
#define MARS_COMMON_RESULT_H

#include <string>
#include <cstdint>

namespace mars
{
namespace common
{
class Result
{
public:
  Result(int8_t resultStatus, std::string description);

  int8_t getResultStatus() const;
  void setResultStatus(const int8_t& resultStatus);

  std::string getDescription() const;
  void setDescription(const std::string& description);

private:
  int8_t mResultStatus;
  std::string mDescription;
};
}  // namespace common
}  // namespace mars

#endif  // MARS_COMMON_RESULT_H
