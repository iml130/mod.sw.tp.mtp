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

#include "mars_common/Result.h"

mars::common::Result::Result(int8_t resultStatus, std::string description)
  : mResultStatus(resultStatus), mDescription(description)
{
}

int8_t mars::common::Result::getResultStatus() const
{
  return mResultStatus;
}

void mars::common::Result::setResultStatus(const int8_t& resultStatus)
{
  mResultStatus = resultStatus;
}

std::string mars::common::Result::getDescription() const
{
  return mDescription;
}

void mars::common::Result::setDescription(const std::string& description)
{
  mDescription = description;
}
