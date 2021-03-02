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

#include "mars_common/Value.h"

mars::common::Value::Value(std::string& valueType, std::string& value)
{
  this->mValueType = valueType;
  this->mValue = value;
}

std::string mars::common::Value::getValueType() const
{
  return mValueType;
}

void mars::common::Value::setValueType(const std::string& valueType)
{
  mValueType = valueType;
}

std::string mars::common::Value::getValue() const
{
  return mValue;
}

void mars::common::Value::setValue(const std::string& value)
{
  mValue = value;
}
