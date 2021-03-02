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

#include "mars_common/Tuple.h"

mars::common::Tuple::Tuple(std::string key, mars::common::Value value) : mKey(key), mValue(value)
{
}

std::string mars::common::Tuple::getKey() const
{
  return mKey;
}

void mars::common::Tuple::setKey(const std::string& key)
{
  mKey = key;
}

mars::common::Value mars::common::Tuple::getValue() const
{
  return mValue;
}

void mars::common::Tuple::setValue(const Value& value)
{
  mValue = value;
}
