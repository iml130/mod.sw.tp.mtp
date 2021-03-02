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

#ifndef MARS_COMMON_TUPLE_H
#define MARS_COMMON_TUPLE_H

#include <mars_common/Value.h>

#include <string>

namespace mars
{
namespace common
{
class Tuple
{
public:
  /**
   * @brief Tuple Constructs an object of this class
   * @param key Key of the value
   * @param value Value behind the key
   */
  Tuple(std::string key, mars::common::Value value);

  /**
   * @brief key Gets the key
   * @return The key
   */
  std::string getKey() const;

  /**
   * @brief setKey Sets the Key
   * @param key Next key value
   */
  void setKey(const std::string& key);

  /**
   * @brief value Gets the value
   * @return The value
   */
  mars::common::Value getValue() const;

  /**
   * @brief setValue Set the value
   * @param value Next value
   */
  void setValue(const Value& value);

private:
  /**
   * @brief mKey The key of this value
   */
  std::string mKey;

  /**
   * @brief mValue The value of this class
   */
  mars::common::Value mValue;
};
}  // namespace common
}  // namespace mars

#endif  // MARS_COMMON_TUPLE_H
