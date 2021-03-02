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


#ifndef MARS_COMMON_BEHAVIOR_BLACKBOARDACCESSELEMENTMANAGER_H
#define MARS_COMMON_BEHAVIOR_BLACKBOARDACCESSELEMENTMANAGER_H

#include "mars_common/Id.h"

#include <list>
#include <mutex>

namespace BT
{

class BlackboardAccessElementManager
{
public:
  BlackboardAccessElementManager(void);
  ~BlackboardAccessElementManager(void);

  /**
   * @brief
   *
   * @param pClassId
   */
  void lock(const mars::common::Id& pClassId);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool tryLock(void);

  /**
   * @brief
   *
   * @param pClassId
   * @return true
   * @return false
   */
  bool unlock(const mars::common::Id& pClassId);

  /**
   * @brief Get the Current Owner Id. If no current Owner is available, an invalid ID is returned
   * (mars::common::Id::createInvalidId()).
   *
   * @return mars::common::Id
   */
  mars::common::Id getCurrentOwnerId(void) const;

private:
  std::mutex* mMutex;
  std::list<mars::common::Id> mAccessListSequence;
};

} // namespace BT

#endif // MARS_COMMON_BEHAVIOR_BLACKBOARDACCESSELEMENTMANAGER_H