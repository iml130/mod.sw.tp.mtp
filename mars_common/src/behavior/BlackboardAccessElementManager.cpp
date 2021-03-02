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


#include "mars_common/behavior/BlackboardAccessElementManager.h"
#include "mars_common/Logger.h"

BT::BlackboardAccessElementManager::BlackboardAccessElementManager()
{
  this->mMutex = new std::mutex();
}

BT::BlackboardAccessElementManager::~BlackboardAccessElementManager() { delete this->mMutex; }

void BT::BlackboardAccessElementManager::lock(const mars::common::Id& pClassId)
{
  this->mMutex->lock();
  this->mAccessListSequence.push_back(pClassId);
}

bool BT::BlackboardAccessElementManager::tryLock() { return this->mMutex->try_lock(); }

bool BT::BlackboardAccessElementManager::unlock(const mars::common::Id& pClassId)
{
  bool lErrorOccurred = false;

  if (this->mAccessListSequence.begin() != this->mAccessListSequence.end())
  {
    if (*(this->mAccessListSequence.begin()) == pClassId)
    {
      this->mAccessListSequence.pop_front();
      this->mMutex->unlock();
    }
    else
    {
      MARS_LOG_INFO("Mutex is logged by another ClassId: "
                    << std::to_string(*(this->mAccessListSequence.begin())));

      lErrorOccurred = true;
    }
  }
  else
  {
    MARS_LOG_INFO("Mutex was not logged.");
    this->mMutex->unlock();
  }

  return !lErrorOccurred;
}

mars::common::Id BT::BlackboardAccessElementManager::getCurrentOwnerId() const 
{

  if (this->mAccessListSequence.size() > 0)
  {
    return *(this->mAccessListSequence.begin());
  }
  else
  {
    return mars::common::Id::createInvalidId();
  } 
}
