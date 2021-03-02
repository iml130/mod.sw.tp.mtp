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


#include "mars_common/behavior/CoutDurationLogger.h"

#include <algorithm>

std::atomic<bool> BT::CoutDurationLogger::gRefCount(false);

static const size_t WHITE_SPACE_COUNT = 25;
static const char* WHITESPACES = "                         ";

BT::CoutDurationLogger::CoutDurationLogger(const BT::Tree& pTree)
    : BT::StatusChangeLogger(pTree.rootNode())
{
  bool lExpected = false;
  if (!gRefCount.compare_exchange_strong(lExpected, true))
  {
    throw LogicError("Only one instance of CoutDurationLogger shall be created");
  }
}

BT::CoutDurationLogger::~CoutDurationLogger() {}

void BT::CoutDurationLogger::callback(BT::Duration pTimestamp, const BT::TreeNode& pNode,
                                      BT::NodeStatus pPrevStatus, BT::NodeStatus pStatus)
{
  std::chrono::duration<double> lDuration = this->handleTransition(pNode);

  printf("[%.3f]: %s%s %s -> %s",
           lDuration.count(), pNode.name().c_str(),
           &WHITESPACES[std::min(WHITE_SPACE_COUNT, pNode.name().size())],
           toStr(pPrevStatus, true).c_str(),
           toStr(pStatus, true).c_str() );
    std::cout << std::endl;
}

std::chrono::duration<double> BT::CoutDurationLogger::handleTransition(const TreeNode& pNode)
{
  std::map<std::string, std::chrono::high_resolution_clock::time_point>::iterator lIt;
  std::chrono::high_resolution_clock::time_point lCurrentTime =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> lDuration = std::chrono::steady_clock::duration::zero();

  lIt = this->mStartTimes.find(pNode.name());

  if (lIt != this->mStartTimes.end())
  {
    lDuration =
        std::chrono::duration_cast<std::chrono::duration<double>>(lCurrentTime - lIt->second);

    lIt->second = lCurrentTime;
  }
  else
  {
    std::pair<std::string, std::chrono::high_resolution_clock::time_point> lNewEntry(pNode.name(),
                                                                                     lCurrentTime);

    this->mStartTimes.insert(lNewEntry);
  }

  return lDuration;
}
