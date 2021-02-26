/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * --------------------------------------------------------------------------------------------- */

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
