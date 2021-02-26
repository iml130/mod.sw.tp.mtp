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