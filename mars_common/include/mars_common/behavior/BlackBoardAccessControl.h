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

#ifndef MARS_COMMON_BEHAVIOR_BLACKBOARDACCESSCONTROL_H
#define MARS_COMMON_BEHAVIOR_BLACKBOARDACCESSCONTROL_H

#include "BlackboardAccessElementManager.h"
#include "mars_common/Id.h"

#include <behaviortree_cpp_v3/tree_node.h>
#include <string>

namespace BT
{
enum ValueAccessMode
{
  READ_AND_LOCK_NOT_BLOCKING,
  READ_AND_LOCK_BLOCKING,
  READ_ONLY
};

class BlackBoardAccessControl
{
public:
  /**
   * @brief
   *
   * @tparam T Type of the data that should be read from blackboard.
   * @param pBlackboardValue Parameter were the value will be written to.
   * @param pBlackboardKey Key to access the value on the blackboard.
   * @param pTreeNode Tree of the blackboard.
   * @param pClassId Id of the class which wants the access.
   * @param pValueAccessMode Mode of how the value should be accessed.
   * @return Returns true if everythings works fine, false in case a lock in not blocking mode
   * (ValueAccessMode::READ_AND_LOCK_NOT_BLOCKING) couldn't be directly set.
   */
  template <typename T>
  static bool readFromBlackboard(T& pBlackboardValue, const std::string& pBlackboardKey,
                                 const TreeNode& pTreeNode, const mars::common::Id& pClassId,
                                 ValueAccessMode pValueAccessMode = ValueAccessMode::READ_ONLY);
  template <typename T>
  static bool writeToBlackboard(const T& pBlackboardValue, const std::string& pBlackboardKey,
                                const TreeNode& pTreeNode, const mars::common::Id& pClassId);

  static bool unlockBlackboardParam(const std::string& pBlackboardKey,
                                    const mars::common::Id& pClassId);

private:
  BlackBoardAccessControl() {}

  static BT::BlackboardAccessElementManager*
  getBlackboardAccessElementManager(const std::string& pBlackboardKey);
};

} // namespace BT

#endif // MARS_COMMON_BEHAVIOR_ALLOCATEENTITY_H
