#ifndef MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREE_H
#define MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREE_H

#include "TimeIntervalBasedObjectTreeNode.h"

#include <mars_common/exception/SetParamException.h>

#include <memory>

namespace mars
{
namespace topology
{
namespace common
{
/**
 * Binary tree for TimeIntervalBasedObjectTreeNodes.
 */
class TimeIntervalBasedObjectTree
{
public:
  TimeIntervalBasedObjectTree();

  /**
   * @brief 
   * 
   * @param timeIntervalBaseObject 
   * @throw mars::common::exception::SetParamException
   */
  void insert(TimeIntervalBaseObject* timeIntervalBaseObject) noexcept(false);

  bool checkOverlap(const mars::common::TimeInterval& pTimeInterval) const;

  std::shared_ptr<TimeIntervalBasedObjectTreeNode> getRoot(void) const;

private:
  std::shared_ptr<TimeIntervalBasedObjectTreeNode> mRoot;

  static std::shared_ptr<TimeIntervalBasedObjectTreeNode>
  insertTimeIntervalBaseObject(std::shared_ptr<TimeIntervalBasedObjectTreeNode> pRoot,
                               TimeIntervalBaseObject* pTimeIntervalBaseObject);

  static std::shared_ptr<TimeIntervalBaseObject>
  checkOverlapWithTree(std::shared_ptr<TimeIntervalBasedObjectTreeNode> pRoot,
                       const TimeIntervalBaseObject& pTimeIntervalBaseObject);
};
} // namespace common
} // namespace topology
} // namespace mars

namespace std
{
inline string
to_string(const mars::topology::common::TimeIntervalBasedObjectTree& pTimeIntervalBasedObjectTree)
{
  return to_string(pTimeIntervalBasedObjectTree.getRoot());
}
} // namespace std

#endif // MARS_TOPOLOGY_COMMON_TIMEINTERVALBASEDOBJECTTREE_H
