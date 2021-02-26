#ifndef REPEATUNTILDONE_H
#define REPEATUNTILDONE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <mars_common/exception/ReadParamException.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace common
{
namespace behavior
{

class RepeatUntilDone : public BT::DecoratorNode
{
public:
  RepeatUntilDone(const std::string& pName);

  RepeatUntilDone(const std::string& name, const BT::NodeConfiguration& config);

  virtual ~RepeatUntilDone() override = default;

  /**
   * @brief providedPorts
   * @return
   */
  static BT::PortsList providedPorts();

private:
  virtual BT::NodeStatus tick() override;

  const bool readDoneStatus() noexcept(false);

  void halt() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // REPEATUNTILDONE_H
