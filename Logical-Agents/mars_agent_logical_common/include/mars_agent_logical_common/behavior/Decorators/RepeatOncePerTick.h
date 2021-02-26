#ifndef REPEATONCEPERTICK_H
#define REPEATONCEPERTICK_H

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

class RepeatOncePerTick : public BT::DecoratorNode
{
public:
  RepeatOncePerTick(const std::string& pName);

  virtual ~RepeatOncePerTick() override = default;

private:
  virtual BT::NodeStatus tick() override;

     void halt() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // REPEATONCEPERTICK_H
