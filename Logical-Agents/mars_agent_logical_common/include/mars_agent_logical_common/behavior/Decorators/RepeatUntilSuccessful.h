#ifndef REPEATUNTILSUCCESSFUL_H
#define REPEATUNTILSUCCESSFUL_H

#include <behaviortree_cpp_v3/decorator_node.h>

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
class RepeatUntilSuccessful : public BT::DecoratorNode
{
public:
  RepeatUntilSuccessful(const std::string& pName);

  virtual ~RepeatUntilSuccessful() override = default;

private:
  virtual BT::NodeStatus tick() override;

  void halt() override;
};

} // namespace behavior
} // namespace common
} // namespace logical
} // namespace agent
} // namespace mars

#endif // REPEATUNTILSUCCESFUL_H
