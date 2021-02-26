#ifndef MARS_TOPOLOGY_COMMON_UTILITY_REGISTRATION_GUARD_H
#define MARS_TOPOLOGY_COMMON_UTILITY_REGISTRATION_GUARD_H

#include <string>
#include <unordered_set>

#include <ros/ros.h>

#include "mars_common/Id.h"
#include "mars_topology_msgs/Registration.h"

namespace mars
{
namespace topology
{
namespace common
{
namespace utility
{
class RegistrationGuard  // static
{
public:
  template <typename... Strings>
  RegistrationGuard(Strings... pIds)
  {
    static ros::NodeHandle sNH;
    std::unordered_set<std::string> lIdStrings({pIds...});

    for (const std::string& iIdString : lIdStrings)
    {
      mRequiredRegistrations.insert(mars::common::Id(iIdString));
    }

    ros::Subscriber lSub = sNH.subscribe("/topology/topology_register", 1000, &RegistrationGuard::checkRegistration, this);

    ros::Rate lRate(10);
    while (!mRequiredRegistrations.empty())
    {
      ros::spinOnce();
      lRate.sleep();
    }
  };

private:
  std::unordered_set<mars::common::Id> mRequiredRegistrations;
  
  void checkRegistration(const mars_topology_msgs::RegistrationConstPtr& pRegistrationMsg)
  {
    if (pRegistrationMsg->registration_action ==
        mars_topology_msgs::Registration::REGISTRATION_ACTION_REGISTER)
    {
      mars::common::Id lRegistrationId(pRegistrationMsg->entity.id);
      std::unordered_set<mars::common::Id>::const_iterator lIterator = mRequiredRegistrations.find(lRegistrationId);

      if (lIterator != mRequiredRegistrations.end())
      {
        mRequiredRegistrations.erase(lIterator);
      }
    }
  };
};
}  // namespace utility
}  // namespace common
}  // namespace topology
}  // namespace mars

#endif  // MARS_TOPOLOGY_COMMON_UTILITY_REGISTRATION_GUARD_H