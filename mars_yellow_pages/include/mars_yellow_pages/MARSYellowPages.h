/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : mars_yellow_pages
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * --------------------------------------------------------------------------------------------- */

#ifndef MARS_YELLOW_PAGES_H
#define MARS_YELLOW_PAGES_H

#include <mars_common/Id.h>
#include <mars_topology_msgs/TopologyEntityRegistration.h>
#include <mars_topology_srvs/GetTopologyEntityContainerId.h>

#include <map>
#include <set>
#include <string>

class MARSYellowPages
{
public:
  MARSYellowPages();
  ~MARSYellowPages();
  void addContainer(const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg);

  mars::common::Id
  getContainerId(const mars_topology_srvs::GetTopologyEntityContainerId::Request& pReq);

private:
  std::map<std::string, std::set<std::string>> mContainerMap;

  std::set<std::string> createContainerEntitiesSet(
      const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg) const;
};

#endif // MARS_YELLOW_PAGES_H