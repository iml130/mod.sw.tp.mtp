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

#include "mars_yellow_pages/MARSYellowPages.h"


MARSYellowPages::MARSYellowPages(){}

MARSYellowPages::~MARSYellowPages(){}

void MARSYellowPages::addContainer(
    const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg)
{
  this->mContainerMap.emplace(
      mars::common::Id(pMsg->container_id).getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC),
      this->createContainerEntitiesSet(pMsg));
}

mars::common::Id MARSYellowPages::getContainerId(
    const mars_topology_srvs::GetTopologyEntityContainerId::Request& pReq)
{
  std::string lTmpEntityIdString =
      mars::common::Id(pReq.entity_id).getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC);
  mars::common::Id lContainerId = mars::common::Id::createInvalidId();

  for (auto& it : this->mContainerMap)
  {
    if (it.second.find(lTmpEntityIdString) != it.second.end())
    {
      lContainerId = mars::common::Id(it.first);
      break;
    }
  }

  return lContainerId;
}

std::set<std::string> MARSYellowPages::createContainerEntitiesSet(
    const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& pMsg) const
{
  std::set<std::string> lContainerEntitiesSet;

  for (size_t i = 0; i < pMsg->contained_entity_ids.size(); i++)
  {
    lContainerEntitiesSet.emplace(mars::common::Id(pMsg->contained_entity_ids[i])
                                      .getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC));
  }

  return lContainerEntitiesSet;
}