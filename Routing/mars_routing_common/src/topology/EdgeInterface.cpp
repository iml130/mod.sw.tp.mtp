#include "mars_routing_common/topology/EdgeInterface.h"
#include "mars_common/Id.h"
#include "mars_common/Logger.h"

#include "mars_topology_srvs/GetConnections.h"

#include <tf2_eigen/tf2_eigen.h>

static const std::string CONTAINER_ID_PREFIX = "container_";
static const ros::Duration ACTION_CLIENT_TIMEOUT(5, 0);
static const ros::Duration TF_LOOKUP_SLEEP_DURATION(0.2);
static const ros::Duration TF_LOOKUP_TIMEOUT(2, 0);

static const std::string REQUEST_CONTAINER_ID_SERVICE = "/lookup_container";

mars::routing::common::topology::EdgeInterface::EdgeInterface(const mars::common::Id& pId)
    : mEdgeId(pId), mAllocationActionClient(nullptr)
{
  this->initContainerId();
}

mars::routing::common::topology::EdgeInterface::~EdgeInterface()
{
  if (this->mAllocationActionClient != nullptr)
  {
    delete this->mAllocationActionClient;
  }

  this->mAllocationActionClient = nullptr;
}

void mars::routing::common::topology::EdgeInterface::setNamespace(const std::string& pNamespace)
{
  std::lock_guard<std::mutex>{sMutex};
  sNamespace = pNamespace;

  if (sNamespace.back() != *"/")
  {
    sNamespace += "/";
  }
}

boost::optional<int> mars::routing::common::topology::EdgeInterface::getType()
{
  mars_topology_srvs::GetType lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_type");

  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get type for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  return lService.response.entity_type.entity_type;
}

mars::common::Result mars::routing::common::topology::EdgeInterface::addReservation(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pTimeInterval)
{
  mars_topology_srvs::AddReservation lService;
  bool callSuccessful = false;

  // Setup Request
  lService.request.entity_id = this->mEdgeId.toMsg();
  lService.request.reservation.agent_id.uuid = pAgentId.getUUID();
  lService.request.reservation.agent_id.description = pAgentId.getDescription();
  lService.request.reservation.path_id.uuid = pPathId.getUUID();
  lService.request.reservation.path_id.description = pPathId.getDescription();
  lService.request.reservation.time_interval.start_time = pTimeInterval.getStartTime();
  lService.request.reservation.time_interval.duration = pTimeInterval.getDuration();

  std::string l_service_name = constructServiceTopic("/add_reservation");

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to add reservation for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  return mars::common::Result(lService.response.result.result,
                              lService.response.result.description);
}

mars::common::Result
mars::routing::common::topology::EdgeInterface::deleteReservation(const mars::common::Id& pAgentId,
                                                                  const mars::common::Id& pPathId)
{
  mars_topology_srvs::DeleteReservation lService;
  bool callSuccessful = false;

  // Setup request
  lService.request.entity_id = this->mEdgeId.toMsg();
  lService.request.agent_id.uuid = pAgentId.getUUID();
  lService.request.agent_id.description = pAgentId.getDescription();
  lService.request.path_id.uuid = pPathId.getUUID();
  lService.request.path_id.description = pPathId.getDescription();

  std::string l_service_name = constructServiceTopic("/delete_reservation");

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to delete reservation for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  return mars::common::Result(lService.response.result.result,
                              lService.response.result.description);
}

void mars::routing::common::topology::EdgeInterface::setContainerId(
    const mars::common::Id& pContainerId)
{
  this->mContainerId = pContainerId;
}

bool mars::routing::common::topology::EdgeInterface::allocateEdge(
    const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
    const mars::common::TimeInterval& pAllocationInterval, const ros::Duration& pWaitForGoal,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleDoneCallback& pDoneCallback,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleActiveCallback& pActiveCallback,
    const actionlib::SimpleActionClient<
        mars_topology_actions::AllocateEntityAction>::SimpleFeedbackCallback& pFeedbackCallback)
{
  std::lock_guard<std::mutex>{this->mMutex};

  if (this->mAllocationActionClient == nullptr)
  {
    std::string l_action_name = constructServiceTopic("/allocate");

    this->mAllocationActionClient =
        new actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>(
            l_action_name, true);

    if (!this->mAllocationActionClient->waitForServer(ACTION_CLIENT_TIMEOUT))
    {
      MARS_LOG_ERROR("Could not connect to ActionServer!");
      return false;
    }
  }

  mars_topology_actions::AllocateEntityGoal lAllocationGoal;
  lAllocationGoal.topology_entity_id = this->mEdgeId.toMsg();
  lAllocationGoal.agent_id = pAgentId.toMsg();
  lAllocationGoal.path_id = pPathId.toMsg();
  lAllocationGoal.allocation_interval = pAllocationInterval.toMsg();

  this->mAllocationActionClient->sendGoal(lAllocationGoal, pDoneCallback, pActiveCallback,
                                          pFeedbackCallback);

  if (!pWaitForGoal.isZero() && !this->mAllocationActionClient->waitForResult(pWaitForGoal))
  {
    this->deleteAllocationActionClient();
    return false;
  }

  this->deleteAllocationActionClient();

  return true;
}

mars::common::Result
mars::routing::common::topology::EdgeInterface::deallocateEdge(const mars::common::Id& pAgentId)
{
  mars_topology_srvs::DeallocateEntity lService;
  bool callSuccessful = false;
  ros::ServiceClient serviceClient;

  // Setup Request
  lService.request.agent_id = pAgentId.toMsg();

  std::string l_service_name = sNamespace + CONTAINER_ID_PREFIX +
                               mContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC) +
                               "/deallocate";
  serviceClient = this->getDeallocServiceClient(l_service_name);
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = serviceClient.call(lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to deallocate entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);
  return mars::common::Result(lService.response.result.result,
                              lService.response.result.description);
}

boost::optional<Eigen::Vector3d> mars::routing::common::topology::EdgeInterface::getCoordinate()
{
  mars_topology_srvs::GetCoordinate lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_coordinate");

  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get coordinate for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  Eigen::Vector3d lLocation;
  lLocation(0) = lService.response.point.point.x;
  lLocation(1) = lService.response.point.point.y;
  lLocation(2) = lService.response.point.point.z;

  return lLocation;
}

boost::optional<mars::common::geometry::Footprint>
mars::routing::common::topology::EdgeInterface::getFootprint()
{
  mars_topology_srvs::GetFootprint lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_footprint");

  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get footprint for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  mars::common::geometry::Footprint lFootprint;
  for (auto& iPoint : lService.response.footprint.polygon.points)
  {
    lFootprint.push_back(Eigen::Vector2d(iPoint.x, iPoint.y));
  }

  return lFootprint;
}

boost::optional<std::vector<mars::common::TimeInterval>>
mars::routing::common::topology::EdgeInterface::getFreeTimeSlots(const ros::Time pStartTime)
{
  mars_topology_srvs::GetFreeTimeSlots lService;
  bool callSuccessful = false;

  // Setup Request
  lService.request.start_time = pStartTime;

  std::string l_service_name = constructServiceTopic("/get_free_time_slots");
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get free time slots for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  std::vector<mars::common::TimeInterval> lFreeTimeSlots;

  for (auto& iTimeInterval : lService.response.time_interval)
  {
    lFreeTimeSlots.push_back(
        mars::common::TimeInterval(iTimeInterval.start_time, iTimeInterval.duration));
  }

  return lFreeTimeSlots;
}

boost::optional<float> mars::routing::common::topology::EdgeInterface::getLength()
{
  mars_topology_srvs::GetLength lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_length");
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get free time slots for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  return lService.response.length;
}

boost::optional<mars::topology::common::TopologyEntityRestrictions>
mars::routing::common::topology::EdgeInterface::getRestrictions()
{
  mars_topology_srvs::GetRestrictions lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_restrictions");
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get restrictions for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  return mars::topology::common::TopologyEntityRestrictions(lService.response);
}

boost::optional<mars::common::Id> mars::routing::common::topology::EdgeInterface::lockEdge(
    const mars::common::Id& pInitiatorId, const mars::common::TimeInterval& pTimeInterval,
    const std::string& pReason)
{
  mars_topology_srvs::LockTopologyEntity lService;

  // Setup Request
  lService.request.entity_id = this->mEdgeId.toMsg();
  lService.request.initiator_id.uuid = pInitiatorId.getUUID();
  lService.request.initiator_id.description = pInitiatorId.getDescription();
  lService.request.time_interval.start_time = pTimeInterval.getStartTime();
  lService.request.time_interval.duration = pTimeInterval.getDuration();
  lService.request.reason = pReason;

  std::string l_service_name = constructServiceTopic("/lock");

  if (ros::service::call(l_service_name, lService))
  {
    return mars::common::Id(lService.response.lock_id.uuid, lService.response.lock_id.description);
  }
  // If the service call was not possible
  else
  {
    MARS_LOG_ERROR("Service " << l_service_name << " is not reachable");

    return boost::none;
  }
}

boost::optional<bool>
mars::routing::common::topology::EdgeInterface::unlockEdge(const mars::common::Id& pLockId,
                                                           const std::string pReason)
{
  mars_topology_srvs::UnlockTopologyEntity lService;

  // Setup Request
  lService.request.entity_id = this->mEdgeId.toMsg();
  lService.request.lock_id.uuid = pLockId.getUUID();
  lService.request.lock_id.description = pLockId.getDescription();
  lService.request.reason = pReason;

  std::string l_service_name = constructServiceTopic("/unlock");

  if (ros::service::call(l_service_name, lService))
  {
    return true;
  }
  // If the service call was not possible
  else
  {
    MARS_LOG_ERROR("Service " << l_service_name << " is not reachable");

    return boost::none;
  }
}

boost::optional<std::vector<mars::routing::common::topology::Connection>>
mars::routing::common::topology::EdgeInterface::getConnections()
{
  mars_topology_srvs::GetConnections lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_connections");
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get restrictions for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  std::vector<mars::routing::common::topology::Connection> lConnections;

  for (auto& iConnection : lService.response.connections)
  {
    lConnections.push_back(mars::routing::common::topology::Connection(iConnection));
  }

  return lConnections;
}

boost::optional<mars::routing::common::topology::Status>
mars::routing::common::topology::EdgeInterface::getStatus()
{
  mars_topology_srvs::GetStatus lService;
  bool callSuccessful = false;

  std::string l_service_name = constructServiceTopic("/get_status");
  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    callSuccessful = ros::service::call(l_service_name, lService);
    if (!callSuccessful)
    {
      MARS_LOG_ERROR("Tried to get restrictions for entity: " +
                     this->mEdgeId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
                     " \nService " + l_service_name + " is not reachable. Retry in 1 second.");
      sleep(1);
    }

  } while (!callSuccessful);

  std::vector<mars::topology::common::TopologyEntityLock> lLocks;
  std::vector<mars::topology::common::TopologyEntityReservation> lReservations;

  for (auto& iLock : lService.response.locks)
  {
    mars::topology::common::TopologyEntityLock lLock(
        mars::common::Id(iLock.initiator_id.uuid, iLock.initiator_id.description), iLock.reason,
        ros::Time(iLock.time_interval.start_time), ros::Duration(iLock.time_interval.duration));

    lLocks.push_back(lLock);
  }

  for (auto& iReservation : lService.response.reservations)
  {
    mars::topology::common::TopologyEntityReservation lReservation(
        mars::common::Id(iReservation.agent_id.uuid, iReservation.agent_id.description),
        mars::common::Id(iReservation.path_id.uuid, iReservation.path_id.description),
        mars::common::TimeInterval(ros::Time(iReservation.time_interval.start_time),
                                   ros::Duration(iReservation.time_interval.duration)));

    lReservations.push_back(lReservation);
  }

  return mars::routing::common::topology::Status(lLocks, lReservations);
}

const std::string mars::routing::common::topology::EdgeInterface::constructServiceTopic(
    const std::string& pServiceName)
{
  if (this->mContainerId.isValid())
  {
    if (pServiceName.front() == '/')
    {
      return (sNamespace + CONTAINER_ID_PREFIX +
              mContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC) + pServiceName);
    }
    else
    {
      return (sNamespace + CONTAINER_ID_PREFIX +
              mContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC) + "/" +
              pServiceName);
    }
  }
  else
  {
    throw mars::common::exception::NotInitializedException(
        "The container id of the entity with id: \n" +
        this->mEdgeId.getUUIDAsString(mars::common::Id::HEXDEC_SPLIT) + "\n is not set.");
  }
}

void mars::routing::common::topology::EdgeInterface::initContainerId()
{
  const std::string lServiceName = REQUEST_CONTAINER_ID_SERVICE;
  mars_topology_srvs::GetTopologyEntityContainerId lService;
  bool lLookupSuccesful = false;

  lService.request.entity_id = this->mEdgeId.toMsg();

  do
  {
    lLookupSuccesful = ros::service::call(lServiceName, lService);
    lLookupSuccesful = lLookupSuccesful &&
                       (lService.response.result.result != mars_common_msgs::Result::RESULT_ERROR);

    if (lLookupSuccesful)
    {
      mContainerId = mars::common::Id(lService.response.container_id);
    }
    else
    {
      // If the service call was not possible
      MARS_LOG_ERROR("Service " << lServiceName << " is not reachable. \n Retry in 2 seconds.");
      ros::Duration(2).sleep();
    }
  } while (!lLookupSuccesful);
}

ros::ServiceClient mars::routing::common::topology::EdgeInterface::getDeallocServiceClient(std::string &serviceName)
{
  std::map<std::string, ros::ServiceClient>::iterator it = this->mDeallocServiceClientMap.find(serviceName);
  ros::ServiceClient lServiceClient;

  if (it != this->mDeallocServiceClientMap.end())
  {
    lServiceClient = it->second;
  }
  else
  {
    ros::NodeHandle nh;
    lServiceClient = nh.serviceClient<mars_topology_srvs::DeallocateEntity>(serviceName);

    this->mDeallocServiceClientMap.insert(std::make_pair(serviceName, lServiceClient));
  } 

  return lServiceClient;
}

void mars::routing::common::topology::EdgeInterface::deleteAllocationActionClient()
{
  delete this->mAllocationActionClient;
  this->mAllocationActionClient = nullptr;
}

std::string mars::routing::common::topology::EdgeInterface::sNamespace("/mars/topology/");
std::mutex mars::routing::common::topology::EdgeInterface::sMutex;
