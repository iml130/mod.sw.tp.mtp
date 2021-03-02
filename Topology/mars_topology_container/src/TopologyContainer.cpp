//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//


#include "mars_topology_container/TopologyContainer.h"
#include "mars_topology_msgs/TopologyEntityRegistration.h"

static const std::string NODE_NAME = "mars_topology_container";

static const int NODE_RATE_MAX_ROS_PARAM = -1;
static const int NODE_RATE_MAX = 50;

static const std::string LOG_LEVEL_DEBUG = "debug";
static const std::string LOG_LEVEL_INFO = "info";
static const std::string LOG_LEVEL_WARN = "warn";
static const std::string LOG_LEVEL_ERROR = "error";
static const std::string LOG_LEVEL_FATAL = "fatal";

static const std::string STD_NODE_RATE_PARAM_NAME = "node_rate";
static const int STD_NODE_RATE_PARAM = 5;

static const std::string PARAM_NAME_VISUALIZATION_RATE = "visualization_rate";
static const float PARAM_DEFAULT_VISUALIZATION_RATE = 0.1;

static const std::string DEFAULT_DESCRIPTION = "MARS Container";
static const int DEFAULT_TYPE = 0;
static const std::string DEFAULT_FRAME_ID = "map";

static const std::string PARAM_NAME_DEFAULT_CONTAINER = "container";
static const std::string PARM_NAME_CONTAINER_ID = "container_id";
static const std::string PARM_NAME_CONTAINER_DESCRIPTION = "container_description";
static const std::string PARAM_NAME_LOG_LEVEL = "log_level";
static const std::string PARAM_DEFAULT_LOG_LEVEL = LOG_LEVEL_DEBUG;

static const std::string PARAM_NAME_SERVICE_NAME_GET_COORDINATE = "get_coordinate";
static const std::string PARAM_NAME_SERVICE_NAME_GET_FOOTPRINT = "get_footprint";
static const std::string PARAM_NAME_SERVICE_NAME_LOCK = "lock";
static const std::string PARAM_NAME_SERVICE_NAME_UNLOCK = "unlock";
static const std::string PARAM_NAME_SERVICE_NAME_ADD_RESERVATION = "add_reservation";
static const std::string PARAM_NAME_SERVICE_NAME_DEALLOCATE = "deallocate";
static const std::string PARAM_NAME_SERVICE_NAME_DELETE_RESERVATION = "delete_"
                                                                      "reservation";
static const std::string PARAM_NAME_SERVICE_NAME_GET_FREE_TIME_SLOTS = "get_free_time_"
                                                                       "slots";
static const std::string PARAM_NAME_SERVICE_NAME_GET_INGOING_EDGES = "get_ingoing_"
                                                                     "edges";
static const std::string PARAM_NAME_SERVICE_NAME_GET_OUTGOING_EDGES = "get_outgoing_"
                                                                      "edges";
static const std::string PARAM_NAME_SERVICE_NAME_GET_LENGTH = "get_length";
static const std::string PARAM_NAME_SERVICE_NAME_GET_CONNECTIONS = "get_connections";
static const std::string PARAM_NAME_SERVICE_NAME_GET_RESTRICTIONS = "get_restrictions";
static const std::string PARAM_NAME_SERVICE_NAME_GET_TYPE = "get_type";
static const std::string PARAM_NAME_SERVICE_NAME_GET_STATUS = "get_status";
static const std::string PARAM_NAME_SERVICE_NAME_GET_FULL_STATUS = "get_full_status";

static const std::string PARAM_NAME_ACTION_NAME_ALLOCATE = "action_name_allocate";

// service server names
static const std::string SERVICE_NAME_DEFAULT_GET_COORDINATE = "get_coordinate";
static const std::string SERVICE_NAME_DEFAULT_GET_FOOTPRINT = "get_footprint";
static const std::string SERVICE_NAME_DEFAULT_GET_LENGTH = "get_length";
static const std::string SERVICE_NAME_DEFAULT_LOCK = "lock";
static const std::string SERVICE_NAME_DEFAULT_UNLOCK = "unlock";
static const std::string SERVICE_NAME_DEFAULT_ADD_RESERVATION = "add_reservation";
static const std::string SERVICE_NAME_DEFAULT_DEALLOCATE = "deallocate";
static const std::string SERVICE_NAME_DEFAULT_DELETE_RESERVATION = "delete_reservation";
static const std::string SERVICE_NAME_DEFAULT_GET_FREE_TIME_SLOTS = "get_free_time_slots";
static const std::string SERVICE_NAME_DEFAULT_GET_INGOING_EDGES = "get_ingoing_edges";
static const std::string SERVICE_NAME_DEFAULT_GET_OUTGOING_EDGES = "get_outgoing_edges";
static const std::string SERVICE_NAME_DEFAULT_GET_CONNECTIONS = "get_connections";
static const std::string SERVICE_NAME_DEFAULT_GET_RESTRICTIONS = "get_restrictions";
static const std::string SERVICE_NAME_DEFAULT_GET_TYPE = "get_type";
static const std::string SERVICE_NAME_DEFAULT_GET_STATUS = "get_status";
static const std::string SERVICE_NAME_DEFAULT_GET_FULL_STATUS = "get_full_status";

// action server names
static const std::string ACTION_NAME_DEFAULT_ALLOCATE = "allocate";

static const std::string PARAM_NAME_TOPIC_NAME_VISUALIZATION_DESCRIPTION =
    "visualization_description";
static const std::string PARAM_NAME_TOPIC_NAME_VISUALIZATION_FOOTPRINT =
    "visualization_footprint_array";
static const std::string PARAM_NAME_TOPIC_NAME_VISUALIZATION_HEATMAP = "visualization_heatmap";

// restriction parameters
static const std::string PARAM_NAME_MAXIMUM_LINEAR_VELOCITY_RESTRICTION = "maximum_linear_velocity";
static const std::string PARAM_NAME_MAXIMUM_ANGULAR_VELOCITY_RESTRICTION =
    "maximum_angular_velocity";
static const std::string PARAM_NAME_MAXIMUM_LINEAR_ACCELERATION_RESTRICTION =
    "maximum_linear_acceleration";
static const std::string PARAM_NAME_MAXIMUM_ANGULAR_ACCELERATION_RESTRICTION =
    "maximum_angular_acceleration";
static const std::string PARAM_NAME_MAXIMUM_HEIGHT_RESTRICTION = "maximum_height";
static const std::string PARAM_NAME_MAXIMUM_TOTAL_WEIGHT_RESTRICTION = "maximum_total_weight";
static const std::string PARAM_NAME_FORBIDDEN_VEHICLE_TYPES_RESTRICTION = "forbidden_vehicle_types";
static const std::string PARAM_NAME_FORBIDDEN_HAZARD_TYPES_RESTRICTION = "forbidden_hazard_types";
static const double PARAM_DEFAULT_MAXIMUM_LINEAR_VELOCITY_RESTRICTION = 10;       // m/s
static const double PARAM_DEFAULT_MAXIMUM_ANGULAR_VELOCITY_RESTRICTION = 1;       // rad/s
static const double PARAM_DEFAULT_MAXIMUM_LINEAR_ACCELERATION_RESTRICTION = 1;    // m/s²
static const double PARAM_DEFAULT_MAXIMUM_ANGULAR_ACCELERATION_RESTRICTION = 0.5; // rad/s²
static const double PARAM_DEFAULT_MAXIMUM_HEIGHT_RESTRICTION = 2;                 // m
static const double PARAM_DEFAULT_MAXIMUM_TOTAL_WEIGHT_RESTRICTION = 200;         // kg

// topic names
static const std::string TOPIC_NAME_DEFAULT_VISUALIZATION_DESCRIPTION = "visualization_description";
static const std::string TOPIC_NAME_DEFAULT_VISUALIZATION_FOOTPRINT = "visualization_footprint";
static const std::string TOPIC_NAME_DEFAULT_VISUALIZATION_HEATMAP = "visualization_heatmap";
static const std::string TOPIC_NAME_DEFAULT_VISUALIZATION_DIRECTION = "visualization_direction";
static const std::string TOPIC_NAME_DEFAULT_REGISTER_CONTAINER = "register_container";

// service return messages
static const std::string RESULT_STRING_ADDED_LOCK_SUCCESSFUL = "Added lock successfully.";
static const std::string RESULT_STRING_REMOVED_LOCK_SUCCESSFUL = "Removed lock successfully.";
static const std::string RESULT_STRING_REMOVED_LOCK_UNSUCCESSFUL = "Unlock mars entity not "
                                                                   "successful.";
static const std::string RESULT_STRING_ADDED_RESERVATION_SUCCESSFUL = "Added reservation "
                                                                      "successfully.";
static const std::string RESULT_STRING_ADDED_RESERVATION_UNSUCCESSFUL = "Reservation of entity was "
                                                                        "not successful.";
static const std::string RESULT_STRING_DELETED_RESERVATION_SUCCESSFUL = "Deleted reservation "
                                                                        "successfully.";
static const std::string RESULT_STRING_DELETED_RESERVATION_UNSUCCESSFUL = "Delete reservation from "
                                                                          "entity was not "
                                                                          "successful. Reservation "
                                                                          "does not exist.";
static const std::string RESULT_STRING_ALLOCATION_SUCCESSFUL = "Allocated entity successfully.";
static const std::string RESULT_STRING_ALLOCATION_UNSUCCESSFUL = "Allocation of entity denied. "
                                                                 "entity is currently "
                                                                 "anavailable for traversing.";
static const std::string RESULT_STRING_DEALLOCATION_SUCCESSFUL = "Deallocated entity successfully.";
static const std::string RESULT_STRING_DEALLOCATION_UNSUCCESSFUL = "Deallocation of entity not "
                                                                   "possible. It requires prior "
                                                                   "allocation.";
static const std::string RESULT_STRING_CANCEL_ALLOCATION_SUCCESSFUL =
    "Cancel Allocation entity successfully.";
static const std::string RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL =
    "Cancel Allocation of entity not "
    "possible. It requires prior "
    "allocation request.";

// overload the << operator for std::vector to print vector elements
template <class T> std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << "[";

  for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    if (it == v.begin())
    {
      os << *it;
    }
    else
    {
      os << ", " << *it;
    }
  }

  os << "]";

  return os;
}

mars::topology::container::TopologyContainer::TopologyContainer()
{
  this->mNHPriv = ros::NodeHandle("~");
}
mars::topology::container::TopologyContainer::~TopologyContainer() {}

template <typename T>
T mars::topology::container::TopologyContainer::getParam(const ros::NodeHandle& nH,
                                                         const std::string& paramName,
                                                         const T& defaultValue) const
{
  T value;
  if (nH.getParam(paramName, value))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName << ", value: " << value);
    return value;
  }
  else
  {
    ROS_INFO_STREAM("Cannot find value for parameter: " << paramName
                                                        << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}

template <typename T>
void mars::topology::container::TopologyContainer::getParam(const ros::NodeHandle& nH,
                                                            const std::string& paramName,
                                                            T& paramValue) const noexcept(false)
{

  if ((nH.getParam(paramName, paramValue)))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName << ", value: " << paramValue);
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + paramName);
  }
}

void mars::topology::container::TopologyContainer::processAllocationGoals(
    const mars::common::Id& pEntity_id) const
{
  this->mMutexProcessAllocation.lock();
  {
    mars::topology::common::TopologyEntity* lEntity = this->mContainer.getEntity(pEntity_id).get();
    mars_topology_actions::AllocateEntityActionFeedback lAllocationFeedback;
    lAllocationFeedback.header.stamp = ros::Time::now();

    size_t lPosition = 0;

    // iterate over all current reservations
    for (const auto& iReservation : lEntity->getSortedReservations())
    {
      // reservation has allocation request
      if (iReservation->getAllocationGoal())
      {
        // check if current reservation is the first
        if (lPosition == 0 && lEntity->getAllocation() == nullptr)
        {
          // send result (allocate entity)
          mars_topology_actions::AllocateEntityResult lAllocationResult;
          lAllocationResult.agent_id = iReservation->getAgentId().toMsg();
          lAllocationResult.path_id = iReservation->getPathId().toMsg();
          lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
          lAllocationResult.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
          lAllocationResult.result.description = RESULT_STRING_ALLOCATION_SUCCESSFUL;

          if (lEntity->getAllocation() != nullptr)
          {
            ROS_FATAL_STREAM("Gave allocation to another agent while the current allocation was "
                             "not cleared yet!!");
          }

          lEntity->setAllocation(iReservation);

          // check if goal handle is already succeeded to avoid error msgs
          if (static_cast<unsigned int>(
                  iReservation->getAllocationGoal()->getGoalStatus().status) !=
              actionlib_msgs::GoalStatus::SUCCEEDED)
          {
            ROS_INFO_STREAM("Entity Id: " << pEntity_id.getUUIDAsString(
                                                 mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                          << " Allocation successful for Agent id: "
                                          << iReservation->getAgentId().getUUIDAsString(
                                                 mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                          << "\n");
            iReservation->getAllocationGoal()->setSucceeded(lAllocationResult,
                                                            RESULT_STRING_ALLOCATION_SUCCESSFUL);
          }
        }
        else
        {
          // send feedback
          mars_topology_actions::AllocateEntityFeedback lAllocationActionFeedback;
          lAllocationActionFeedback.agent_id = iReservation->getAgentId().toMsg();
          lAllocationActionFeedback.path_id = iReservation->getPathId().toMsg();
          lAllocationActionFeedback.position_in_queue = lPosition;
          lAllocationActionFeedback.topology_entity_id = lEntity->getId().toMsg();

          iReservation->getAllocationGoal()->publishFeedback(lAllocationActionFeedback);
        }
      }
      lPosition++;
    }
  }
  this->mMutexProcessAllocation.unlock();
}

bool mars::topology::container::TopologyContainer::actionGoalCallbackAllocateEntity(
    const actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>& pAllocationGoal)
{
  mars::topology::common::TopologyEntity* lEntity =
      this->mContainer.getEntity(mars::common::Id(pAllocationGoal.getGoal()->topology_entity_id))
          .get();
  // Find reservation to the allocation request
  const auto lReservations =
      lEntity->getReservations().find(std::make_pair<mars::common::Id, mars::common::Id>(
          mars::common::Id(pAllocationGoal.getGoal()->agent_id),
          mars::common::Id(pAllocationGoal.getGoal()->path_id)));

  if (lReservations == lEntity->getReservations().end())
  {
    actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction> lAllocationGoal =
        pAllocationGoal;

    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_ERROR;
    lAllocationResult.result.description = RESULT_STRING_ALLOCATION_UNSUCCESSFUL;

    lAllocationGoal.setRejected(lAllocationResult, RESULT_STRING_ALLOCATION_UNSUCCESSFUL);
    return true;
  }

  // There is at least one reservation for given agent and route on this entity

  mars::common::TimeInterval lAllocationInterval(pAllocationGoal.getGoal()->allocation_interval);

  std::vector<mars::topology::common::TopologyEntityReservation*>::const_iterator
      lCorrespondingReservation = std::find_if(
          lReservations->second.begin(), lReservations->second.end(),
          [lAllocationInterval](mars::topology::common::TopologyEntityReservation* pReservation) {
            return pReservation->getTimeInterval() == lAllocationInterval;
          });

  if (lCorrespondingReservation == lReservations->second.end())
  {
    ROS_ERROR("[mars::topology::container::TopologyContainer::actionGoalCallbackAllocateEntity] No "
              "reservation matches the allocation time interval.");
    actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction> lAllocationGoal =
        pAllocationGoal;

    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_ERROR;
    lAllocationResult.result.description = RESULT_STRING_ALLOCATION_UNSUCCESSFUL;

    lAllocationGoal.setRejected(lAllocationResult, RESULT_STRING_ALLOCATION_UNSUCCESSFUL);
    return true;
  }

  if ((lEntity->getAllocation() != nullptr) &&
      (lEntity->getAllocation()->getAgentId() == mars::common::Id(pAllocationGoal.getGoal()->agent_id)) &&
      (lEntity->getAllocation()->getPathId() == mars::common::Id(pAllocationGoal.getGoal()->path_id)))
  {
    actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction> lAllocationGoal =
        pAllocationGoal;
    
    lAllocationGoal.setAccepted();

    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    lAllocationResult.result.description = RESULT_STRING_ALLOCATION_SUCCESSFUL;

    lAllocationGoal.setSucceeded(lAllocationResult, RESULT_STRING_ALLOCATION_SUCCESSFUL);
    return true;
  }

  // There is a reservation corresponding to the allocation request

  (*lCorrespondingReservation)->requestAllocation(pAllocationGoal);
  this->processAllocationGoals(mars::common::Id(pAllocationGoal.getGoal()->topology_entity_id));

  return true;
}

bool mars::topology::container::TopologyContainer::actionCancelCallbackAllocateEntity(
    const actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>& pAllocationGoal)
{
  mars::topology::common::TopologyEntity* lEntity =
      this->mContainer.getEntity(mars::common::Id(pAllocationGoal.getGoal()->topology_entity_id))
          .get();
  const auto lReservations =
      lEntity->getReservations().find(std::make_pair<mars::common::Id, mars::common::Id>(
          mars::common::Id(pAllocationGoal.getGoal()->agent_id),
          mars::common::Id(pAllocationGoal.getGoal()->path_id)));

  actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction> lAllocationGoal =
      pAllocationGoal;

  if (lReservations == lEntity->getReservations().end())
  {

    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_ERROR;
    lAllocationResult.result.description = RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL;

    lAllocationGoal.setRejected(lAllocationResult, RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL);
    return true;
  }
  // There is at least one reservation for given agent and route on this entity

  mars::common::TimeInterval lAllocationInterval(pAllocationGoal.getGoal()->allocation_interval);

  std::vector<mars::topology::common::TopologyEntityReservation*>::const_iterator
      lCorrespondingReservation = std::find_if(
          lReservations->second.begin(), lReservations->second.end(),
          [lAllocationInterval](mars::topology::common::TopologyEntityReservation* pReservation) {
            return pReservation->getTimeInterval() == lAllocationInterval;
          });

  if (lCorrespondingReservation == lReservations->second.end())
  {
    // There is no reservation for the given allocation interval
    ROS_ERROR(
        "[mars::topology::container::TopologyContainer::actionGoalCallbackDeallocateEntity] No "
        "reservation matches the deallocation time interval.");
    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_ERROR;
    lAllocationResult.result.description = RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL;

    lAllocationGoal.setRejected(lAllocationResult, RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL);
    return true;
  }
  else if (!(*lCorrespondingReservation)->getAllocationGoal())
  {
    // There is a matching reservation, but that had not been allocated before
    ROS_ERROR("[mars::topology::container::TopologyContainer::actionGoalCallbackDeallocateEntity]"
              " Reservation matches deallocation time interval, but it had not been allocated.");
    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_ERROR;
    lAllocationResult.result.description = RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL;

    lAllocationGoal.setRejected(lAllocationResult, RESULT_STRING_CANCEL_ALLOCATION_UNSUCCESSFUL);
    return true;
  }
  else
  {
    // There is a matching reservation, which had been allocated before
    mars_topology_actions::AllocateEntityResult lAllocationResult;
    lAllocationResult.agent_id = pAllocationGoal.getGoal()->agent_id;
    lAllocationResult.path_id = pAllocationGoal.getGoal()->path_id;
    lAllocationResult.topology_entity_id = lEntity->getId().toMsg();
    lAllocationResult.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    lAllocationResult.result.description = RESULT_STRING_CANCEL_ALLOCATION_SUCCESSFUL;

    (*lCorrespondingReservation)
        ->getAllocationGoal()
        ->setCanceled(lAllocationResult, RESULT_STRING_CANCEL_ALLOCATION_SUCCESSFUL);
  }

  (*lCorrespondingReservation)->requestAllocation(pAllocationGoal);

  this->processAllocationGoals(pAllocationGoal.getGoal()->topology_entity_id);

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackDeallocateEntity(
    mars_topology_srvs::DeallocateEntity::Request& req,
    mars_topology_srvs::DeallocateEntity::Response& res)
{
  mars::topology::common::TopologyEntity* lEntity =
      this->mContainer.getEntity(mars::common::Id(req.entity_id)).get();
  mars::topology::common::TopologyEntityReservation* lAllocation = lEntity->getAllocation();
  std::pair<mars::common::Id, mars::common::Id> reservationIdPair;

  if (lAllocation == nullptr)
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    if (lEntity->checkReservationAlreadyDeleted(mars::common::Id(req.agent_id), reservationIdPair))
    {
      res.result.description =
          "Reservation maybe already deleted. Last deleted reservation for agent " +
          reservationIdPair.first.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
          " with path id " +
          reservationIdPair.second.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);
    }
    else
    {
      res.result.description = RESULT_STRING_DEALLOCATION_UNSUCCESSFUL;
    }
    return true;
  }

  if (lAllocation->getAgentId() != req.agent_id)
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    if (lEntity->checkReservationAlreadyDeleted(mars::common::Id(req.agent_id), reservationIdPair))
    {
      res.result.description =
          "Reservation maybe already deleted. Last deleted reservation for agent " +
          reservationIdPair.first.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
          " with path id " +
          reservationIdPair.second.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);
    }
    else
    {
      res.result.description = RESULT_STRING_DEALLOCATION_UNSUCCESSFUL;
    }
    return true;
  }

  if (lEntity->deleteReservation(lAllocation->getAgentId(), lAllocation->getPathId()))
  {
    res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    res.result.description = RESULT_STRING_DEALLOCATION_SUCCESSFUL;

    MARS_LOG_DEBUG("Entity id: " << lEntity->getId().getUUIDAsString(
                                        mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                 << " Deallocation for agent id: "
                                 << lAllocation->getAgentId().getUUIDAsString(
                                        mars::common::Id::UUIDFormat::HEXDEC_SPLIT)
                                 << "\n");
    lEntity->setAllocation(nullptr);
    this->processAllocationGoals(req.entity_id);
  }
  else
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    if (lEntity->checkReservationAlreadyDeleted(mars::common::Id(req.agent_id), reservationIdPair))
    {
      res.result.description =
          "Reservation maybe already deleted. Last deleted reservation for agent " +
          reservationIdPair.first.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) +
          " with path id " +
          reservationIdPair.second.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);
    }
    else
    {
      res.result.description = RESULT_STRING_DEALLOCATION_UNSUCCESSFUL;
    }
  }

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetCoordinate(
    mars_topology_srvs::GetCoordinate::Request& req,
    mars_topology_srvs::GetCoordinate::Response& res)
{
  res.point = this->mContainer.getEntity(mars::common::Id(req.entity_id))->getCoordinate();

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetFootprint(
    mars_topology_srvs::GetFootprint::Request& req, mars_topology_srvs::GetFootprint::Response& res)
{
  res.footprint = this->mContainer.getEntity(mars::common::Id(req.entity_id))->getFootprint();

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackLockTopologyEntity(
    mars_topology_srvs::LockTopologyEntity::Request& req,
    mars_topology_srvs::LockTopologyEntity::Response& res)
{
  mars::topology::common::TopologyEntityLock entityLock;
  bool addLockSuccessfully = true;

  try
  {
    entityLock =
        this->mContainer.getEntity(mars::common::Id(req.entity_id))
            ->addLock(mars::common::Id(req.initiator_id.uuid, req.initiator_id.description),
                      req.reason, req.time_interval.start_time, req.time_interval.duration,
                      req.overwrite_reservations);

    res.lock_id.uuid = entityLock.getId().getUUID();
    res.lock_id.description = entityLock.getId().getDescription();

    res.confirmed_time_interval.start_time = entityLock.getTimeInterval().getStartTime();
    res.confirmed_time_interval.duration = entityLock.getTimeInterval().getDuration();

    res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    res.result.description = RESULT_STRING_ADDED_LOCK_SUCCESSFUL;
  }
  catch (mars::common::exception::SetParamException& e)
  {
    MARS_LOG_ERROR("Can't create lock: " << e.what());

    addLockSuccessfully = false;
  }

  return addLockSuccessfully;
}

bool mars::topology::container::TopologyContainer::serviceCallbackUnlockTopologyEntity(
    mars_topology_srvs::UnlockTopologyEntity::Request& req,
    mars_topology_srvs::UnlockTopologyEntity::Response& res)
{
  bool successfullyUnlocked =
      this->mContainer.getEntity(mars::common::Id(req.entity_id))
          ->deleteLock(mars::common::Id(req.lock_id.uuid, req.lock_id.description), req.reason);

  if (successfullyUnlocked)
  {
    res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    res.result.description = RESULT_STRING_REMOVED_LOCK_SUCCESSFUL;
  }
  else
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    res.result.description = RESULT_STRING_REMOVED_LOCK_UNSUCCESSFUL;
  }

  return successfullyUnlocked;
}

bool mars::topology::container::TopologyContainer::serviceCallbackAddReservation(
    mars_topology_srvs::AddReservation::Request& req,
    mars_topology_srvs::AddReservation::Response& res)
{
  bool successfulAddedReservation = true;

  try
  {
    if (this->mContainer.getEntity(mars::common::Id(req.entity_id))
            ->addReservation(
                mars::common::Id(req.reservation.agent_id.uuid,
                                 req.reservation.agent_id.description),
                mars::common::Id(req.reservation.path_id.uuid, req.reservation.path_id.description),
                mars::common::TimeInterval(req.reservation.time_interval.start_time,
                                           req.reservation.time_interval.duration)))
    {
      res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
      res.result.description = RESULT_STRING_ADDED_RESERVATION_SUCCESSFUL;
    }
    else
    {
      res.result.result = mars_common_msgs::Result::RESULT_ERROR;
      res.result.description = RESULT_STRING_ADDED_RESERVATION_UNSUCCESSFUL + " Time interval "
                                                                              "overlaps with other "
                                                                              "existing time "
                                                                              "interval or lock.";
    }
  }
  catch (mars::common::exception::SetParamException& e)
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    res.result.description =
        RESULT_STRING_ADDED_RESERVATION_UNSUCCESSFUL + " Exception: " + e.what();

    successfulAddedReservation = false;
  }

  return successfulAddedReservation;
}

bool mars::topology::container::TopologyContainer::serviceCallbackDeleteReservation(
    mars_topology_srvs::DeleteReservation::Request& req,
    mars_topology_srvs::DeleteReservation::Response& res)
{
  bool successfulDeleteReservation = true;

  if (this->mContainer.getEntity(mars::common::Id(req.entity_id))
          ->deleteReservation(mars::common::Id(req.agent_id.uuid, req.agent_id.description),
                              mars::common::Id(req.path_id.uuid, req.path_id.description)))
  {
    res.result.result = mars_common_msgs::Result::RESULT_SUCCESS;
    res.result.description = RESULT_STRING_DELETED_RESERVATION_SUCCESSFUL;
  }
  else
  {
    res.result.result = mars_common_msgs::Result::RESULT_ERROR;
    res.result.description = RESULT_STRING_DELETED_RESERVATION_UNSUCCESSFUL;
  }

  return successfulDeleteReservation;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetFreeTimeSlots(
    mars_topology_srvs::GetFreeTimeSlots::Request& req,
    mars_topology_srvs::GetFreeTimeSlots::Response& res)
{
  ros::Time lStartTime(req.start_time);

  const std::vector<mars::common::TimeInterval>& lFreeTimeSlots(
      this->mContainer.getEntity(mars::common::Id(req.entity_id))->getFreeTimeSlots(lStartTime));

  res.time_interval.reserve(lFreeTimeSlots.size());

  for (auto& iTimeInterval : lFreeTimeSlots)
  {
    res.time_interval.push_back(iTimeInterval.toMsg());
  };

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetLength(
    mars_topology_srvs::GetLength::Request& req, mars_topology_srvs::GetLength::Response& res)
{
  res.length = this->mContainer.getEdge(mars::common::Id(req.entity_id))->getLength();
  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetConnections(
    mars_topology_srvs::GetConnections::Request& req,
    mars_topology_srvs::GetConnections::Response& res)
{
  bool successfullyFoundConnections = true;
  try
  {

    std::shared_ptr<mars::topology::edge::MarsEdge> lEdge =
        this->mContainer.getEdge(mars::common::Id(req.entity_id));
    switch (lEdge->getDirection())
    {
    case mars::topology::common::DirectionType::unidirectional:
    {
      mars_topology_msgs::Connection lConnection, lContainerConnection;

      lConnection.origin_id = lEdge->getOriginId().toMsg();
      lConnection.destination_id = lEdge->getTargetId().toMsg();
      res.connections.push_back(lConnection);
      lContainerConnection.origin_id = lEdge->getOriginContainer().toMsg();
      lContainerConnection.destination_id = lEdge->getTargetContainer().toMsg();
      res.container_connections.push_back(lContainerConnection);

      break;
    }
    case mars::topology::common::DirectionType::unidirectional_reversed:
    {
      mars_topology_msgs::Connection lConnection, lContainerConnection;
      lConnection.origin_id = lEdge->getTargetId().toMsg();
      lConnection.destination_id = lEdge->getOriginId().toMsg();
      res.connections.push_back(lConnection);
      lContainerConnection.destination_id = lEdge->getOriginContainer().toMsg();
      lContainerConnection.origin_id = lEdge->getTargetContainer().toMsg();
      res.container_connections.push_back(lContainerConnection);
      break;
    }
    // bidirectional edge means two simple and opposite directions
    case mars::topology::common::DirectionType::bidirectional:
    {
      // at first adds the vertex ids as connections
      mars_topology_msgs::Connection lConnection1, lConnection2, lContainerConnection1,
          lContainerConnection2;
      lConnection1.origin_id = lEdge->getOriginId().toMsg();
      lConnection1.destination_id = lEdge->getTargetId().toMsg();
      lConnection2.origin_id = lEdge->getTargetId().toMsg();
      lConnection2.destination_id = lEdge->getOriginId().toMsg();
      res.connections.push_back(lConnection1);
      res.connections.push_back(lConnection2);
      lContainerConnection1.origin_id = lEdge->getOriginContainer().toMsg();
      lContainerConnection1.destination_id = lEdge->getTargetContainer().toMsg();
      lContainerConnection2.origin_id = lContainerConnection1.destination_id;
      lContainerConnection2.destination_id = lContainerConnection1.origin_id;
      res.container_connections.push_back(lContainerConnection1);
      res.container_connections.push_back(lContainerConnection2);

      break;
    }
    default:
      break;
    }
  }
  catch (mars::common::exception::SetParamException e)
  {
    MARS_LOG_ERROR("Can't return the wanted connections : " << e.what());
    successfullyFoundConnections = false;
  }
  return successfullyFoundConnections;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetIngoingEdges(
    mars_topology_srvs::GetIngoingEdges::Request& req,
    mars_topology_srvs::GetIngoingEdges::Response& res)
{
  bool successfulFoundIngoingEdges = true;
  try
  {
    std::shared_ptr<mars::topology::vertex::MarsVertex> lVertex =
        this->mContainer.getVertex(req.entity_id);
    res.ingoing_edges_ids = mars::common::Id::convertToMsgId(lVertex->getIngoingEdgeIds());
    std::vector<mars::common::Id> lIds = lVertex->getIngoingContainerIds();
    std::vector<mars_common_msgs::Id> lContainerIds;
    lContainerIds.reserve(lIds.size());
    for (std::vector<mars::common::Id>::iterator i = lIds.begin(); i != lIds.end(); i++)
    {
      if (!i->isInitialized())
      {
        lContainerIds.push_back(mars_common_msgs::Id());
      }
      else
      {
        lContainerIds.push_back(i->toMsg());
      }
    }
    res.ingoing_container_ids = lContainerIds;
  }
  catch (mars::common::exception::SetParamException& e)
  {
    MARS_LOG_ERROR("Can't return the ids : " << e.what());
    successfulFoundIngoingEdges = false;
  }
  return successfulFoundIngoingEdges;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetOutgoingEdges(
    mars_topology_srvs::GetOutgoingEdges::Request& req,
    mars_topology_srvs::GetOutgoingEdges::Response& res)
{
  bool successfulFoundOutgoingEdges = true;
  try
  {
    std::shared_ptr<mars::topology::vertex::MarsVertex> lVertex =
        this->mContainer.getVertex(req.entity_id);

    res.outgoing_edges_ids = mars::common::Id::convertToMsgId(lVertex->getOutgoingEdgeIds());
    std::vector<mars::common::Id> lIds = lVertex->getOutgoingContainerIds();

    std::vector<mars_common_msgs::Id> lContainerIds;
    lContainerIds.reserve(lIds.size());
    for (std::vector<mars::common::Id>::iterator i = lIds.begin(); i != lIds.end(); i++)
    {
      if (!i->isInitialized())
      {
        lContainerIds.push_back(mars_common_msgs::Id());
      }
      else
      {
        lContainerIds.push_back(i->toMsg());
      }
    }
    res.outgoing_container_ids = lContainerIds;
  }
  catch (mars::common::exception::SetParamException& e)
  {
    MARS_LOG_ERROR("Can't return the ids : " << e.what());
    successfulFoundOutgoingEdges = false;
  }
  return successfulFoundOutgoingEdges;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetRestrictions(
    mars_topology_srvs::GetRestrictions::Request& req,
    mars_topology_srvs::GetRestrictions::Response& res)
{
  this->mContainer.getEntity(mars::common::Id(req.entity_id))
      ->getRestrictions()
      .mergeIntoServiceResponse(res);

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetType(
    mars_topology_srvs::GetType::Request& req, mars_topology_srvs::GetType::Response& res)
{
  res.entity_type.entity_type =
      this->mContainer.getEntity(mars::common::Id(req.entity_id))->getType().getType();

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetStatus(
    mars_topology_srvs::GetStatus::Request& req, mars_topology_srvs::GetStatus::Response& res)
{
  res.locks = mars::topology::common::TopologyEntityLock::convertToMsgLock(
      this->mContainer.getEntity(mars::common::Id(req.entity_id))->getLocks());
  res.reservations = mars::topology::common::TopologyEntityReservation::convertToMsgReservation(
      this->mContainer.getEntity(mars::common::Id(req.entity_id))->getReservations());

  return true;
}

bool mars::topology::container::TopologyContainer::serviceCallbackGetFullStatus(
    mars_topology_srvs::GetFullStatus::Request& req,
    mars_topology_srvs::GetFullStatus::Response& res)
{

  res.status.resize(0); // Reserve to actual size better

  for (auto iEntity : this->mContainer)
  {
    mars_topology_msgs::FullStatus lStatus;
    lStatus.entity_id = iEntity.first.toMsg();
    lStatus.point = iEntity.second->getCoordinate();
    lStatus.type.entity_type = iEntity.second->getType().getType();
    lStatus.locks =
        mars::topology::common::TopologyEntityLock::convertToMsgLock(iEntity.second->getLocks());
    lStatus.reservations =
        mars::topology::common::TopologyEntityReservation::convertToMsgReservation(
            iEntity.second->getReservations());
    res.status.push_back(lStatus);
  }

  return true;
}

bool mars::topology::container::TopologyContainer::init()
{
  bool initSuccessfully;
  std::string ns = ros::this_node::getNamespace();

  // first read launch parameter
  initSuccessfully = this->readLaunchParams();

  // if no error occurs...
  if (initSuccessfully)
  {
    // .. and continue with the initialization of the services
    initSuccessfully = this->initServices();
  }
  else
  {
    ROS_ERROR("Error while reading launch file params "
              "-> shutting down node!");
  }

  // ... if no error occurs initialize the publisher
  if (initSuccessfully)
  {
    initSuccessfully = this->initPublisher();
  }
  else
  {
    ROS_ERROR("Error while initializing node Services "
              "-> shutting down node!");
  }

  // ... if no error occurs initialize the subscriber
  if (initSuccessfully)
  {
    initSuccessfully = this->initSubscriber();
  }
  else
  {
    ROS_ERROR("Error while setting up publisher -> "
              "shutting down node!");
  }

  // ... if no error occurs initialize the action server
  if (initSuccessfully)
  {
    initSuccessfully = this->initActions();
  }
  else
  {
    ROS_ERROR("Error while setting up subscriber -> "
              "shutting down node!");
  }

  // ... if no error occurs initialize the timer
  // if (initSuccessfully)
  // {
  //   initSuccessfully = this->initTimer();
  // }
  // else
  // {
  //   MARS_LOG_ERROR("Error while setting up action server -> "
  //                  "shutting down node!");
  // }

  if (!initSuccessfully)
  {
    ROS_ERROR("Error while setting up timer "
              "shutting down node!");
  }

  return initSuccessfully;
}

bool mars::topology::container::TopologyContainer::initServices()
{
  bool initSuccessfully = true;
  try
  {
    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetCoordinate,
        this->mServiceGetCoordinate, PARAM_NAME_SERVICE_NAME_GET_COORDINATE, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetFootprint,
        this->mServiceGetFootprint, PARAM_NAME_SERVICE_NAME_GET_FOOTPRINT, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackLockTopologyEntity,
        this->mServiceLockTopologyEntity, PARAM_NAME_SERVICE_NAME_LOCK, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackUnlockTopologyEntity,
        this->mServiceUnlockTopologyEntity, PARAM_NAME_SERVICE_NAME_UNLOCK, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackAddReservation,
        this->mServiceAddReservation, PARAM_NAME_SERVICE_NAME_ADD_RESERVATION, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackDeallocateEntity,
        this->mServiceDeallocateEntity, PARAM_NAME_SERVICE_NAME_DEALLOCATE, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackDeleteReservation,
        this->mServiceDeleteReservation, PARAM_NAME_SERVICE_NAME_DELETE_RESERVATION, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetFreeTimeSlots,
        this->mServiceGetFreeTimeSlots, PARAM_NAME_SERVICE_NAME_GET_FREE_TIME_SLOTS, this);

    this->advertiseService(&mars::topology::container::TopologyContainer::serviceCallbackGetLength,
                           this->mServiceGetLength, SERVICE_NAME_DEFAULT_GET_LENGTH, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetConnections,
        this->mServiceGetConnections, SERVICE_NAME_DEFAULT_GET_CONNECTIONS, this);
    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetIngoingEdges,
        this->mServiceGetIngoingEdges, PARAM_NAME_SERVICE_NAME_GET_INGOING_EDGES, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetOutgoingEdges,
        this->mServiceGetOutgoingEdges, PARAM_NAME_SERVICE_NAME_GET_OUTGOING_EDGES, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetRestrictions,
        this->mServiceGetRestrictions, PARAM_NAME_SERVICE_NAME_GET_RESTRICTIONS, this);

    this->advertiseService(&mars::topology::container::TopologyContainer::serviceCallbackGetType,
                           this->mServiceGetType, PARAM_NAME_SERVICE_NAME_GET_TYPE, this);

    this->advertiseService(&mars::topology::container::TopologyContainer::serviceCallbackGetStatus,
                           this->mServiceGetStatus, PARAM_NAME_SERVICE_NAME_GET_STATUS, this);

    this->advertiseService(
        &mars::topology::container::TopologyContainer::serviceCallbackGetFullStatus,
        this->mServiceGetFullStatus, this->mServiceNameGetFullStatus, this);
  }
  catch (mars::common::exception::AdvertiseServiceException& e)
  {
    initSuccessfully = false;
    MARS_LOG_ERROR("(AdvertisedServiceException) "
                   "Exception occurred: "
                   << e.what());
  }
  return initSuccessfully;
}

bool mars::topology::container::TopologyContainer::initPublisher()
{
  bool initSuccessfully = true;

  this->mContainerRegistrationPublisher =
      mNH.advertise<mars_topology_msgs::TopologyEntityRegistration>(
          "/" + TOPIC_NAME_DEFAULT_REGISTER_CONTAINER, 1, true);

  // Visualization publisher
  this->mVisualizationDescriptionPublisher = this->mNH.advertise<visualization_msgs::Marker>(
      PARAM_NAME_TOPIC_NAME_VISUALIZATION_DESCRIPTION, 1);
  this->mVisualizationFootprintPublisher = this->mNH.advertise<visualization_msgs::MarkerArray>(
      PARAM_NAME_TOPIC_NAME_VISUALIZATION_FOOTPRINT, 2, true);
  this->mVisualizationHeatmapPublisher = this->mNH.advertise<visualization_msgs::Marker>(
      PARAM_NAME_TOPIC_NAME_VISUALIZATION_HEATMAP, 1);
  return initSuccessfully;
}

bool mars::topology::container::TopologyContainer::initSubscriber()
{
  bool initSuccessfully = true;

  return initSuccessfully;
}

bool mars::topology::container::TopologyContainer::initActions()
{
  bool initSuccessfully = true;

  // init AllocateEntity action
  {
    this->mActionAllocateEntity =
        new actionlib::ActionServer<mars_topology_actions::AllocateEntityAction>(
            this->mNHPriv, ACTION_NAME_DEFAULT_ALLOCATE,
            boost::bind(
                &mars::topology::container::TopologyContainer::actionGoalCallbackAllocateEntity,
                this, _1),
            boost::bind(
                &mars::topology::container::TopologyContainer::actionCancelCallbackAllocateEntity,
                this, _1),
            false);

    this->mActionAllocateEntity->start();
  }
  return initSuccessfully;
}

template <class T, class MReq, class MRes>
bool mars::topology::container::TopologyContainer::advertiseService(
    bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer, std::string serviceName,
    T* obj) noexcept(false)
{
  bool advertisedSuccessfully = true;

  try
  {
    serviceServer = this->mNHPriv.advertiseService(serviceName, srvFunc, obj);
    if (!serviceServer)
    {
      advertisedSuccessfully = false;
      throw mars::common::exception::AdvertiseServiceException("Could not advertise service: " +
                                                               serviceName);
    }
  }
  catch (const ros::InvalidNameException& e)
  {
    advertisedSuccessfully = false;
    MARS_LOG_ERROR("(InvalidNameException) "
                   "Exception occurred: "
                   << e.what());
  }

  return advertisedSuccessfully;
}

void mars::topology::container::TopologyContainer::drawMarker(const ros::TimerEvent& event)
{
  this->mContainer.draw(this->mVisualizationFootprintPublisher, DEFAULT_FRAME_ID,
                        this->mNH.getNamespace());
}

bool mars::topology::container::TopologyContainer::initTimer()
{
  this->mVisualizationTimer =
      this->mNH.createTimer(ros::Duration(1 / PARAM_DEFAULT_VISUALIZATION_RATE),
                            &mars::topology::container::TopologyContainer::drawMarker, this, true);
  return true;
}

bool mars::topology::container::TopologyContainer::readLaunchParams()
{
  bool initSuccessfully = true;

  std::string lContainerID, lContainerDescription;
  this->mNHPriv.getParam(PARM_NAME_CONTAINER_ID, lContainerID);
  if (this->mNHPriv.hasParam(PARM_NAME_CONTAINER_DESCRIPTION))
  {
    this->mNHPriv.getParam(PARM_NAME_CONTAINER_DESCRIPTION, lContainerDescription);
  }
  this->mContainer.setId(lContainerID, lContainerDescription);

  MARS_LOG_DEBUG("The container id:" << lContainerID << std::endl
                                     << "The description is:" << lContainerDescription
                                     << std::endl);
  mars::topology::container::EntityParser parser;
  XmlRpc::XmlRpcValue lTopology;
  this->mNHPriv.getParam(PARAM_NAME_DEFAULT_CONTAINER, lTopology);
  parser.parse(lTopology, mContainer);
  // read program params
  {
    this->mNodeLogLevel = getParam(this->mNHPriv, PARAM_NAME_LOG_LEVEL, PARAM_DEFAULT_LOG_LEVEL);
    // set node log level
    this->setNodeLogLevel();
  }

  this->mHz = getParam(this->mNHPriv, STD_NODE_RATE_PARAM_NAME, STD_NODE_RATE_PARAM);
  this->mHz = (this->mHz == NODE_RATE_MAX_ROS_PARAM) ? NODE_RATE_MAX : this->mHz;

  this->mServiceNameGetFullStatus = getParam(this->mNHPriv, PARAM_NAME_SERVICE_NAME_GET_FULL_STATUS,
                                             SERVICE_NAME_DEFAULT_GET_FULL_STATUS);

  return initSuccessfully;
}

bool mars::topology::container::TopologyContainer::setNodeLogLevel() const
{
  ros::console::Level nodeLogLevel;
  bool setLogLevelSuccessfully = true;

  if (this->mNodeLogLevel == LOG_LEVEL_DEBUG)
  {
    nodeLogLevel = ros::console::levels::Debug;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_INFO)
  {
    nodeLogLevel = ros::console::levels::Info;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_WARN)
  {
    nodeLogLevel = ros::console::levels::Warn;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_ERROR)
  {
    nodeLogLevel = ros::console::levels::Error;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_FATAL)
  {
    nodeLogLevel = ros::console::levels::Fatal;
  }
  else
  {
    ROS_WARN_STREAM("Wrong log level was "
                    "set in launch file! Level was '"
                    << this->mNodeLogLevel << "' but must be '" << LOG_LEVEL_DEBUG << "', '"
                    << LOG_LEVEL_INFO << "', '" << LOG_LEVEL_WARN << "', '" << LOG_LEVEL_ERROR
                    << "' or '" << LOG_LEVEL_FATAL << "'.");
    ;

    setLogLevelSuccessfully = false;
  }

  // set node log level
  if (setLogLevelSuccessfully &&
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, nodeLogLevel))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  else
  {
    ROS_WARN_STREAM("Can not set ros logger "
                    "level with level: "
                    << this->mNodeLogLevel);

    setLogLevelSuccessfully = false;
  }

  return setLogLevelSuccessfully;
}

void mars::topology::container::TopologyContainer::rosMainLoop() const
{
  ROS_INFO("Node successfully initialized. Starting node now!");

  ros::spin();
  // ros::Rate r(10);
  // while(ros::ok()){
  //   ros::spinOnce();
  //   r.sleep();
  // }
  // Stop the node's resources
  ros::shutdown();
}

void mars::topology::container::TopologyContainer::registerContainer() const
{
  mars_topology_msgs::TopologyEntityRegistration registerMsg;

  // Register to system
  registerMsg.container_id = this->mContainer.getId().toMsg();
  registerMsg.registration_action =
      mars_topology_msgs::TopologyEntityRegistration::REGISTRATION_ACTION_REGISTER;

  for (const auto& it : mContainer.getContainer())
  {
    registerMsg.contained_entity_ids.push_back(it.second->getId().toMsg());
  }

  this->mContainerRegistrationPublisher.publish(registerMsg);
}

bool mars::topology::container::TopologyContainer::runROSNode()
{
  bool runNodeSuccessfully = true;

  if (this->init())
  {
    this->registerContainer();
    this->mContainer.draw(this->mVisualizationFootprintPublisher, DEFAULT_FRAME_ID,
                          this->mNH.getNamespace());
    this->rosMainLoop();
  }
  else
  {
    runNodeSuccessfully = false;
  }

  return runNodeSuccessfully;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);

  mars::topology::container::TopologyContainer topologyContainer;

  return topologyContainer.runROSNode() ? EXIT_SUCCESS : EXIT_FAILURE;
}
