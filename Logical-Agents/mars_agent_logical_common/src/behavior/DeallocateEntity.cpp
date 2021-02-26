#include "mars_agent_logical_common/behavior/DeallocateEntity.h"
#include "mars_common/Logger.h"

#include <boost/geometry.hpp>

static const unsigned int HOLD_N_ALLOCATIONS = 1;

static const std::string PARAM_NAME_ROBOT_PROPERTIES = "robot_properties";
static const std::string PARAM_NAME_AGENT_ID = "agent_id";
static const std::string PARAM_NAME_PHYSICAL_AGENT_NAMESPACE = "physical_agent_namespace";
static const std::string PARAM_NAME_ALLOCATION_COUNT = "allocation_count";
static const std::string PARAM_NAME_ROUTE_DEALLOCATE_STEP = "route_deallocate_step";
static const std::string PARAM_NAME_ROBOT_POSE = "robot_pose";
static const std::string PARAM_NAME_ROUTE_ID = "route_id";

static const std::string PARAM_NAME_CURRENT_TOPOLOGY_ENTITY = "current_topology_entity";

mars::agent::logical::common::behavior::DeallocateEntity::DeallocateEntity(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::CoroActionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::DeallocateEntity::providedPorts()
{
  return {
      BT::InputPort<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
          PARAM_NAME_ROBOT_PROPERTIES),
      BT::InputPort<mars::common::Id>(PARAM_NAME_AGENT_ID),
      BT::InputPort<mars::common::Id>(PARAM_NAME_ROUTE_ID),
      BT::InputPort<std::string>(PARAM_NAME_PHYSICAL_AGENT_NAMESPACE),
      BT::InputPort<Eigen::Affine3d>(PARAM_NAME_ROBOT_POSE),
      BT::OutputPort<std::shared_ptr<mars::routing::common::topology::Entity>>(
          PARAM_NAME_CURRENT_TOPOLOGY_ENTITY),
      BT::BidirectionalPort<unsigned int>(PARAM_NAME_ALLOCATION_COUNT),
      BT::BidirectionalPort<mars::routing::core::IterationStep*>(PARAM_NAME_ROUTE_DEALLOCATE_STEP)};
}

BT::NodeStatus mars::agent::logical::common::behavior::DeallocateEntity::tick()
{
  Eigen::Vector2d robotCoordinate2D;
  bool canDeallocateEntity = false;
  bool deallocationSuccessful = false;

  this->getParams();

  // test if we could deallocate based on spatial conditions. Wait and test
  // again if we currently can not deallocate.
  while (!canDeallocateEntity)
  {
    this->updateRobotPose();
    robotCoordinate2D = this->getRobotCoordinate();

    canDeallocateEntity =
        this->mDeallocateStep->canDeallocateOrigin(robotCoordinate2D, *this->mRobotProperties);

    this->setStatusRunningAndYield();
  }

  // deallocate only if we have at least another topology entity allocated. We
  // should not free the entity on which the robot currently stands to avoid
  // possible crashes.
  this->updateAllocationCount();

  while (this->mAllocationCount <= HOLD_N_ALLOCATIONS)
  {
    MARS_LOG_WARN("The vehicle with the Id: "
                  << this->mAgentId.value().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC)
                  << " \n only holds " << this->mAllocationCount
                  << " allocations, but it is requiered to at least hold " << HOLD_N_ALLOCATIONS
                  << " at all times.");

    this->updateAllocationCount();

    this->setStatusRunningAndYield();
  }

  do
  {
    deallocationSuccessful = this->mDeallocateStep->getOrigin().deallocate(this->mAgentId.value(),
                                                                           this->mRouteId.value());
    this->setStatusRunningAndYield();
  } while (!deallocationSuccessful);

  // update again to counter a race condition with the allocation. It could
  // update the value while we waited in the loop.
  this->updateAllocationCount();
  this->mAllocationCount--;

  if (!this->setParams())
  {
    // at least one param could not be set
    MARS_LOG_WARN("At least one parameter could not be set.");
  }

  return BT::NodeStatus::SUCCESS;
}

void mars::agent::logical::common::behavior::DeallocateEntity::halt()
{
  std::cout << name() << ": Halted." << std::endl;

  // Do not forget to call this at the end.
  CoroActionNode::halt();
}

bool mars::agent::logical::common::behavior::DeallocateEntity::getParams()
{
  BT::Optional<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>
      robotProperties;
  BT::Optional<mars::common::Id> agentId;
  BT::Optional<mars::common::Id> routeId;
  BT::Optional<std::string> physicalAgentNamespace;
  BT::Optional<unsigned int> allocationCount;
  BT::Optional<mars::routing::core::IterationStep*> deallocateStep;

  // gets set to false if any param could not be found
  bool successful = true;

  // robot properties
  robotProperties =
      this->getInput<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
          PARAM_NAME_ROBOT_PROPERTIES);
  if (!robotProperties)
  {
    MARS_LOG_ERROR(robotProperties.error());
    successful = false;
  }
  else
  {
    this->mRobotProperties = robotProperties.value();
  }

  // deallocation step
  deallocateStep =
      this->getInput<mars::routing::core::IterationStep*>(PARAM_NAME_ROUTE_DEALLOCATE_STEP);
  if (!deallocateStep)
  {
    MARS_LOG_ERROR(deallocateStep.error());
    successful = false;
  }
  else
  {
    this->mDeallocateStep = deallocateStep.value();
  }

  // robot pose
  this->updateRobotPose();

  // allocation count
  updateAllocationCount();

  // read params from blackboard, which are only needed once (no update at
  // runtime)

  // agent Id
  if (!this->mAgentId)
  {
    agentId = this->getInput<mars::common::Id>(PARAM_NAME_AGENT_ID);

    if (!agentId)
    {
      MARS_LOG_ERROR(agentId.error());
      successful = false;
    }
    this->mAgentId = agentId.value();
  }

  // route Id
  routeId = this->getInput<mars::common::Id>(PARAM_NAME_ROUTE_ID);
  if (!routeId)
  {
    MARS_LOG_ERROR(routeId.error());
    successful = false;
  }
  this->mRouteId = routeId.value();

  // physical agent namespace
  if (!this->mPhysicalAgentNamespace)
  {
    physicalAgentNamespace = this->getInput<std::string>(PARAM_NAME_PHYSICAL_AGENT_NAMESPACE);
    if (!physicalAgentNamespace)
    {
      MARS_LOG_ERROR(physicalAgentNamespace.error());
      successful = false;
    }
    this->mPhysicalAgentNamespace = physicalAgentNamespace.value();
  }
  return successful;
}

bool mars::agent::logical::common::behavior::DeallocateEntity::setParams()
{
  BT::Result result;
  int entityType;
  mars::common::Id currentEntityId;
  bool successful = true;

  // update the number of hold allocations on the blackboard
  result = this->setOutput<unsigned int>(PARAM_NAME_ALLOCATION_COUNT, this->mAllocationCount);
  if (!result)
  {
    MARS_LOG_ERROR(result.error());
    successful = false;
  }

  // write pointer to next step on the blackboard
  result = this->setOutput<mars::routing::core::IterationStep*>(PARAM_NAME_ROUTE_DEALLOCATE_STEP,
                                                                this->mDeallocateStep->getNext());
  if (!result)
  {
    MARS_LOG_ERROR(result.error());
    successful = false;
  }

  // write current topology on the blackboard
  entityType = this->mDeallocateStep->getTarget().getType();
  currentEntityId = this->mDeallocateStep->getTarget().getId();

  if (entityType >= mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
      entityType <= mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
  {
    result = this->setOutput<std::shared_ptr<mars::routing::common::topology::Entity>>(
        PARAM_NAME_CURRENT_TOPOLOGY_ENTITY,
        std::make_shared<mars::routing::common::topology::Vertex>(currentEntityId));
  }
  else if (entityType >= mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
           entityType <= mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
  {
    result = this->setOutput<std::shared_ptr<mars::routing::common::topology::Entity>>(
        PARAM_NAME_CURRENT_TOPOLOGY_ENTITY,
        std::make_shared<mars::routing::common::topology::Edge>(currentEntityId));
  }
  else
  {
    MARS_LOG_WARN_UNKNOWN_TOPOLOGY_ENTITY_TYPE();
  }

  if (!result)
  {
    MARS_LOG_ERROR(result.error());
    successful = false;
  }

  return successful;
}

Eigen::Vector2d mars::agent::logical::common::behavior::DeallocateEntity::getRobotCoordinate()
{
  Eigen::Vector2d robotCoordinate2D;

  robotCoordinate2D[0] = this->mRobotPose.translation().x();
  robotCoordinate2D[1] = this->mRobotPose.translation().y();

  return robotCoordinate2D;
}

bool mars::agent::logical::common::behavior::DeallocateEntity::updateRobotPose()
{
  BT::Optional<Eigen::Affine3d> robotPose;
  bool successful = true;

  robotPose = this->getInput<Eigen::Affine3d>(PARAM_NAME_ROBOT_POSE);
  if (!robotPose)
  {
    MARS_LOG_ERROR(robotPose.error());
    successful = false;
  }
  else
  {
    this->mRobotPose = robotPose.value();
  }
  return successful;
}

bool mars::agent::logical::common::behavior::DeallocateEntity::updateAllocationCount()
{
  BT::Optional<unsigned int> allocationCount;
  bool successful = true;

  allocationCount = this->getInput<unsigned int>(PARAM_NAME_ALLOCATION_COUNT);
  if (!allocationCount)
  {
    MARS_LOG_ERROR(allocationCount.error());
    successful = false;
  }
  else
  {
    this->mAllocationCount = allocationCount.value();
  }
  return successful;
}
