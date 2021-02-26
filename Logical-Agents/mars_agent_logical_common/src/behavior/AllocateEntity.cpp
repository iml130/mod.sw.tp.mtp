#include "mars_agent_logical_common/behavior/AllocateEntity.h"

// TODO set constants
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_STEP = "route_allocate_step";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_PATHID = "route_id";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_AGENTID = "agent_id";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT = "allocation_count";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONLIMIT = "allocation_limit";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ROBOTPROPERTIES = "robot_properties";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_NUM_NEW_ALLOCATED = "num_new_allocated";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATION_DONE = "allocation_done";

static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_STEP = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_PATHID = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_AGENTID = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATIONCOUNT = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATIONLIMIT = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ROBOTPROPERTIES = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_NAME_NUM_NEW_ALLOCATED = "";
static const std::string BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATION_DONE = "";

static const int ALLOCATE_ENTITY_ACTION_TIMEOUT = 5;

mars::agent::logical::common::behavior::AllocateEntity::AllocateEntity(
    const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::CoroActionNode(pName, pConfig)
{
}

BT::PortsList mars::agent::logical::common::behavior::AllocateEntity::providedPorts()
{
  return {
      // Path id
      BT::InputPort<mars::common::Id>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_PATHID,
                                      BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_PATHID),
      // Physical agent id
      BT::InputPort<mars::common::Id>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_AGENTID,
                                      BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_AGENTID),
      // Route step
      BT::InputPort<mars::routing::core::IterationStep*>(
          BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_STEP,
          BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_STEP),
      // Allocation limit
      BT::InputPort<unsigned int>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONLIMIT,
                                  BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATIONLIMIT),
      // Robot properties
      BT::InputPort<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
          BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ROBOTPROPERTIES,
          BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ROBOTPROPERTIES),
      // number of newly allocated entities
      BT::OutputPort<unsigned int>(
          BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_NUM_NEW_ALLOCATED,
          BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_NAME_NUM_NEW_ALLOCATED),
      // number of newly allocated entities
      BT::OutputPort<bool>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATION_DONE,
                           BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATION_DONE),
      // Current number of allocated topology entities
      BT::BidirectionalPort<unsigned int>(
          BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT,
          BEHAVIOR_ALLOCATEENTITY_PARAMETER_DESCRIPTION_ALLOCATIONCOUNT)};
}

BT::NodeStatus mars::agent::logical::common::behavior::AllocateEntity::tick()
{
  // Get parameter from blackboard
  if (!getParams())
  {
    // at least one parameter could not be found
    return BT::NodeStatus::FAILURE;
  }

  if (this->mRouteStep != nullptr)
  {
    // check if the physical vehicle fits in the footprint of the topology
    // entity of current step target.
    try
    {
      if (fitsOnCurrentTarget(this->mRouteStep->getTarget(), *this->mRobotProperties.value()))

      {
        waitForNFreeAllocations(1);
        // allocate target
        allocate(this->mRouteStep);
        incAndWriteAllocationCount(1);
        writeNumNewAllocatedCount(1);
      }
      else
      {
        waitForNFreeAllocations(2);
        // allocate target of next step
        allocate(this->mRouteStep->getNext());

        // allocate target of current step
        allocate(this->mRouteStep);
        incAndWriteAllocationCount(2);
        writeNumNewAllocatedCount(2);
      }
    }
    catch (mars::common::exception::ReadParamException e)
    {
      MARS_LOG_ERROR(e.what());
      return BT::NodeStatus::FAILURE;
    }
    catch (mars::common::exception::SetParamException e)
    {
      MARS_LOG_ERROR(e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    // since there are no more steps, the allocation of this route is done
    setDoneOnBlackboard();
  }

  return BT::NodeStatus::SUCCESS;
}

void mars::agent::logical::common::behavior::AllocateEntity::halt()
{
  std::cout << name() << ": Halted." << std::endl;

  // Do not forget to call this at the end.
  CoroActionNode::halt();
}

bool mars::agent::logical::common::behavior::AllocateEntity::getParams()
{
  BT::Optional<mars::common::Id> getPathIdResult;
  BT::Optional<mars::common::Id> getAgentIdResult;
  BT::Optional<unsigned int> getAllocationLimitResult;
  BT::Optional<mars::routing::core::IterationStep*> getRouteStepResult;
  BT::Optional<unsigned int> getAllocationCountResult;
  BT::Optional<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>
      getRobotPropertiesResult;

  // gets set to false if any param could not be found
  bool successful = true;

  // get path id
  getPathIdResult = getInput<mars::common::Id>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_PATHID);
  if (!getPathIdResult)
  {
    MARS_LOG_ERROR(getPathIdResult.error());
    successful = false;
  }
  this->mPathId = getPathIdResult.value();

  // read params from blackboard, which need to be updated every tick

  // get route step
  getRouteStepResult =
      getInput<mars::routing::core::IterationStep*>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_STEP);
  if (!getRouteStepResult)
  {
    MARS_LOG_ERROR(getRouteStepResult.error());
    successful = false;
  }
  this->mRouteStep = getRouteStepResult.value();

  // get allocation count
  getAllocationCountResult =
      getInput<unsigned int>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT);
  if (!getAllocationCountResult)
  {
    MARS_LOG_ERROR(getAllocationCountResult.error());
    successful = false;
  }
  this->mAllocationCount = getAllocationCountResult.value();

  // read params from blackboard, which are only needed once (no update at
  // runtime)

  // get agent id
  if (!this->mAgentId)
  {
    getAgentIdResult = getInput<mars::common::Id>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_AGENTID);
    if (!getAgentIdResult)
    {
      MARS_LOG_ERROR(getAgentIdResult.error());
      successful = false;
    }
    this->mAgentId = getAgentIdResult.value();
  }

  // get allocation limit
  if (!this->mAllocationLimit)
  {
    getAllocationLimitResult =
        getInput<unsigned int>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONLIMIT);
    if (!getAllocationLimitResult)
    {
      MARS_LOG_ERROR(getAllocationLimitResult.error());
      successful = false;
    }
    this->mAllocationLimit = getAllocationLimitResult.value();
  }

  // get robot properties
  if (!this->mRobotProperties)
  {
    getRobotPropertiesResult =
        getInput<std::shared_ptr<mars::agent::physical::common::RobotAgentProperties>>(
            BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ROBOTPROPERTIES);
    if (!getRobotPropertiesResult)
    {
      MARS_LOG_ERROR(getRobotPropertiesResult.error());
      successful = false;
    }
    this->mRobotProperties = getRobotPropertiesResult.value();
  }

  return successful;
}

unsigned int mars::agent::logical::common::behavior::AllocateEntity::readAllocationCount()
{
  BT::Optional<unsigned int> getAllocationCountResult;

  // get allocation count
  getAllocationCountResult =
      getInput<unsigned int>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT);
  if (!getAllocationCountResult)
  {
    MARS_LOG_ERROR(getAllocationCountResult.error());
    throw mars::common::exception::ReadParamException(
        "Could not read the number of current allocated entities on port: " +
        BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT + "\n");
  }
  return getAllocationCountResult.value();
}

void mars::agent::logical::common::behavior::AllocateEntity::waitForNFreeAllocations(
    const unsigned int pWantedAllocations)
{
  // wait until allocation limit is not reached anymore
  while (this->mAllocationCount >= this->mAllocationLimit.value() - pWantedAllocations)
  {
    // yield execution to avoid busy loop
    this->setStatusRunningAndYield();
    this->mAllocationCount = readAllocationCount();
  }
}

bool mars::agent::logical::common::behavior::AllocateEntity::fitsOnCurrentTarget(
    mars::routing::common::topology::Entity& pEntity,
    mars::agent::physical::common::RobotAgentProperties pRAP)
{

  return pEntity.fitsOn(pRAP);
}

void mars::agent::logical::common::behavior::AllocateEntity::incAndWriteAllocationCount(
    const unsigned int pNumOfNewAllocations)
{
  BT::Result setOutputResult;

  // read to get newest
  this->mAllocationCount = readAllocationCount();

  this->mAllocationCount += pNumOfNewAllocations;

  // update allocationcount on blackboard
  setOutputResult =
      setOutput<unsigned int>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT, mAllocationCount);

  // log error from setting output
  if (!setOutputResult)
  {
    throw mars::common::exception::SetParamException(
        "Could not set parameter: " + BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATIONCOUNT +
        " on blackboard \n");
  }
}

void mars::agent::logical::common::behavior::AllocateEntity::writeNumNewAllocatedCount(
    const unsigned int pNumOfNewAllocations)
{
  // update allocationcount on blackboard
  BT::Result setOutputResult = setOutput<unsigned int>(
      BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_NUM_NEW_ALLOCATED, pNumOfNewAllocations);

  // log error from setting output
  if (!setOutputResult)
  {
    throw mars::common::exception::SetParamException(
        "Could not set parameter: " + BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_NUM_NEW_ALLOCATED +
        " on blackboard \n");
  }
}

void mars::agent::logical::common::behavior::AllocateEntity::setDoneOnBlackboard()
{
  // update allocationcount on blackboard
  BT::Result setOutputResult =
      setOutput<bool>(BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATION_DONE, true);

  // log error from setting output
  if (!setOutputResult)
  {
    throw mars::common::exception::SetParamException(
        "Could not set parameter: " + BEHAVIOR_ALLOCATEENTITY_PARAM_NAME_ALLOCATION_DONE +
        " on blackboard \n");
  }
}

bool mars::agent::logical::common::behavior::AllocateEntity::allocate(
    mars::routing::core::IterationStep* pRouteStep)
{
  bool successful = false;
  bool isFinished = false;

  this->setStatusRunningAndYield();

  // execute allocate in another thread std:ref()
  std::thread t([&isFinished, &successful, &pRouteStep, this]() {
    do
    {
      successful = pRouteStep->getTarget().allocate(this->mAgentId.value(), this->mPathId,
                                                    pRouteStep->getTargetOccupationInterval(),
                                                    ros::Duration(ALLOCATE_ENTITY_ACTION_TIMEOUT, 0));
    } while (!successful);

    isFinished = true;
  });

  // wait till allocate is done
  while (!isFinished)
  {
    // yield execution to avoid busy loop
    this->setStatusRunningAndYield();
    MARS_LOG_DEBUG("Wait for allocation of topology entity");
  }
  t.join();

  return successful;
}
