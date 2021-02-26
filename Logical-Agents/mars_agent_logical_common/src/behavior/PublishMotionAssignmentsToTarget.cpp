#include "mars_agent_logical_common/behavior/PublishMotionAssignmentsToTarget.h"

// TODO set constants
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_PATHID = "route_id";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP = "route_allocate_step";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP_COUNT = "route_step_count";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_NUM_NEW_ALLOCATED = "num_new_allocated";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_MOTION_ASSIGNMENT_TOPIC =
    "topic_name_motion_assignment";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAM_NAME_CURRENT_SEQUENCE_POSITION =
    "motion_assignment_index";

static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_PATHID = "";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_STEP = "";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_NAME_NUM_NEW_ALLOCATED = "";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_STEP_COUNT = "";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_MOTION_ASSIGNMENT_TOPIC = "";
static const std::string BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_CURRENT_SEQUENCE_POSITION =
    "";

mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::
    PublishMotionAssignmentsToTarget(const std::string& pName, const BT::NodeConfiguration& pConfig)
    : BT::SyncActionNode(pName, pConfig)
{
}

BT::PortsList
mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::providedPorts()
{
  return {// current sequence position
          BT::BidirectionalPort<unsigned int>(
              BEHAVIOR_PUBLISHMOTION_PARAM_NAME_CURRENT_SEQUENCE_POSITION,
              BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_CURRENT_SEQUENCE_POSITION),
          // motion assignment topic
          BT::InputPort<std::string>(
              BEHAVIOR_PUBLISHMOTION_PARAM_NAME_MOTION_ASSIGNMENT_TOPIC,
              BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_MOTION_ASSIGNMENT_TOPIC),
          // current path id
          BT::InputPort<mars::common::Id>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_PATHID,
                                          BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_PATHID),
          // Number of steps in current route
          BT::InputPort<int>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP_COUNT,
                             BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_STEP_COUNT),
          // Number of newly allocated entities
          BT::InputPort<unsigned int>(
              BEHAVIOR_PUBLISHMOTION_PARAM_NAME_NUM_NEW_ALLOCATED,
              BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_NAME_NUM_NEW_ALLOCATED),
          // current route step
          BT::BidirectionalPort<mars::routing::core::IterationStep*>(
              BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP,
              BEHAVIOR_PUBLISHMOTION_PARAMETER_DESCRIPTION_STEP)};
}

BT::NodeStatus mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::tick()
{
  // Get parameter from blackboard
  if (!getParams())
  {
    // at least one parameter could not be found
    return BT::NodeStatus::FAILURE;
  }

  if (!this->mMotionPublisher)
  {
    // only needed for setting up the publisher
    ros::NodeHandle nh;

    // Set publisher topic
    this->mMotionPublisher = nh.advertise<mars_agent_physical_robot_msgs::MotionAssignment>(
        this->mMotionAssignmentTopic, 10, true);
    // This sleep is nessesary due prevent dropping of the first motion
    // assignment.
    sleep(1);
  }
  if (this->mRouteStep != nullptr)
  {
    try
    {
      switch (readNumNewAllocatedSteps())
      {
      case 1:
        fillAndPublishToIntersection();
        fillAndPublishToTarget();
        writeCurrentSequencePosition();
        incAndWriteRouteStep(1);
        break;

      case 2:
        fillAndPublishToIntersection();
        fillAndPublishToTarget();
        writeCurrentSequencePosition();
        incAndWriteRouteStep(1);

        fillAndPublishToIntersection();
        fillAndPublishToTarget();
        writeCurrentSequencePosition();
        incAndWriteRouteStep(1);
        break;
      }
    }
    catch (mars::common::exception::SetParamException e)
    {
    }
  }
  return BT::NodeStatus::SUCCESS;
}

bool mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::getParams()
{
  BT::Optional<mars::common::Id> getTaskIdResult;
  BT::Optional<std::string> getMotionTopicResult;
  BT::Optional<mars::routing::core::IterationStep*> getRouteStepResult;
  BT::Optional<unsigned int> getSequencePositionResult;
  BT::Optional<int> getStepCountResult;

  // gets set to false if any param could not be found
  bool successful = true;

  // get path id
  getTaskIdResult = getInput<mars::common::Id>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_PATHID);
  if (!getTaskIdResult)
  {
    MARS_LOG_ERROR(getTaskIdResult.error());
    successful = false;
  }
  this->mTaskId = getTaskIdResult.value();

  // get route step
  getRouteStepResult =
      getInput<mars::routing::core::IterationStep*>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP);
  if (!getRouteStepResult)
  {
    MARS_LOG_ERROR(getRouteStepResult.error());
    successful = false;
  }
  this->mRouteStep = getRouteStepResult.value();

  // get motion assignment topic
  getMotionTopicResult =
      getInput<std::string>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_MOTION_ASSIGNMENT_TOPIC);
  if (!getMotionTopicResult)
  {
    MARS_LOG_ERROR(getMotionTopicResult.error());
    successful = false;
  }
  this->mMotionAssignmentTopic = getMotionTopicResult.value();

  // get the number of steps in current route
  getStepCountResult = getInput<int>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP_COUNT);
  if (!getStepCountResult)
  {
    MARS_LOG_ERROR(getStepCountResult.error());
    successful = false;
  }
  this->mStepCount = getStepCountResult.value();

  // get current sequence number
  getSequencePositionResult =
      getInput<unsigned int>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_CURRENT_SEQUENCE_POSITION);
  if (!getSequencePositionResult)
  {
    MARS_LOG_ERROR(getSequencePositionResult.error());
    successful = false;
  }
  this->mCurrentSequencePosition = getSequencePositionResult.value();

  return successful;
}

void mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::
    fillAndPublishToIntersection()
{
  // generates unique id
  mars::common::Id motionId;
  mars_agent_physical_robot_msgs::MotionAssignment toIntersection;

  const boost::optional<mars::common::geometry::Footprint&> originFootprint =
      this->mRouteStep->getOrigin().getFootprint();
  const boost::optional<mars::topology::common::TopologyEntityRestrictions&> originRestrictions =
      this->mRouteStep->getOrigin().getRestrictions();
  const boost::optional<Eigen::Vector2d> intersectionLocation =
      this->mRouteStep->getFootprintIntersection();


//  bool targetAvailable = false;

//  boost::optional<mars::common::geometry::Footprint&> originFootprint;
//  boost::optional<mars::topology::common::TopologyEntityRestrictions&> originRestrictions;
//  boost::optional<Eigen::Vector3d> intersectionLocation;

//  do
//  {
//    originFootprint = this->mRouteStep->getTarget().getFootprint();
//    originRestrictions = this->mRouteStep->getTarget().getRestrictions();
//    intersectionLocation = this->mRouteStep->getTarget().getLocation();

//    // check if nessesary details are available
//    if (originFootprint && originRestrictions && intersectionLocation)
//    {
//      targetAvailable = true;
//    }

//  } while (targetAvailable == false);


  // calc sequence length, double the step count in the current route due to
  // sending two motion assignments per step
  const unsigned int sequenceLength = this->mStepCount * 2;

  // check if nessesary details are available
  if (!originFootprint || !originRestrictions || !intersectionLocation)
  {
    throw mars::common::exception::ReadParamException("Failed to get the target footprint, restrictions or location");
  }

  // generate a ID for the motion
  motionId.initialize();

  // fill motion msg for the assignment to the intersection of both topology
  // entity footprints
  toIntersection.header.stamp = ros::Time::now();
  for (const Eigen::Vector2d& iVector : *originFootprint)
  {
    geometry_msgs::Point32 point;
    point.x = iVector[0];
    point.y = iVector[1];
    point.z = 0;
    toIntersection.motion_area.polygon.points.push_back(point);
  }
  toIntersection.is_waypoint = true;
  toIntersection.max_acceleration.linear.x = originRestrictions->getMaxLinearAcceleration();
  toIntersection.max_acceleration.linear.y = 0;
  toIntersection.max_acceleration.linear.z = 0;

  toIntersection.max_acceleration.angular.x = 0;
  toIntersection.max_acceleration.angular.y = 0;
  toIntersection.max_acceleration.angular.z = originRestrictions->getMaxAngularAcceleration();

  toIntersection.max_velocity.linear.x = originRestrictions->getMaxLinearVelocity();
  toIntersection.max_velocity.linear.y = 0;
  toIntersection.max_velocity.linear.z = 0;

  toIntersection.max_velocity.angular.x = 0;
  toIntersection.max_velocity.angular.y = 0;
  toIntersection.max_velocity.angular.z = originRestrictions->getMaxAngularVelocity();

  toIntersection.motion_id = motionId.toMsg();
  toIntersection.task_id = this->mTaskId.toMsg();
  toIntersection.point.x = (*intersectionLocation)[0];
  toIntersection.point.y = (*intersectionLocation)[1];
  toIntersection.point.theta = 0;
  toIntersection.point_id = this->mRouteStep->getOrigin().getId().toMsg();
  toIntersection.sequence.sequence_number = this->mCurrentSequencePosition;
  toIntersection.sequence.length = sequenceLength;
  toIntersection.use_orientation = false;

  // increment sequence position
  this->mCurrentSequencePosition++;
  // publish msg
  this->mMotionPublisher.value().publish(toIntersection);
}

void mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::
    fillAndPublishToTarget()
{
  mars_agent_physical_robot_msgs::MotionAssignment toTarget;

  // generates unique id
  mars::common::Id motionId;
  motionId.initialize();

//  bool targetAvailable = false;

//  boost::optional<mars::common::geometry::Footprint&> targetFootprint;
//  boost::optional<mars::topology::common::TopologyEntityRestrictions&> targetRestrictions;
//  boost::optional<Eigen::Vector3d> targetLocation;

//  do
//  {
//    targetFootprint = this->mRouteStep->getTarget().getFootprint();
//    targetRestrictions = this->mRouteStep->getTarget().getRestrictions();
//    targetLocation = this->mRouteStep->getTarget().getLocation();

//    // check if nessesary details are available
//    if (targetFootprint && targetRestrictions && targetLocation)
//    {
//      targetAvailable = true;
//    }

//  } while (targetAvailable == false);

   const boost::optional<mars::common::geometry::Footprint&> targetFootprint =
      this->mRouteStep->getTarget().getFootprint();
   const boost::optional<mars::topology::common::TopologyEntityRestrictions&>
      targetRestrictions = this->mRouteStep->getTarget().getRestrictions();
   const boost::optional<Eigen::Vector3d> targetLocation =
      this->mRouteStep->getTarget().getLocation();

  // calc sequence length, double the step count in the current route due to
  // sending two motion assignments per step
  const unsigned int sequenceLength = this->mStepCount * 2;

  // check if nessesary details are available
  if (!targetFootprint || !targetRestrictions || !targetLocation)
  {
    MARS_LOG_ERROR("Failed to get the target footprint, restrictions or location");
    throw mars::common::exception::ReadParamException("Failed to get the target footprint, restrictions or location");
  }

  // fill mostion assignment to Target
  toTarget.header.stamp = ros::Time::now();
  for (const Eigen::Vector2d& iVector : *targetFootprint)
  {
    geometry_msgs::Point32 point;
    point.x = iVector[0];
    point.y = iVector[1];
    point.z = 0;
    toTarget.motion_area.polygon.points.push_back(point);
  }
  toTarget.is_waypoint = true;
  toTarget.max_acceleration.linear.x = targetRestrictions->getMaxLinearAcceleration();
  toTarget.max_acceleration.linear.y = 0;
  toTarget.max_acceleration.linear.z = 0;

  toTarget.max_acceleration.angular.x = 0;
  toTarget.max_acceleration.angular.y = 0;
  toTarget.max_acceleration.angular.z = targetRestrictions->getMaxAngularAcceleration();

  toTarget.max_velocity.linear.x = targetRestrictions->getMaxLinearVelocity();
  toTarget.max_velocity.linear.y = 0;
  toTarget.max_velocity.linear.z = 0;

  toTarget.max_velocity.angular.x = 0;
  toTarget.max_velocity.angular.y = 0;
  toTarget.max_velocity.angular.z = targetRestrictions->getMaxAngularVelocity();

  toTarget.motion_id = motionId.toMsg();
  toTarget.task_id = this->mTaskId.toMsg();
  toTarget.point.x = (*targetLocation)[0];
  toTarget.point.y = (*targetLocation)[1];
  toTarget.point.theta = 0;
  toTarget.point_id = this->mRouteStep->getTarget().getId().toMsg();
  toTarget.sequence.sequence_number = this->mCurrentSequencePosition;
  toTarget.sequence.length = sequenceLength;
  toTarget.use_orientation = false;

  // increment sequence position
  this->mCurrentSequencePosition++;
  // publish msg
  this->mMotionPublisher.value().publish(toTarget);
}

void mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::
    writeCurrentSequencePosition()
{
  // write current sequence position back to the blackboard
  const BT::Result setOutputResult = setOutput<unsigned int>(
      BEHAVIOR_PUBLISHMOTION_PARAM_NAME_CURRENT_SEQUENCE_POSITION, this->mCurrentSequencePosition);

  if (!setOutputResult)
  {
    throw mars::common::exception::SetParamException("Could not set result on blackboard: " + BEHAVIOR_PUBLISHMOTION_PARAM_NAME_CURRENT_SEQUENCE_POSITION);
  }
}

void mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::incAndWriteRouteStep(
    const unsigned int pNumNewAllocatedSteps)
{
  for (unsigned int i = 1; i <= pNumNewAllocatedSteps; ++i)
  {
    // get next step
    this->mRouteStep = this->mRouteStep->getNext();
  }

  // write step back to the blackboard
  const BT::Result setOutputResult = setOutput<mars::routing::core::IterationStep*>(
      BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP, this->mRouteStep);

  if (!setOutputResult)
  {
    throw mars::common::exception::SetParamException("Could not set result on blackboard: " + BEHAVIOR_PUBLISHMOTION_PARAM_NAME_STEP);
  }
}

const unsigned int
mars::agent::logical::common::behavior::PublishMotionAssignmentsToTarget::readNumNewAllocatedSteps()
{
  BT::Optional<unsigned int> getNumNewAllocatedSteps =
      getInput<unsigned int>(BEHAVIOR_PUBLISHMOTION_PARAM_NAME_NUM_NEW_ALLOCATED);
  if (!getNumNewAllocatedSteps)
  {
    throw mars::common::exception::ReadParamException("Could not read from blackboard: " + BEHAVIOR_PUBLISHMOTION_PARAM_NAME_NUM_NEW_ALLOCATED);
  }
  return getNumNewAllocatedSteps.value();
}
