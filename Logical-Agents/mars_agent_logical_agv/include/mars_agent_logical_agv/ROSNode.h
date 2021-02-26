/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : MARS
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#ifndef MARS_AGENT_LOGICAL_AGV_NODE_H
#define MARS_AGENT_LOGICAL_AGV_NODE_H

// ros includes

#include <ros/ros.h>
#include <ros/package.h>

// ros node internal includes

#include <mars_agent_physical_common/RobotAgentProperties.h>
#include <mars_common/Id.h>
#include <mars_routing_common/topology/Entity.h>

#include <mars_common/Logger.h>
#include <mars_routing_common/topology/Edge.h>
#include <mars_routing_common/topology/EdgeInterface.h>
#include <mars_routing_common/topology/Vertex.h>
#include <mars_routing_common/topology/VertexInterface.h>

// exception includes
#include <mars_common/exception/AdvertiseServiceException.h>
#include <mars_common/exception/ReadParamException.h>
#include <mars_common/exception/SetParamException.h>

// includes services
#include <mars_agent_logical_srvs/AddMoveOrder.h>
#include <mars_agent_logical_srvs/AddServiceOrder.h>
#include <mars_agent_logical_srvs/AddTransportOrder.h>
#include <mars_agent_logical_srvs/GetOrderBid.h>

// includes msgs
#include <mars_agent_physical_robot_msgs/ActionAssignment.h>
#include <mars_agent_physical_robot_msgs/AssignmentStatus.h>
#include <mars_agent_physical_robot_msgs/CancelTask.h>
#include <mars_agent_physical_robot_msgs/Motion.h>
#include <mars_agent_physical_robot_msgs/MotionAssignment.h>
#include <mars_agent_physical_robot_msgs/RobotAgentProperties.h>
#include <mars_topology_msgs/TopologyEntityType.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// C++ includes
#include <boost/optional.hpp>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

// behaviortree includes
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

// behaviortree actions
#include <mars_agent_logical_common/behavior/AllocateEntity.h>
#include <mars_agent_logical_common/behavior/DeallocateEntity.h>
#include <mars_agent_logical_common/behavior/ExtractMoveOrder.h>
#include <mars_agent_logical_common/behavior/ExtractTransportOrder.h>
#include <mars_agent_logical_common/behavior/ExtractTransportOrderStep.h>
#include <mars_agent_logical_common/behavior/ImportROSParameter.h>
#include <mars_agent_logical_common/behavior/PlanRoute.h>
#include <mars_agent_logical_common/behavior/PublishMotionAssignmentsToTarget.h>
#include <mars_agent_logical_common/behavior/PublishOrderStatus.h>
#include <mars_agent_logical_common/behavior/SetCurrentMotion.h>
#include <mars_agent_logical_common/behavior/SetCurrentOrder.h>
#include <mars_agent_logical_common/behavior/SetCurrentTime.h>
#include <mars_agent_logical_common/behavior/SetId.h>
#include <mars_agent_logical_common/behavior/SetRobotProperties.h>
#include <mars_agent_logical_common/behavior/SetTopologyEntity.h>
#include <mars_agent_logical_common/behavior/WaitForManualTrigger.h>
#include <mars_agent_logical_common/behavior/StringToBool.h>

#include <mars_agent_logical_common/behavior/OrderStatus/SetOrderState.h>
#include <mars_agent_logical_common/behavior/OrderStatus/SetOrderStatus.h>
#include <mars_agent_logical_common/behavior/OrderStatus/SetOrderType.h>
#include <mars_agent_logical_common/behavior/SetDuration.h>
#include <mars_agent_logical_common/behavior/WaitForNSeconds.h>

//behaviortree decorator
#include <mars_agent_logical_common/behavior/Decorators/RepeatOncePerTick.h>
#include <mars_agent_logical_common/behavior/Decorators/RepeatUntilDone.h>
#include <mars_agent_logical_common/behavior/Decorators/RepeatUntilSuccessful.h>

// behaviortree conditions
#include <mars_agent_logical_common/behavior/Conditions/AreIdsEqual.h>
#include <mars_agent_logical_common/behavior/Conditions/AreTopologyEntitiesEqual.h>
#include <mars_agent_logical_common/behavior/Conditions/IsMoveOrder.h>
#include <mars_agent_logical_common/behavior/Conditions/IsTransportOrder.h>
#include <mars_agent_logical_common/behavior/Conditions/IsManualAction.h>
#include <mars_agent_logical_common/behavior/Conditions/IsTrue.h>



//behaviortree logger
#include <mars_common/behavior/CoutDurationLogger.h>

namespace mars
{
namespace agent
{
namespace logical
{
namespace robot
{
/**
 * @brief The Mars Agent Logical Robot ROS Node class
 */
class ROSNode
{
public:
  /**
   * @brief ROSNode Creates an object of MarsAgentRobot.
   */
  ROSNode();

  /**
   * @brief ROSNode Deletes an object MarsAgentRobot object.
   */
  ~ROSNode();

  /**
   * @brief run Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool run(void);

protected:
  // First member variables, second methods.
private:
  // ros node handles
  /**
   * @brief mNH Public ros handle.
   */
  ros::NodeHandle mNH;
  /**
   * @brief mNHPriv Private node handle.
   */
  ros::NodeHandle mNHPriv;

  // Publisher
  ros::Publisher mPublisherCancelTask;
  ros::Publisher mPublisherInitPose;

  // Physical Agent topic names
  std::string mTopicNameCancelTask;
  std::string mTopicNameInitPose;

  // Service names
  std::string mServiceNameManualTrigger;
  ros::ServiceServer mTriggerService;

  /**
   * @brief mHz Contains the information about the set loop rate of the node.
   */
  int mHz;

  /**
   * @brief mNodeLogLevel Current log level of the node.
   */
  std::string mNodeLogLevel;

  mars::common::Id mLogicalAgentId;
  mars::common::Id mPhysicalAgentId;

  mars::routing::common::topology::Entity* mCurrentTopologyEntity;
  mars::common::Id mCurrentPathId;

  // init methods for ros node

  /**
   * @brief init Inits the node. This method automatically calls the methods:
   * initServices(), initPublisher(), initSubsriber(), readLaunchParams() and
   * printNodeInfos() if no error occours during execution.
   *
   * @return Returns true if no error occours.
   */
  bool init(void);

  /**
   * @brief initServices Initializes the provided services.
   * @return Returns true if no error occours.
   */
  bool initServices(void);

  /**
   * @brief initPublisher Initializes the provided publishers.
   * @return Returns true if no error occours.
   */
  bool initPublisher(void);

  /**
   * @brief advertiseService Adertises services.
   * @return Returns true ich the service was advertised successfully.
   */
  template <class T, class MReq, class MRes>
  bool advertiseService(bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer,
                        std::string serviceName, T* obj) noexcept(false);

  /**
   * @brief initSubsriber Initializes the subscribers.
   * @return Returns true if no error occours.
   */
  bool initSubscriber(void);

  /**
   * @brief initActions Initializes the provided actions.
   * @return Returns true if no error occours.
   */
  bool initActions(void);

  /**
   * @brief readLaunchParams Reads the paramters from the launch file.
   * @return Returns true if no error occours.
   */
  bool readLaunchParams(void);

  /**
   * @brief getParam Reads the named paramter form the parameter server.
   * If the parameter could not be found a default value is assigned.
   */
  template <typename T>
  T getParam(ros::NodeHandle& nH, const std::string& paramName, const T& defaultValue);

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   */
  template <typename T>
  void getParam(ros::NodeHandle& nH, const std::string& paramName, T& param) noexcept(false);

  /**
   * @brief rosMainLoop Calls rosSpinOnce() with the rate you set with
   * 'node_rate'.
   */
  void rosMainLoop(void) const;

  /**
   * @brief setNodeLogLevel Set the log level of the ros console for the node.
   * @return Return true if log level was successfully set.
   */
  bool setNodeLogLevel(void) const;

  BT::BehaviorTreeFactory registerBehavior(void) const;
};

} // namespace robot
} // namespace logical
} // namespace agent
} // namespace mars

#endif // MARS_AGENT_LOGICAL_AGV_NODE_H
