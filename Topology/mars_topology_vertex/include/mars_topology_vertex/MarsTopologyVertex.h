/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : TODO: INSERT PROJECT NAME
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Marc Down, Max Mustermann
 * Contact(s)       : marc.down@iml.fraunhofer.de
 *                    max.mustermann@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#ifndef MARS_TOPOLOGY_COMMON_MARSTOPOLOGYVERTEX_H
#define MARS_TOPOLOGY_COMMON_MARSTOPOLOGYVERTEX_H

#include "MarsVertex.h"

// ros node internal includes
#include <mars_common/EntityVisualization.h>
#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_topology_actions/AllocateEntityAction.h>
#include <mars_topology_common/TopologyEntityLock.h>
#include <mars_topology_srvs/AddReservation.h>
#include <mars_topology_srvs/DeallocateEntity.h>
#include <mars_topology_srvs/DeleteReservation.h>
#include <mars_topology_srvs/GetCoordinate.h>
#include <mars_topology_srvs/GetFootprint.h>
#include <mars_topology_srvs/GetFreeTimeSlots.h>
#include <mars_topology_srvs/GetIngoingEdges.h>
#include <mars_topology_srvs/GetOutgoingEdges.h>
#include <mars_topology_srvs/GetRestrictions.h>
#include <mars_topology_srvs/GetStatus.h>
#include <mars_topology_srvs/GetType.h>
#include <mars_topology_srvs/LockTopologyEntity.h>
#include <mars_topology_srvs/UnlockTopologyEntity.h>

// own exception includes
#include <mars_common/exception/AdvertiseServiceException.h>
#include <mars_common/exception/ReadParamException.h>
#include <mars_common/exception/SetParamException.h>

// ros includes
#include <actionlib/server/action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

// C++ includes
#include <string>
#include <uuid/uuid.h>

namespace mars
{
namespace topology
{
namespace vertex
{
/**
 * This node represents a topological vertex.
 */
class MarsTopologyVertex
{
public:
  /**
   * @brief MarsTopologyVertex Creates an object of MarsTopologyVertex.
   */
  MarsTopologyVertex();

  /**
   * @brief MarsTopologyVertex Deletes an object MarsTopologyVertex object.
   */
  ~MarsTopologyVertex();

  /**
   * @brief runROSNode Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool runROSNode(void);

  bool actionGoalCallbackAllocateEntity(
      const actionlib::ServerGoalHandle<
          mars_topology_actions::AllocateEntityAction>& pAllocationGoal);

  bool actionCancelCallbackAllocateEntity(
      const actionlib::ServerGoalHandle<
          mars_topology_actions::AllocateEntityAction>& pAllocationGoal);

  /**
   * @brief serviceCallbackGetCoordinate The callback funktion
   * which is called by clients that want to know the
   * pecise position (x,y) of the Vertex.
   *
   * @param req The request, empty in this case.
   * @param res The response, contains the precise position.
   *
   * @return true if the service was executed successsfully.
   */
  bool serviceCallbackGetCoordinate(
      mars_topology_srvs::GetCoordinate::Request& req,
      mars_topology_srvs::GetCoordinate::Response& res);

  /**
   * @brief serviceCallbackGetFootprint The callback funktion which
   * is called by clients that want to know the footprint
   * of the vertex. The footprint is described by a polygon.
   *
   * @param req The request, empty in this case.
   * @param res The response, contains the footprint.
   *
   * @return true if the service was executed successfully.
   */
  bool
  serviceCallbackGetFootprint(mars_topology_srvs::GetFootprint::Request& req,
                              mars_topology_srvs::GetFootprint::Response& res);

  /**
   * @brief serviceCallbackLockTopologyEntity The callback function is called by
   * the service clients if an initiator wants to lock the vertex.
   * @param req The request contains the initiator id, a reason and the time
   * interval to lock the vertex.
   * @param res The response contains a result if the lock request is accepted
   * and a lock id.
   * @return true if the service was executed successfully.
   */
  bool serviceCallbackLockTopologyEntity(
      mars_topology_srvs::LockTopologyEntity::Request& req,
      mars_topology_srvs::LockTopologyEntity::Response& res);

  /**
   * @brief serviceCallbackUnlockTopologyEntity The callback function which
   * is called by the service clients that want to unlock
   * the vertex.
   *
   * @param req The request, contains the id of the lock and optionally reason.
   * @param res The response, contains the result.
   *
   * @return true if the vertex is unlocked.
   */
  bool serviceCallbackUnlockTopologyEntity(
      mars_topology_srvs::UnlockTopologyEntity::Request& req,
      mars_topology_srvs::UnlockTopologyEntity::Response& res);

  bool serviceCallbackAddReservation(
      mars_topology_srvs::AddReservation::Request& req,
      mars_topology_srvs::AddReservation::Response& res);
  bool serviceCallbackDeallocateEntity(
      mars_topology_srvs::DeallocateEntity::Request& req,
      mars_topology_srvs::DeallocateEntity::Response& res);
  bool serviceCallbackDeleteReservation(
      mars_topology_srvs::DeleteReservation::Request& req,
      mars_topology_srvs::DeleteReservation::Response& res);
  bool serviceCallbackGetFreeTimeSlots(
      mars_topology_srvs::GetFreeTimeSlots::Request& req,
      mars_topology_srvs::GetFreeTimeSlots::Response& res);
  bool serviceCallbackGetIngoingEdges(
      mars_topology_srvs::GetIngoingEdges::Request& req,
      mars_topology_srvs::GetIngoingEdges::Response& res);
  bool serviceCallbackGetOutgoingEdges(
      mars_topology_srvs::GetOutgoingEdges::Request& req,
      mars_topology_srvs::GetOutgoingEdges::Response& res);
  bool serviceCallbackGetRestrictions(
      mars_topology_srvs::GetRestrictions::Request& req,
      mars_topology_srvs::GetRestrictions::Response& res);
  bool serviceCallbackGetType(mars_topology_srvs::GetType::Request& req,
                              mars_topology_srvs::GetType::Response& res);
  bool serviceCallbackGetStatus(mars_topology_srvs::GetStatus::Request& req,
                                mars_topology_srvs::GetStatus::Response& res);

  /**
   * @brief drawMarker Helpfer function for visualization.
   */
  void drawMarker(const ros::TimerEvent& event) const;

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

  // ros publisher
  ros::Publisher mRegistrationPublisher;
  ros::Publisher mVisualizationDescriptionPublisher;
  ros::Publisher mVisualizationHeatmapPublisher;
  ros::Publisher mVisualizationFootprintPublisher;

  // ros subscriber

  // ros service server
  ros::ServiceServer mServiceGetCoordinate;
  ros::ServiceServer mServiceGetFootprint;
  ros::ServiceServer mServiceLockTopologyEntity;
  ros::ServiceServer mServiceUnlockTopologyEntity;
  ros::ServiceServer mServiceAddReservation;
  ros::ServiceServer mServiceDeallocateEntity;
  ros::ServiceServer mServiceDeleteReservation;
  ros::ServiceServer mServiceGetFreeTimeSlots;
  ros::ServiceServer mServiceGetIngoingEdges;
  ros::ServiceServer mServiceGetOutgoingEdges;
  ros::ServiceServer mServiceGetRestrictions;
  ros::ServiceServer mServiceGetType;
  ros::ServiceServer mServiceGetStatus;

  // ros action server
  actionlib::ActionServer<mars_topology_actions::AllocateEntityAction>*
      mActionAllocateEntity;

  // ros Timer
  ros::Timer mVisualizationTimer;

  // members
  mars::topology::vertex::MarsVertex* mMarsVertex;

  /**
   * @brief mHz Contains the information about the set loop rate of the node.
   */
  int mHz;

  /**
   * @brief mVisualizationHz Contains the information about the set loop rate of the visualization of this node.
   */
  float mVisualizationHz;

  /**
   * @brief mNodeLogLevel Current log level of the node.
   */
  std::string mNodeLogLevel;

  std::string mServiceNameGetCoordinate;
  std::string mServiceNameGetFootprint;
  std::string mServiceNameLockTopologyEntity;
  std::string mServiceNameUnlockTopologyEntity;
  std::string mServiceNameAddReservationRequest;
  std::string mServiceNameDeallocateEntity;
  std::string mServiceNameDeleteReservation;
  std::string mServiceNameGetFreeTimeSlots;
  std::string mServiceNameGetIngoingEdges;
  std::string mServiceNameGetOutgoingEdges;
  std::string mServiceNameGetRestrictions;
  std::string mServiceNameGetType;
  std::string mServiceNameGetStatus;
  std::string mActionNameAllocateEntity;
  std::string mTopicNameVisualizationDescription;
  std::string mTopicNameVisualizationFootprint;
  std::string mTopicNameVisualizationHeatmap;



  // init methods for ros node

  /**
   * @brief init Inits the node. This method automatically calls the methods:
   * bool mMarkerInit; initServices(), initPublisher(), initSubsriber(),
   * readLaunchParams() and printNodeInfos() if no error occours during
   * execution.
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
   * @brief processAllocationGoals Evaluates allocation goals and publishes
   * feedback and result messages.
   */
  void processAllocationGoals() const;

  /**
   * @brief advertiseService Adertises services.
   * @return Returns true ich the service was advertised successfully.
   * @throw mars::common::exception::AdvertiseServiceException
   */
  template <class T, class MReq, class MRes>
  bool advertiseService(
      bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer,
      std::string serviceName,
      T* obj) noexcept(false);
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
   * @brief initTimer Initializes the used timers.
   * @return ture if no errors occoured.
   */
  bool initTimer(void);

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
  T getParam(const ros::NodeHandle& nH, const std::string& paramName,
             const T& defaultValue) const;

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   * @throw mars::common::exception::ReadParamException
   */
  template <typename T>
  void getParam(const ros::NodeHandle& nH, const std::string& paramName,
                T& param) const
      noexcept(false);

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

};
} // namespace vertex
} // namespace topology
} // namespace mars

#endif // MARS_TOPOLOGY_COMMON_MARSTOPOLOGYVERTEX_H
