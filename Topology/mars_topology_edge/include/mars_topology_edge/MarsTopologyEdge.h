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


#ifndef MARSTOPOLOGYEDGE_H
#define MARSTOPOLOGYEDGE_H

#include "MarsEdge.h"

// own ros includes
#include <mars_common/EntityVisualization.h>
#include <mars_common/Id.h>
#include <mars_common/Logger.h>
#include <mars_topology_actions/AllocateEntityAction.h>
#include <mars_topology_common/TopologyEntityLock.h>
#include <mars_topology_srvs/AddReservation.h>
#include <mars_topology_srvs/DeallocateEntity.h>
#include <mars_topology_srvs/DeleteReservation.h>
#include <mars_topology_srvs/GetConnections.h>
#include <mars_topology_srvs/GetCoordinate.h>
#include <mars_topology_srvs/GetFootprint.h>
#include <mars_topology_srvs/GetFreeTimeSlots.h>
#include <mars_topology_srvs/GetLength.h>
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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++ includes
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <limits>
#include <map>
#include <string>

namespace mars
{
namespace topology
{
namespace edge
{
/**
 * This class represents an edge of a graph. In this case, an object of
 * this class represents the edge of a topological graph
 *
 */
class MarsTopologyEdge
{
public:
  /**
   * @brief Edge Creates an object of Edge.
   */
  MarsTopologyEdge();

  /**
   * @brief ~Edge Deletes an object Edge object.
   */
  ~MarsTopologyEdge();

  /**
   * @brief runROSNode Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool runROSNode(void);

  bool actionGoalCallbackAllocateEntity(
      const actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>& pAllocationGoal);

  bool actionCancelCallbackAllocateEntity(
      const actionlib::ServerGoalHandle<mars_topology_actions::AllocateEntityAction>& pAllocationGoal);

  /**
   * @brief serviceCallbackGetCoordinate The callback funktion
   * which is called by clients that want to know the
   * pecise position (x,y) of the Edge.
   *
   * @param req The request, empty in this case.
   * @param res The response, contains the precise position.
   *
   * @return true if the service was executed successsfully.
   */
  bool serviceCallbackGetCoordinate(mars_topology_srvs::GetCoordinate::Request& req,
                                    mars_topology_srvs::GetCoordinate::Response& res);

  /**
 * @brief serviceCallbackGetCoordinate The callback funktion
 * which is called by clients that want to know the
 * pecise length of the Edge.
 *
 * @param req The request, empty in this case.
 * @param res The response, contains the lenght of the edge.
 *
 * @return true if the service was executed successsfully.
 */
  bool serviceCallbackGetLength(mars_topology_srvs::GetLength::Request& req,
                                mars_topology_srvs::GetLength::Response& res);

  /**
   * @brief serviceCallbackGetFootprint The callback funktion which
   * is called by clients that want to know the footprint
   * of the edge. The footprint is described by a polygon.
   *
   * @param req The request, empty in this case.
   * @param res The response, contains the footprint.
   *
   * @return true if the service was executed successfully.
   */
  bool serviceCallbackGetFootprint(mars_topology_srvs::GetFootprint::Request& req,
                                   mars_topology_srvs::GetFootprint::Response& res);

  /**
   * @brief serviceCallbackLockTopologyEntity The callback function is called by the
   * service clients if an initiator wants to lock the edge.
   * @param req The request contains the initiator id, a reason and the time interval to lock the
   * edge.
   * @param res The response contains a result if the lock request is accepted and a lock id.
   * @return true if the service was executed successfully.
   */
  bool serviceCallbackLockTopologyEntity(mars_topology_srvs::LockTopologyEntity::Request& req,
                                         mars_topology_srvs::LockTopologyEntity::Response& res);

  /**
   * @brief serviceCallbackUnlockTopologyEntity The callback function which
   * is called by the service clients that want to unlock
   * the edge.
   *
   * @param req The request, contains the id of the lock and optionally reason.
   * @param res The response, contains the result.
   *
   * @return true if the edge is unlocked.
   */
  bool serviceCallbackUnlockTopologyEntity(mars_topology_srvs::UnlockTopologyEntity::Request& req,
                                           mars_topology_srvs::UnlockTopologyEntity::Response& res);

  bool serviceCallbackAddReservation(mars_topology_srvs::AddReservation::Request& req,
                                     mars_topology_srvs::AddReservation::Response& res);
  bool serviceCallbackDeallocateEntity(mars_topology_srvs::DeallocateEntity::Request& req,
                                       mars_topology_srvs::DeallocateEntity::Response& res);
  bool serviceCallbackDeleteReservation(mars_topology_srvs::DeleteReservation::Request& req,
                                        mars_topology_srvs::DeleteReservation::Response& res);
  bool serviceCallbackGetFreeTimeSlots(mars_topology_srvs::GetFreeTimeSlots::Request& req,
                                       mars_topology_srvs::GetFreeTimeSlots::Response& res);
  bool serviceCallbackGetConnections(mars_topology_srvs::GetConnections::Request& req,
                                       mars_topology_srvs::GetConnections::Response& res);
  bool serviceCallbackGetRestrictions(mars_topology_srvs::GetRestrictions::Request& req,
                                      mars_topology_srvs::GetRestrictions::Response& res);
  bool serviceCallbackGetType(mars_topology_srvs::GetType::Request& req, mars_topology_srvs::GetType::Response& res);
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
  ros::Publisher mVisualizationDirectionPublisher;

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
  ros::ServiceServer mServiceGetLength;
  ros::ServiceServer mServiceGetConnections;
  ros::ServiceServer mServiceGetRestrictions;
  ros::ServiceServer mServiceGetType;
  ros::ServiceServer mServiceGetStatus;

  // ros action server
  actionlib::ActionServer<mars_topology_actions::AllocateEntityAction>* mActionAllocateEntity;

  // ros Timer
  ros::Timer mVisualizationTimer;

  // members
  mars::topology::edge::MarsEdge* mMarsEdge;

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
  std::string mServiceNameGetLength;
  std::string mServiceNameGetConnections;
  std::string mServiceNameGetRestrictions;
  std::string mServiceNameGetType;
  std::string mServiceNameGetStatus;
  std::string mActionNameAllocateEntity;
  std::string mTopicNameVisualizationDescription;
  std::string mTopicNameVisualizationFootprint;
  std::string mTopicNameVisualizationHeatmap;
  std::string mTopicNameVisualizationDirection;

  /**
   * @brief mMarkerInit True if footprint was initially published.
   */
  bool mMarkerInit;
  float mDrawEntityHz;

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
   * @brief processAllocationGoals Evaluates allocation goals and publishes feedback and result
   * messages.
   */
  void processAllocationGoals() const;

  /**
   * @brief advertiseService Adertises services.
   * @return Returns true ich the service was advertised successfully.
   * @throw mars::common::exception::AdvertiseServiceException
   */
  template <class T, class MReq, class MRes>
  bool advertiseService(bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer, std::string serviceName,
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
  T getParam(const ros::NodeHandle& nH, const std::string& paramName, const T& defaultValue) const;

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   * @throw mars::common::exception::ReadParamException
   */
  template <typename T>
  void getParam(const ros::NodeHandle& nH, const std::string& paramName, T& param) const
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
}  // namespace edge
}  // namespace topology
}  // namespace mars

#endif  // MARSTOPOLOGYEDGE_H
