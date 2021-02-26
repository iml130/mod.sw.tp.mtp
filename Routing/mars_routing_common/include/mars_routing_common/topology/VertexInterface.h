#ifndef MARS_ROUTING_COMMON_TOPOLOGY_VERTEXINTERFACE_H
#define MARS_ROUTING_COMMON_TOPOLOGY_VERTEXINTERFACE_H

// Standart C++ Includes
#include <mutex>
#include <string>
#include <vector>
#include <map>

// Ros Includes
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <ros/service_client.h>

// Eigen Includes
#include <eigen3/Eigen/Core>

#include <boost/geometry.hpp>
#include <boost/optional.hpp>

// Project related Includes
#include "mars_common/Id.h"
#include "mars_common/Result.h"
#include "mars_common/TimeInterval.h"
#include "mars_common/Tuple.h"
#include <mars_common/exception/NotInitializedException.h>

#include "mars_common/geometry/Footprint.h"
#include "mars_routing_common/topology/Status.h"

#include "mars_topology_actions/AllocateEntityAction.h"
#include "mars_topology_msgs/TopologyEntityType.h"
#include "mars_topology_srvs/AddReservation.h"
#include "mars_topology_srvs/DeallocateEntity.h"
#include "mars_topology_srvs/DeleteReservation.h"
#include "mars_topology_srvs/GetCoordinate.h"
#include "mars_topology_srvs/GetFootprint.h"
#include "mars_topology_srvs/GetFreeTimeSlots.h"
#include "mars_topology_srvs/GetIngoingEdges.h"
#include "mars_topology_srvs/GetLength.h"
#include "mars_topology_srvs/GetOutgoingEdges.h"
#include "mars_topology_srvs/GetRestrictions.h"
#include "mars_topology_srvs/GetStatus.h"
#include "mars_topology_srvs/GetType.h"
#include "mars_topology_srvs/LockTopologyEntity.h"
#include "mars_topology_srvs/UnlockTopologyEntity.h"
#include <mars_topology_srvs/GetTopologyEntityContainerId.h>

#include "mars_topology_common/TopologyEntityRestrictions.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class VertexInterface
{
public:
  /**
   * @brief Interface
   * @param pId Unique Id of the vertex. Has to be unique in regards to other
   * topology entities, too.
   */
  VertexInterface(const mars::common::Id& pId);

  /**
   * @brief Default destructor
   * Deletes allocated ActionClient
   */
  ~VertexInterface();

  /**
   * @brief Sets the static namespace of all topology vertices for service and
   * topic access
   * @param pNamespace The global namespace to set
   */
  static void setNamespace(const std::string& pNamespace);

  /**
   * @brief Gets the type of an Topology vertex
   * @return The vertex type, 0 if the type is unknown
   */
  boost::optional<int> getType();

  /**
   * @brief Adds a reservation to the given vertex
   * @param pAgentId Agent ID of the agent which wants to reservate the vertex
   * @param pPathId Path ID of the current agent path
   * @param pTimeInterval Interval of the reservation
   * @return A result type which indicates the success.
   */
  mars::common::Result addReservation(const mars::common::Id& pAgentId,
                                      const mars::common::Id& pPathId,
                                      const mars::common::TimeInterval& pTimeInterval);

  /**
   * @brief Deletes a reservation from a vertex
   * @param pAgentId Agent ID of the agent which causes the deletion
   * @param pPathId Path ID of the agent path
   * @return A result type which indicates the success.
   */
  mars::common::Result deleteReservation(const mars::common::Id& pAgentId,
                                         const mars::common::Id& pPathId);

  /**
   * @brief Allocates the given vertex for use by the provided agent ID
   * @param pAgentId Agent ID of the agent which is allocating the vertex and is
   * allowed to use it
   * @param pPathId Id of the agent path
   * @return A result type which indicates the success.
   */
  bool
  allocateVertex(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
                 const mars::common::TimeInterval& pAllocationInterval,
                 const ros::Duration& pWaitForGoal = ros::Duration(0, 0),
                 const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                     SimpleDoneCallback& pDoneCallback = actionlib::SimpleActionClient<
                         mars_topology_actions::AllocateEntityAction>::SimpleDoneCallback(),
                 const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                     SimpleActiveCallback& pActiveCallback = actionlib::SimpleActionClient<
                         mars_topology_actions::AllocateEntityAction>::SimpleActiveCallback(),
                 const actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>::
                     SimpleFeedbackCallback& pFeedbackCallback = actionlib::SimpleActionClient<
                         mars_topology_actions::AllocateEntityAction>::SimpleFeedbackCallback());

  /**
   * @brief Frees the given Vertex, in order to make it
   * useable by other Entities
   * @param pAgentId Agent ID of the agent which shall "do" the deallocation
   * @return A result type which indicates the success.
   */
  mars::common::Result deallocateVertex(const mars::common::Id& pAgentId);

  /**
   * @brief Gets the coordinate of the given vertex
   * @return Coordinate of the Vertex as point, if service is available
   */
  boost::optional<Eigen::Vector3d> getCoordinate();

  /**
   * @brief Gets the footprint of the given vertex
   * @return The footprint of the vertex, if service is available
   */
  boost::optional<mars::common::geometry::Footprint> getFootprint();

  /**
   * @brief Gets the free timeslots of a given vertex
   * @param pStartTime Start time to check for free time slots
   * @return The free timeslots of the vertex, if service is available
   */
  boost::optional<std::vector<mars::common::TimeInterval>>
  getFreeTimeSlots(const ros::Time pStartTime);

  /**
   * @brief Gets the ingoing edges of a vertex
   * @return Traversable ingoing edges, if service is available
   */
  boost::optional<std::vector<mars::common::Id>> getIngoingEdges();

  /**
   * @brief Gets the outgoing edges of a vertex
   * @return Traversable outgoing edges, if service is available
   */
  boost::optional<std::vector<mars::common::Id>> getOutgoingEdges();

  /**
   * @brief Gets the restrictions of a vertex
   * @return The restrictions of the given vertex, if service is available
   */
  boost::optional<mars::topology::common::TopologyEntityRestrictions> getRestrictions();

  /**
   * @brief Locks a vertex
   * @param pInitiatorId Id of the initiatior who wants to lock the vertex
   * @param pTimeInterval Interval of how long the vertex should be locked
   * @param pReason Optional reason (for logging)
   * @return Id of the lock, if service is available. Needed if you want to
   * delete the lock.
   */
  boost::optional<mars::common::Id> lockVertex(const mars::common::Id& pInitiatorId,
                                               const mars::common::TimeInterval& pTimeInterval,
                                               const std::string& pReason = "");

  /**
   * @brief Unlocks a vertex
   * @param pLockId Id of the lock
   * @param pReason Optional reason (for logging)
   * @return A result type which indicates the success.
   */
  boost::optional<bool> unlockVertex(const mars::common::Id& pLockId,
                                     const std::string pReason = "");

  /**
   * @brief Getter for status attributes like locks and reservations of a
   * topology vertex
   * @return The status of the vertex containing locks and reservations
   */
  boost::optional<mars::routing::common::topology::Status> getStatus();

private:
  static std::string sNamespace; /**< Holds the static part of the namespace for
                                    the interface */
  static std::mutex sMutex;      /**< Synchronization mutex for the namespace */
  mars::common::Id mContainerId;
  mars::common::Id mVertexId; /**< The Id of the referring Vertex */
  actionlib::SimpleActionClient<mars_topology_actions::AllocateEntityAction>*
      mAllocationActionClient;
  mutable std::mutex mMutex; /**< Synchronization mutex for the allocation client */
  std::map<std::string, ros::ServiceClient> mDeallocServiceClientMap;

  ros::ServiceClient getDeallocServiceClient(std::string &serviceName);

  /**
   * @brief constructServiceTopic Constructs the complete topic name for ros.
   * @param pServiceName The name of the service e.g. get_params
   * @return complete topic name with namespace for a service call.
   * @throws NotInitializedException if the container id is no set/valid.
   */
  const std::string constructServiceTopic(const std::string& pServiceName) noexcept(false);

  /**
   * @brief initContainerId Requests the container id from the yellowpage server.
   */
  void initContainerId(void);

  /**
   *  @brief Deletes the SimpleActionClient and stops the spinning thread.
   */
  void deleteAllocationActionClient(void);
};
} // namespace topology
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_TOPOLOGY_VERTEXINTERFACE_H
