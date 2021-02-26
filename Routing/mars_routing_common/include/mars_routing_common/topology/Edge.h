#ifndef MARS_ROUTING_COMMON_TOPOLOGY_EDGE_H
#define MARS_ROUTING_COMMON_TOPOLOGY_EDGE_H

#include "mars_common/Id.h"
#include "mars_common/TimeInterval.h"
#include <mars_common/Logger.h>

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_common/geometry/Footprint.h"
#include "mars_routing_common/Cache.h"
#include "mars_routing_common/topology/EdgeInterface.h"
#include "mars_routing_common/topology/Entity.h"
#include "mars_routing_common/topology/Vertex.h"

#include "mars_topology_msgs/TopologyEntity.h"

#include "mars_routing_common/utility/AffineProfile.h"
#include "mars_routing_common/utility/TimeInfo.h"
#include "mars_topology_common/TopologyEntityRestrictions.h"

#include <memory>

namespace mars
{
namespace routing
{
namespace common
{
namespace topology
{
class Vertex;

/**
 * @class Edge
 */
class Edge : public mars::routing::common::topology::Entity
{
public:
  /**
   * @brief Default Edge Constructor with cache.
   * @param pId Unique Id of the edge. Has to be unique in regards to other
   * topology entities, too.
   * @param pCache True, if the edge attributes should be statically cached
   * and shared.
   */
  Edge(const mars::common::Id& pId, const bool pCache = true);

  /**
   * @brief Complex getter for edge type.
   */
  int getType();

  /**
   * @return Current location of the edge in reference to the map frame.
   */
  const boost::optional<Eigen::Vector3d> getLocation();

  /**
   * @brief Set the location of the edge directly.
   * Only works, if it hasn't been set or requested already prior to this call.
   * @param pLocation Location of the edge in reference to the map frame.
   */
  void setLocation(const Eigen::Vector3d& pLocation);

  /**
   * @brief Complex getter for the footprint of this edge.
   * Requests the footprint from a connected topology node, if needed.
   * @return Footprint of the edge in absolute coordinates (world frame).
   * @return boost::none, if service and value are unavailable.
   */
  const boost::optional<mars::common::geometry::Footprint&> getFootprint();

  /**
   * @brief Complex getter for the restrictions of this edge.
   * Requests the restrictions from a connected topology node, if needed.
   * @return Restrictions for traversing this edge.
   * @return boost::none, if service and value are unavailable.
   */
  const boost::optional<mars::topology::common::TopologyEntityRestrictions&> getRestrictions();

  /**
   * @brief Complex getter for the lock state of this edge.
   * Evaluates responded list of locked timeframes from a service.
   * @return True, if this edge is locked at the current time.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<bool> isLocked();

  /**
   * @brief Checks, if this edge can be traversed with certain robot properties.
   * @param pRAP Properties of the robot.
   * @return True, if this edge can be traveresed by a robot with the given properties.
   */
  bool isTraversable(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Complex getter for the length of this edge.
   * Requests the length from a connected topology node, if needed.
   * @return Inner length of an edge between the connected vertex footprints.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<double> getLength();

  /**
   * @brief Complex getter for the origin vertex of this edge.
   * Requests the origin from a connected topology node, if needed.
   * @param pTarget Connected target vertex to the requested origin.
   * @return Cached reference to origin vertex.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<mars::routing::common::topology::Vertex&>
  getOrigin(mars::routing::common::topology::Vertex& pTarget);

  /**
   * @brief Complex getter for the target vertex of this edge.
   * Requests the target from a connected topology node, if needed.
   * @param pOrigin Connected origin vertex to the requested target.
   * @return Cached reference to target vertex.
   * @return boost::none, if service and value are unavailable.
   */
  boost::optional<mars::routing::common::topology::Vertex&>
  getTarget(mars::routing::common::topology::Vertex& pOrigin);

  /**
   * @brief Complex getter for all reachable vertices from inside the edge.
   * @param pRAP Properties of the robot.
   * @return Connected vertices in the topology to this edge, which are traversable by a robot with
   * the given properties.
   */
  std::vector<mars::routing::common::topology::Vertex>
  getTraversableTargets(const mars::agent::physical::common::RobotAgentProperties& pRAP);

  /**
   * @brief Complex getter for the free time intervals of this edge.
   * Requests the time intervals from a connected topology node, if needed.
   * @return Free time intervals of the edge in the topology.
   * @return boost::none, if service is unavailable.
   */
  boost::optional<std::vector<mars::common::TimeInterval>>
  getFreeTimeIntervals(const ros::Time& pStartTime);

  /**
   * @brief Complex getter for the shortest occupation duration while traversing the edge from one
   * vertex to the target. Requests topology entity locations and footprints from a connected
   * topology node, if needed. Calculates the occupation duration and stores it in a map for lookup.
   * @return Shortest possible occupation duration on the edge for certain robot parameters.
   * @return boost::none, if service and lookup value are unavailable.
   */
  boost::optional<ros::Duration> getShortestOccupationDuration(
      mars::routing::common::topology::Vertex& pEntryVertex,
      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
      mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
          pTimeInfo);

  /**
   * @brief Complex getter for the real occupation duration while traversing the vertex from one
   * edge to another. Requests topology entity locations and footprints from a connected topology
   * node, if needed. Calculates the occupation duration and stores it in a map for lookup.
   * @return Occupation duration on the edge for certain robot parameters when traversing it.
   * @return boost::none, if service and lookup value are unavailable.
   */
  boost::optional<ros::Duration> getOccupationDuration(
      mars::routing::common::topology::Vertex& pEntryVertex,
      mars::routing::common::topology::Vertex& pExitVertex,
      const mars::agent::physical::common::RobotAgentProperties& pRAP, double& pVelocity,
      mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>&
          pTimeInfo);

  /**
   * @brief Adds a reservation to this edge.
   * @param pAgentId Agent ID of the agent which wants to reservate the entity.
   * @param pPathId Path ID of the current agent path.
   * @param pTimeInterval Interval of the reservation.
   * @return true on success, false otherwise.
   */
  bool addReservation(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
                      const mars::common::TimeInterval& pTimeInterval);

  /**
   * @brief Deletes a reservation from this edge.
   * @param pAgentId Agent ID of the agent which causes the deletion.
   * @param pPathId Path ID of the agent path.
   * @return true on success, false otherwise.
   */
  bool deleteReservation(const mars::common::Id& pAgentId, const mars::common::Id& pPathId);

  /**
   * @brief TODO: documentation, implementation
   */
  bool allocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId,
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
   * @brief Frees this edge, in order to make it usable by other agents.
   * @param pAgentId Agent ID of the agent which shall "do" the deallocation.
   * @return true on success, false otherwise.
   */
  bool deallocate(const mars::common::Id& pAgentId, const mars::common::Id& pPathId);

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's footprint.
   * @param pFootprint Footprint to be checked.
   * @return true, if passed footprint is fully within the footprint of this entity.
   */
  bool contains(const mars::common::geometry::Footprint& pFootprint);

  /**
   * @brief Checks if the passed footprint is fully inside this entitie's footprint.
   * @param pLocation Location of the robot.
   * @param pRAP Properties of the robot.
   * @return true, if passed footprint is fully within the footprint of this entity.
   */
  bool contains(const Eigen::Vector2d& pLocation,
                const mars::agent::physical::common::RobotAgentProperties& pRAP);

  bool fitsOn(mars::agent::physical::common::RobotAgentProperties& pRAP);

private:
  static mars::routing::common::Cache<mars::common::Id, mars::routing::common::topology::Edge>
      sCache;

  std::shared_ptr<mars::routing::common::topology::EdgeInterface> mInterface;
  mutable std::shared_ptr<std::recursive_mutex> mMutex;

  std::shared_ptr<std::unordered_map<
      mars::routing::common::topology::ShortestOccupationDurationKey<Vertex>, ros::Duration,
      mars::routing::common::topology::ShortestOccupationDurationKey<Vertex>::Hash>>
      mShortestOccupationDurations;

  // std::shared_ptr<std::unordered_map<mars::routing::common::topology::OccupationDurationKey<Vertex>,
  // ros::Duration,
  //                                    mars::routing::common::topology::OccupationDurationKey<Vertex>::Hash>>
  //     mOccupationDurations;

  std::shared_ptr<double> mLength;

  std::shared_ptr<int> mType;
  std::shared_ptr<Eigen::Vector3d> mLocation;
  std::shared_ptr<mars::common::geometry::Footprint> mFootprint;
  std::shared_ptr<mars::topology::common::TopologyEntityRestrictions> mRestrictions;

  std::shared_ptr<std::vector<mars::routing::common::topology::Connection>> mConnections;
};
} // namespace topology
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_TOPOLOGY_EDGE_H
