#ifndef MARS_ROUTING_PLUGIN_CARP_ROUTE_H
#define MARS_ROUTING_PLUGIN_CARP_ROUTE_H

#include "mars_common/TimeInterval.h"

#include "mars_agent_physical_common/RobotAgentProperties.h"
#include "mars_routing_core/Route.h"
#include "mars_routing_plugin_carp/PlanningRoute.h"
#include "mars_routing_plugin_carp/PlanningStep.h"
#include "mars_routing_plugin_carp/Step.h"

#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

#include <ros/time.h>

namespace mars
{
namespace routing
{
namespace plugin
{
namespace carp
{
/**
 * @class Route
 */
template <class Origin, class Target, class Destination, class MotionProfile>
class Route : public mars::routing::core::Route<Origin, Target, Destination>,
              public mars::routing::plugin::carp::PlanningRoute
{
public:
  Route(
      Origin& pOrigin, Destination& pDestination,
      const mars::agent::physical::common::RobotAgentProperties& pRAP,
      const double pOrientation2D, const ros::Time& pStartTime,
      const ros::Duration& pDestinationReservationDuration = ros::Duration(-1));

  bool reserve(const mars::common::Id& pAgentId);

private:
  ros::Duration mDestinationReservationDuration;

  bool
  checkStepValid(const mars::routing::plugin::carp::PlanningStep& pStep);

  void printStepTimeInfo(mars::routing::plugin::carp::PlanningStep* pStep);

  void handleEqualDestination();

  void handleTraversableDestination(
      const mars::agent::physical::common::RobotAgentProperties& pRAP,
      const double pOrientation2D, const ros::Time& pStartTime);

  void handleUntraversableDestination();

  void handleUnreachableDestination();

  std::vector<mars::routing::plugin::carp::PlanningStep*> initiateRouting(
      const mars::agent::physical::common::RobotAgentProperties& pRAP,
      const double pOrientation2D, const ros::Time& pStartTime);

  /**
   * @brief Initializes the doubly linked list that makes up the route by
   * copying the relevant planning steps and augmenting them with the timing
   * information
   *
   * @param pLastStep Last step to the routing destination, allowing
   * backtracking of the route
   */
  void initializeSteps(mars::routing::plugin::carp::PlanningStep* pLastStep);

  void printRouteTimeInfo(mars::routing::plugin::carp::PlanningStep* pLastStep);
};
} // namespace carp
} // namespace plugin
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_PLUGIN_CARP_ROUTE_H
