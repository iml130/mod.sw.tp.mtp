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

#include "mars_routing_core/Route.h"

#include "mars_common/Color.h"
#include "mars_routing_common/topology/Edge.h"
#include "mars_routing_common/topology/Vertex.h"

static const std::string EXCEPTION_MSG_ROUTE_INVALID = "The Route is marked as invalid!";

template <class Origin, class Target, class Destination>
ros::Time mars::routing::core::Route<Origin, Target, Destination>::sLatestTime = ros::Time(0, 0);

template <class Origin, class Target, class Destination>
int mars::routing::core::Route<Origin, Target, Destination>::sVisualizationID = 0;

template <class Origin, class Target, class Destination>
mars::routing::core::Route<Origin, Target, Destination>::Route(Origin& pOrigin,
                                                               Destination& pDestination)
    : mOrigin(pOrigin), mDestination(pDestination),
      mTravelInterval(mars::common::TimeInterval::INFINITE_END_TIME,
                      mars::common::TimeInterval::INFINITE_DURATION),
      mTravelDistance(0), mValid(false), mStepCount(0), mFirstStep(nullptr),
      mVisualizationInitialized(false)
{
  this->mId.initialize();
  this->mVisualizationID = sVisualizationID++;
}

template <class Origin, class Target, class Destination>
mars::routing::core::Route<Origin, Target, Destination>::Route(
    const mars_routing_srvs::GetRoute& pService)
    : mId(pService.response.path_id), mOrigin(mars::common::Id(pService.request.origin.id)),
      mDestination(mars::common::Id(pService.request.destination.id)),
      mTravelInterval(mars::common::TimeInterval::INFINITE_END_TIME,
                      mars::common::TimeInterval::INFINITE_DURATION),
      mTravelDistance(0), mValid(false), mStepCount(0), mFirstStep(nullptr)
{

  if (pService.response.result.result != pService.response.result.RESULT_ERROR)
  {
    this->mValid = true;

    this->mVisualizationID = pService.response.visualization_id;

    mars::routing::core::IterationStep* lIterationStep = nullptr;

    for (const mars_routing_msgs::Step& iStepMsg : pService.response.route)
    {
      this->mStepCount++;

      if (iStepMsg.origin.entity_type.entity_type >=
              mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MIN &&
          iStepMsg.origin.entity_type.entity_type <=
              mars_topology_msgs::TopologyEntityType::TOPOLOGY_VERTEX_TYPE_MAX)
      {
        if (lIterationStep == nullptr)
        {
          mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                    mars::routing::common::topology::Edge>* lCurrentStep =
              new mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                            mars::routing::common::topology::Edge>(iStepMsg);

          this->mTravelInterval.setStartTime(
              lCurrentStep->getTargetOccupationInterval().getStartTime());
          this->mFirstStep = (mars::routing::core::Step<Origin, Target>*)lCurrentStep;
          lIterationStep = lCurrentStep;
        }
        else
        {
          mars::routing::core::Step<mars::routing::common::topology::Edge,
                                    mars::routing::common::topology::Vertex>* lPreviousStep =
              (mars::routing::core::Step<mars::routing::common::topology::Edge,
                                         mars::routing::common::topology::Vertex>*)lIterationStep;

          mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                    mars::routing::common::topology::Edge>* lCurrentStep =
              new mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                            mars::routing::common::topology::Edge>(iStepMsg);

          lPreviousStep->setNext(lCurrentStep);
          lCurrentStep->setPrevious(lPreviousStep);

          if (iStepMsg == pService.response.route.back())
          {
            this->mLastStep = (mars::routing::core::Step<Origin, Target>*)lCurrentStep;
          }
          else
          {
            lIterationStep = lCurrentStep;
          }
        }
      }
      else if (iStepMsg.origin.entity_type.entity_type >=
                   mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MIN &&
               iStepMsg.origin.entity_type.entity_type <=
                   mars_topology_msgs::TopologyEntityType::TOPOLOGY_EDGE_TYPE_MAX)
      {
        if (lIterationStep == nullptr)
        {
          mars::routing::core::Step<mars::routing::common::topology::Edge,
                                    mars::routing::common::topology::Vertex>* lCurrentStep =
              new mars::routing::core::Step<mars::routing::common::topology::Edge,
                                            mars::routing::common::topology::Vertex>(iStepMsg);

          this->mTravelInterval.setStartTime(
              lCurrentStep->getTargetOccupationInterval().getStartTime());
          this->mFirstStep = (mars::routing::core::Step<Origin, Target>*)lCurrentStep;
          lIterationStep = lCurrentStep;
        }
        else
        {
          mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                    mars::routing::common::topology::Edge>* lPreviousStep =
              (mars::routing::core::Step<mars::routing::common::topology::Vertex,
                                         mars::routing::common::topology::Edge>*)lIterationStep;

          mars::routing::core::Step<mars::routing::common::topology::Edge,
                                    mars::routing::common::topology::Vertex>* lCurrentStep =
              new mars::routing::core::Step<mars::routing::common::topology::Edge,
                                            mars::routing::common::topology::Vertex>(iStepMsg);

          lPreviousStep->setNext(lCurrentStep);
          lCurrentStep->setPrevious(lPreviousStep);

          if (iStepMsg == pService.response.route.back())
          {
            this->mLastStep = (mars::routing::core::Step<Origin, Target>*)lCurrentStep;
          }
          else
          {
            lIterationStep = lCurrentStep;
          }
        }
      }
      else
      {
        // TODO: Unknown Topology Entity! Error handling!
      }

      this->mTravelDistance += lIterationStep->getDistance();
    }
    if (lIterationStep != nullptr)
    {
      this->mTravelInterval.setDuration(
          (lIterationStep->getTargetOccupationInterval().getStartTime()
           // + lIterationStep->getTargetOccupationInterval().getDuration()
           - this->mTravelInterval.getStartTime()));
    }
  }
}

template <class Origin, class Target, class Destination>
mars::routing::core::Route<Origin, Target, Destination>::~Route()
{
  if (this->mFirstStep == nullptr)
  {
    return;
  }

  mars::routing::core::Route<Origin, Target, Destination>::iterator lStepIterator(
      (mars::routing::core::IterationStep*)mFirstStep);

  while (lStepIterator != mars::routing::core::Route<Origin, Target, Destination>::end())
  {
    mars::routing::core::Route<Origin, Target, Destination>::iterator lNextStepIterator(
        lStepIterator->getNext());
    delete lStepIterator.getStep();
    lStepIterator = lNextStepIterator;
  }
}

template <class Origin, class Target, class Destination>
const mars::common::Id& mars::routing::core::Route<Origin, Target, Destination>::getId() const
{
  return this->mId;
}

template <class Origin, class Target, class Destination>
void mars::routing::core::Route<Origin, Target, Destination>::mergeIntoServiceResponse(
    mars_routing_srvs::GetRouteResponse& pServiceResponse) const
{
  pServiceResponse.path_id = this->mId.toMsg();

  if (this->isValid())
  {
    pServiceResponse.result.result = pServiceResponse.result.RESULT_SUCCESS;
    for (mars::routing::core::IterationStep& iStep : *this)
    {
      pServiceResponse.route.push_back(iStep.toMsg());
    }
    pServiceResponse.visualization_id = this->mVisualizationID;
  }
  else
  {
    pServiceResponse.result.result = pServiceResponse.result.RESULT_ERROR;
  }
}

template <class Origin, class Target, class Destination>
const mars::common::TimeInterval&
mars::routing::core::Route<Origin, Target, Destination>::getTravelInterval() const noexcept(false)
{
  if (!this->isValid())
  {
    throw exception::InvalidRouteException(EXCEPTION_MSG_ROUTE_INVALID);
  }

  return this->mTravelInterval;
}

template <class Origin, class Target, class Destination>
double mars::routing::core::Route<Origin, Target, Destination>::getTravelDistance() const
    noexcept(false)
{
  if (!this->isValid())
  {
    throw exception::InvalidRouteException(EXCEPTION_MSG_ROUTE_INVALID);
  }

  return this->mTravelDistance;
}

template <class Origin, class Target, class Destination>
Origin& mars::routing::core::Route<Origin, Target, Destination>::getOrigin()
{
  return this->mOrigin;
}

template <class Origin, class Target, class Destination>
Destination& mars::routing::core::Route<Origin, Target, Destination>::getDestination()
{
  return this->mDestination;
}

template <class Origin, class Target, class Destination>
unsigned int mars::routing::core::Route<Origin, Target, Destination>::getStepCount() const
{
  return this->mStepCount;
}

template <class Origin, class Target, class Destination>
bool mars::routing::core::Route<Origin, Target, Destination>::isValid() const
{
  return this->mValid;
}

template <class Origin, class Target, class Destination>
typename mars::routing::core::Route<Origin, Target, Destination>::iterator
mars::routing::core::Route<Origin, Target, Destination>::begin() const noexcept(false)
{
  if (!this->isValid())
  {
    throw exception::InvalidRouteException(EXCEPTION_MSG_ROUTE_INVALID);
  }

  return mars::routing::core::RouteIterator(this->mFirstStep);
}

template <class Origin, class Target, class Destination>
typename mars::routing::core::Route<Origin, Target, Destination>::iterator
mars::routing::core::Route<Origin, Target, Destination>::end() const
{
  return mars::routing::core::RouteIterator();
}

template <class Origin, class Target, class Destination>
visualization_msgs::Marker
mars::routing::core::Route<Origin, Target, Destination>::visualize(const ros::Time& pStartTime)
{

  visualization_msgs::Marker lMarker;

  lMarker.header.frame_id = "map";
  lMarker.header.stamp = ros::Time::now();
  lMarker.ns = "route_visualization";
  lMarker.id = this->mVisualizationID;
  lMarker.pose.orientation.w = 1.0;
  lMarker.type = visualization_msgs::Marker::LINE_STRIP;
  lMarker.scale.x = 0.2;

  if (!this->mVisualizationInitialized)
  {
    // Update global last time point by *this

    sLatestTime =
        std::max(sLatestTime, this->mLastStep->getTargetOccupationInterval().getStartTime());
    if (this->mVisualizationID == 0)
    {
      this->mHueDegree = 0;
    }
    else
    {
      /**
       * Here, the range of hue is semi-equally split among the existing routes.
       * This happens by splitting the hue range in powers of 2. To this end we define the bracket
       * as the smallest power-of-2 exponent a given ID fits into, eg. [4, 5, 6, 7] -> bracket 3
       */
      int lBracket = std::floor(std::log2(this->mVisualizationID) + 1);
      int lNumerator = 2 * (this->mVisualizationID % (int)std::pow(2, lBracket - 1)) + 1;
      int lDenominator = std::pow(2, lBracket);
      this->mHueDegree = 360.0 * (double)lNumerator / (double)lDenominator;
    }
    this->mVisualizationInitialized = true;
  }

  mars::common::HSV lHSV{this->mHueDegree, 1.0, 1.0};

  lMarker.points.reserve(this->mStepCount);
  lMarker.color = mars::common::hsv2msg(lHSV);

  /**
   *   Each step adds 3 points {p0, p1, p2} to the line strip, where the line segment p1-p2
   * symbolizes the reservation duration the entity In the sketch below, x_i resembles the target
   * position of a step and [start_i, end_i] the reservation interval of its target
   *
   *       time ^               p2
   *            |
   *    end_i   |               X---------X p0
   *            |     p2        |         |
   *  start_i+1 |               |         X p1
   *    end_i-1 +     X---------X p0
   *            |     |         |
   *  start_i   +     |         X p1
   *    end_i-2 +  ---X p0
   *            |     |
   *  start_i-1 +     X p1
   *            |___________________________________>
   *                 x_i-1      x_i     x_i+1      position
   */

  ros::Time lRelativeStartTime = (pStartTime.is_zero() ? ros::Time::now() : pStartTime);

  double lMaxHeight = std::min((sLatestTime - lRelativeStartTime).toSec(), 20.0);

  ros::Time lPrevEndTime = this->begin()
                               ->getTargetOccupationInterval()
                               .getStartTime(); // On the first step there is no previous step's end
                                                // time, so its p0 will just be the same as p1

  for (mars::routing::core::IterationStep& iStep : *this)
  {
    Eigen::Vector2d lPos = iStep.getTarget().getLocation().get().head(2);

    double lp0HeightParam = (std::min(lPrevEndTime, sLatestTime) - lRelativeStartTime).toSec() /
                            std::max(0.001, (sLatestTime - lRelativeStartTime).toSec());

    double lp1HeightParam =
        (std::min(iStep.getTargetOccupationInterval().getStartTime(), sLatestTime) -
         lRelativeStartTime)
            .toSec() /
        std::max(0.001, (sLatestTime - lRelativeStartTime).toSec());

    double lp2HeightParam =
        (std::min(iStep.getTargetOccupationInterval().getEndTime(), sLatestTime) - lRelativeStartTime)
            .toSec() /
        std::max(0.001, (sLatestTime - lRelativeStartTime).toSec());

    for (double iHeightParam : std::vector<double>{lp0HeightParam, lp1HeightParam, lp2HeightParam})
    {
      geometry_msgs::Point lPoint;
      lPoint.x = lPos[0];
      lPoint.y = lPos[1];
      double lzMin =
          -0.05 * this->mVisualizationID -
          1.0; // Keep routes 1 m below ground, offsetting slightly from one another for visibility
      lPoint.z = std::max(lzMin, 0.1 + lMaxHeight * std::min(1.0, iHeightParam));
      lMarker.points.push_back(lPoint);
    }

    lPrevEndTime = iStep.getTargetOccupationInterval().getEndTime();
  }

  lMarker.points.pop_back(); // Remove last point for now, including the end of destination
                             // reservation is skews the visualization
  return lMarker;
}

template class mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                          mars::routing::common::topology::Edge,
                                          mars::routing::common::topology::Vertex>;

template class mars::routing::core::Route<mars::routing::common::topology::Vertex,
                                          mars::routing::common::topology::Edge,
                                          mars::routing::common::topology::Edge>;

template class mars::routing::core::Route<mars::routing::common::topology::Edge,
                                          mars::routing::common::topology::Vertex,
                                          mars::routing::common::topology::Vertex>;

template class mars::routing::core::Route<mars::routing::common::topology::Edge,
                                          mars::routing::common::topology::Vertex,
                                          mars::routing::common::topology::Edge>;
