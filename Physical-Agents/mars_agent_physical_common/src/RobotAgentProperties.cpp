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

#include "mars_agent_physical_common/RobotAgentProperties.h"

#include <algorithm>

mars::agent::physical::common::RobotAgentProperties::RobotAgentProperties(
    const mars::common::Id& pId,
    const mars_agent_physical_robot_msgs::VehicleType& pVehicleType,
    const mars::common::geometry::Footprint& pFootprint,
    double pForwardVelocity, double pBackwardVelocity,
    double pLinearAcceleration, double pLinearDeceleration,
    double pAngularVelocity, double pAngularAcceleration,
    double pAngularDeceleration, double pMinHeight, double pMaxHeight,
    double pWeight, double pPayload, double pTurningRadius)
    : mId(pId), mVehicleType(pVehicleType), mFootprint(pFootprint),
      mForwardVelocity(pForwardVelocity), mBackwardVelocity(pBackwardVelocity),
      mLinearAcceleration(pLinearAcceleration),
      mLinearDeceleration(pLinearDeceleration),
      mAngularVelocity(pAngularVelocity),
      mAngularAcceleration(pAngularAcceleration),
      mAngularDeceleration(pAngularDeceleration), mMinHeight(pMinHeight),
      mMaxHeight(pMaxHeight), mWeight(pWeight), mPayload(pPayload),
      mTurningRadius(pTurningRadius)
{
}

mars::agent::physical::common::RobotAgentProperties::RobotAgentProperties(
    const mars_agent_physical_robot_msgs::RobotAgentProperties&
        pRobotAgentProperties)
    : mId(pRobotAgentProperties.robot_id),
      mVehicleType(pRobotAgentProperties.type), mFootprint(),
      mForwardVelocity(pRobotAgentProperties.max_pos_x_vel),
      mBackwardVelocity(pRobotAgentProperties.max_neg_x_vel),
      mLinearAcceleration(pRobotAgentProperties.max_pos_x_acc),
      mLinearDeceleration(pRobotAgentProperties.max_neg_x_acc),
      mAngularVelocity(pRobotAgentProperties.max_pos_ang_vel),
      mAngularAcceleration(pRobotAgentProperties.max_pos_ang_acc),
      mAngularDeceleration(pRobotAgentProperties.max_neg_ang_acc),
      mMinHeight(pRobotAgentProperties.min_height),
      mMaxHeight(pRobotAgentProperties.max_height),
      mWeight(pRobotAgentProperties.weight),
      mPayload(pRobotAgentProperties.payload),
      mTurningRadius(pRobotAgentProperties.min_turning_radius)
{
  for (auto& i_point : pRobotAgentProperties.footprint.polygon.points)
  {
    mFootprint.push_back(Eigen::Vector2d(i_point.x, i_point.y));
  }
}

bool mars::agent::physical::common::RobotAgentProperties::match(
    const mars::topology::common::TopologyEntityRestrictions&
        pTopologyEntityRestrictions) const
{
  const auto& lForbiddenVehicleTypeMsgs =
      pTopologyEntityRestrictions.getForbiddenVehicleTypes();
  std::vector<mars_agent_physical_robot_msgs::VehicleType::_vehicle_type_type>
      lForbiddenVehicleTypes;

  for (const auto& iForbiddenVehicleTypeMsg : lForbiddenVehicleTypeMsgs)
  {
    lForbiddenVehicleTypes.push_back(iForbiddenVehicleTypeMsg.vehicle_type);
  }

  bool lIsAllowedVehicleType =
      std::find(lForbiddenVehicleTypes.begin(), lForbiddenVehicleTypes.end(),
                this->mVehicleType.vehicle_type) ==
      lForbiddenVehicleTypes.end();

  bool lIsSmallEnough =
      this->mMaxHeight <= pTopologyEntityRestrictions.getMaxHeight();

  bool lIsLightEnough = this->mWeight + this->mPayload <=
                        pTopologyEntityRestrictions.getMaxTotalWeight();

  return (lIsAllowedVehicleType && lIsSmallEnough && lIsLightEnough);
}

bool mars::agent::physical::common::RobotAgentProperties::match(
    const mars::topology::common::TopologyEntityRestrictions&
        pTopologyEntityRestrictions,
    const mars_topology_msgs::HazardType& pHazardType) const
{
  bool lMatchesHazards = true; // TODO: Implement correct hazard check
  return lMatchesHazards && this->match(pTopologyEntityRestrictions);
}

const mars_agent_physical_robot_msgs::VehicleType&
mars::agent::physical::common::RobotAgentProperties::getVehicleType() const
{
  return this->mVehicleType;
}

const mars::common::geometry::Footprint&
mars::agent::physical::common::RobotAgentProperties::getFootprint() const
{
  return this->mFootprint;
}

double
mars::agent::physical::common::RobotAgentProperties::getForwardVelocity() const
{
  return this->mForwardVelocity;
}

double
mars::agent::physical::common::RobotAgentProperties::getBackwardVelocity() const
{
  return this->mBackwardVelocity;
}

double
mars::agent::physical::common::RobotAgentProperties::getLinearAcceleration()
    const
{
  return this->mLinearAcceleration;
}

double
mars::agent::physical::common::RobotAgentProperties::getLinearDeceleration()
    const
{
  return this->mLinearDeceleration;
}

double
mars::agent::physical::common::RobotAgentProperties::getAngularVelocity() const
{
  return this->mAngularVelocity;
}

double
mars::agent::physical::common::RobotAgentProperties::getAngularAcceleration()
    const
{
  return this->mAngularAcceleration;
}

double
mars::agent::physical::common::RobotAgentProperties::getAngularDeceleration()
    const
{
  return this->mAngularDeceleration;
}

double mars::agent::physical::common::RobotAgentProperties::getMinHeight() const
{
  return this->mMinHeight;
}

double mars::agent::physical::common::RobotAgentProperties::getMaxHeight() const
{
  return this->mMaxHeight;
}

double mars::agent::physical::common::RobotAgentProperties::getWeight() const
{
  return this->mWeight;
}

double mars::agent::physical::common::RobotAgentProperties::getPayload() const
{
  return this->mPayload;
}

double
mars::agent::physical::common::RobotAgentProperties::getCollisionRadius() const
{
  double lCollisionRadius = 0;

  for (const Eigen::Matrix<double, 2, 1>& iPoint : this->mFootprint)
  {
    double lRadius = iPoint.norm();

    if (lRadius > lCollisionRadius)
    {
      lCollisionRadius = lRadius;
    }
  }

  return lCollisionRadius;
}

double
mars::agent::physical::common::RobotAgentProperties::getTurningRadius() const
{
  return this->mTurningRadius;
}

mars_agent_physical_robot_msgs::RobotAgentProperties
mars::agent::physical::common::RobotAgentProperties::toMsg() const
{
  mars_agent_physical_robot_msgs::RobotAgentProperties lRAPMsg;
  // lRAPMsg.max_angular_acceleration = this->mAngularAcceleration;
  // lRAPMsg.max_angular_deceleration = this->mAngularDeceleration;
  // lRAPMsg.max_angular_velocity = this->mAngularVelocity;
  // lRAPMsg.max_linear_acceleration = this->mLinearAcceleration;
  // lRAPMsg.max_linear_deceleration = this->mLinearDeceleration;
  // lRAPMsg.max_backward_velocity = this->mBackwardVelocity;
  // lRAPMsg.max_forward_velocity = this->mForwardVelocity;
  // lRAPMsg.max_height = this->mMaxHeight;
  // lRAPMsg.min_height = this->mMinHeight;
  // lRAPMsg.max_payload = this->mPayload;
  // lRAPMsg.min_turning_radius = this->mTurningRadius;
  // lRAPMsg.robot_id = this->mId.toMsg();
  // lRAPMsg.type = this->mVehicleType;
  // lRAPMsg.weight = this->mWeight;

  lRAPMsg.robot_id = this->mId.toMsg();
  lRAPMsg.type = this->mVehicleType;
  lRAPMsg.min_height = this->mMinHeight;
  lRAPMsg.max_height = this->mMaxHeight;
  lRAPMsg.payload = this->mPayload;
  lRAPMsg.max_pos_x_vel = this->mForwardVelocity;
  lRAPMsg.max_neg_x_vel = this->mBackwardVelocity;
  lRAPMsg.max_pos_x_acc = this->mLinearAcceleration;
  lRAPMsg.max_neg_x_acc = this->mLinearDeceleration;
  lRAPMsg.max_pos_y_vel = 0;
  lRAPMsg.max_neg_y_vel = 0;
  lRAPMsg.max_pos_y_acc = 0;
  lRAPMsg.max_neg_y_acc = 0;
  lRAPMsg.max_pos_ang_vel = this->mAngularVelocity;
  lRAPMsg.max_neg_ang_vel = this->mAngularVelocity;
  lRAPMsg.max_pos_ang_acc = this->mAngularAcceleration;
  lRAPMsg.max_neg_ang_acc = this->mAngularDeceleration;
  lRAPMsg.velocity_control_sensitivity = 0; // INFO:
  lRAPMsg.min_turning_radius = this->mTurningRadius;
  lRAPMsg.batt_capacity = 1;
  lRAPMsg.batt_max_voltage = 24;
  lRAPMsg.weight = this->mWeight;
  lRAPMsg.vendor = "";

  for (const Eigen::Vector2d& iVector : this->mFootprint)
  {
    geometry_msgs::Point32 lPoint;
    lPoint.x = iVector[0];
    lPoint.y = iVector[1];
    lPoint.z = 0;
    lRAPMsg.footprint.polygon.points.push_back(lPoint);
  }

  return lRAPMsg;
}

double
mars::agent::physical::common::RobotAgentProperties::getFrontLength() const
{
  // Search for the needed dimensions of the vehicle
  double lMaxXValue =
      (std::max_element(
           this->mFootprint.begin(), this->mFootprint.end(),
           [](Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) {
             return lhs.x() < rhs.x();
           }))
          ->x();
  return std::abs(lMaxXValue);
}

double
mars::agent::physical::common::RobotAgentProperties::getBackLength() const
{
  double lMinXValue =
      (std::min_element(
           this->mFootprint.begin(), this->mFootprint.end(),
           [](Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) {
             return lhs.x() < rhs.x();
           }))
          ->x();

  return std::abs(lMinXValue);
}
