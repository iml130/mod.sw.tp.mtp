#include "mars_topology_common/TopologyEntityRestrictions.h"

static const std::string EXCEPTION_MSG_ALREADY_INITIALIZED = "TopologyEntityRestrictions are already initialized!";
static const std::string EXCEPTION_MSG_NOT_INITIALIZED = "TopologyEntityRestrictions were not initialized!";

mars::topology::common::TopologyEntityRestrictions::TopologyEntityRestrictions(void) : mIsInitialized(false)
{
}

mars::topology::common::TopologyEntityRestrictions::~TopologyEntityRestrictions(void)
{
}

mars::topology::common::TopologyEntityRestrictions::TopologyEntityRestrictions(
    const double &pMaxLinearVelocity, const double &pMaxAngularVelocity, const double &pMaxLinearAcceleration,
    const double &pMaxAngularAcceleration, const double &pMaxTotalWeight, const double &pMaxHeight,
    const std::vector<mars_topology_msgs::HazardType> &pForbiddenHazardTypes,
    const std::vector<mars_agent_physical_robot_msgs::VehicleType> &pForbiddenVehicleTypes)
  : mMaxLinearVelocity(pMaxLinearVelocity)
  , mMaxAngularVelocity(pMaxAngularVelocity)
  , mMaxLinearAcceleration(pMaxLinearAcceleration)
  , mMaxAngularAcceleration(pMaxAngularAcceleration)
  , mMaxTotalWeight(pMaxTotalWeight)
  , mMaxHeight(pMaxHeight)
  , mForbiddenHazardTypes(pForbiddenHazardTypes)
  , mForbiddenVehicleTypes(pForbiddenVehicleTypes)
  , mIsInitialized(true)
{
}

mars::topology::common::TopologyEntityRestrictions::TopologyEntityRestrictions(
    const mars_topology_srvs::GetRestrictionsResponse &pServiceResponse)
  : mMaxLinearVelocity(pServiceResponse.max_linear_velocity)
  , mMaxAngularVelocity(pServiceResponse.max_angular_velocity)
  , mMaxLinearAcceleration(pServiceResponse.max_linear_acceleration)
  , mMaxAngularAcceleration(pServiceResponse.max_angular_acceleration)
  , mMaxTotalWeight(pServiceResponse.max_total_weight)
  , mMaxHeight(pServiceResponse.max_height)
  , mForbiddenHazardTypes{ pServiceResponse.forbidden_hazard_types }
  , mForbiddenVehicleTypes{ pServiceResponse.forbidden_vehicle_types }
  , mIsInitialized(true)
{
}

void mars::topology::common::TopologyEntityRestrictions::initialize(
    const double &pMaxLinearVelocity, const double &pMaxAngularVelocity, const double &pMaxLinearAcceleration,
    const double &pMaxAngularAcceleration, const double &pMaxTotalWeight, const double &pMaxHeight,
    const std::vector<mars_topology_msgs::HazardType> &pForbiddenHazardTypes,
    const std::vector<mars_agent_physical_robot_msgs::VehicleType>
        &pForbiddenVehicleTypes) noexcept(false)
{
  if (this->mIsInitialized)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  this->mMaxLinearVelocity = pMaxLinearVelocity;
  this->mMaxAngularVelocity = pMaxAngularVelocity;
  this->mMaxLinearAcceleration = pMaxLinearAcceleration;
  this->mMaxAngularAcceleration = pMaxAngularAcceleration;
  this->mMaxTotalWeight = pMaxTotalWeight;
  this->mMaxHeight = pMaxHeight;
  this->mForbiddenHazardTypes = pForbiddenHazardTypes;
  this->mForbiddenVehicleTypes = pForbiddenVehicleTypes;
  this->mIsInitialized = true;
}

void mars::topology::common::TopologyEntityRestrictions::initialize(
    const mars_topology_srvs::GetRestrictionsResponse
        &pServiceResponse) noexcept(false)
{
  if (this->mIsInitialized)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  this->mMaxLinearVelocity = pServiceResponse.max_linear_velocity;
  this->mMaxAngularVelocity = pServiceResponse.max_angular_velocity;
  this->mMaxLinearAcceleration = pServiceResponse.max_linear_acceleration;
  this->mMaxAngularAcceleration = pServiceResponse.max_angular_acceleration;
  this->mMaxTotalWeight = pServiceResponse.max_total_weight;
  this->mMaxHeight = pServiceResponse.max_height;
  this->mForbiddenHazardTypes = { pServiceResponse.forbidden_hazard_types };
  this->mForbiddenVehicleTypes = { pServiceResponse.forbidden_vehicle_types };
  this->mIsInitialized = true;
}

void mars::topology::common::TopologyEntityRestrictions::mergeIntoServiceResponse(
    mars_topology_srvs::GetRestrictionsResponse &pServiceResponse) const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  pServiceResponse.max_linear_velocity = this->mMaxLinearVelocity;
  pServiceResponse.max_angular_velocity = this->mMaxAngularVelocity;
  pServiceResponse.max_linear_acceleration = this->mMaxLinearAcceleration;
  pServiceResponse.max_angular_acceleration = this->mMaxAngularAcceleration;
  pServiceResponse.max_total_weight = this->mMaxTotalWeight;
  pServiceResponse.max_height = this->mMaxHeight;
  pServiceResponse.forbidden_hazard_types = { this->mForbiddenHazardTypes };
  pServiceResponse.forbidden_vehicle_types = { this->mForbiddenVehicleTypes };
}

double mars::topology::common::TopologyEntityRestrictions::getMaxLinearVelocity() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxLinearVelocity;
}

double mars::topology::common::TopologyEntityRestrictions::getMaxAngularVelocity() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxAngularVelocity;
}

double mars::topology::common::TopologyEntityRestrictions::getMaxLinearAcceleration() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxLinearAcceleration;
}

double mars::topology::common::TopologyEntityRestrictions::getMaxAngularAcceleration() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxAngularAcceleration;
}

double mars::topology::common::TopologyEntityRestrictions::getMaxTotalWeight() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxTotalWeight;
}

double mars::topology::common::TopologyEntityRestrictions::getMaxHeight() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mMaxHeight;
}

const std::vector<mars_topology_msgs::HazardType> &
mars::topology::common::TopologyEntityRestrictions::getForbiddenHazardTypes() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mForbiddenHazardTypes;
}

const std::vector<mars_agent_physical_robot_msgs::VehicleType> &
mars::topology::common::TopologyEntityRestrictions::getForbiddenVehicleTypes() const
    noexcept(false)
{
  if (!this->mIsInitialized)
  {
    throw mars::common::exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mForbiddenVehicleTypes;
}