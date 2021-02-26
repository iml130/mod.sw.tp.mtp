#include "mars_topology_container/EntityParser.h"

#include <xmlrpcpp/XmlRpcException.h>

// vertex parameter names with no default assignable value
static const std::string ENTITY_ID_PARAM_NAME = "id";
static const std::string ENTITY_ID_DESCRIPTION_PARAM_NAME = "description";
static const std::string ENTITY_FRAME_ID_PARAM_NAME = "frame_id"
                                                      "";
static const std::string ENTITY_X_COORDINATE_PARAM_NAME = "x_pos";
static const std::string ENTITY_Y_COORDINATE_PARAM_NAME = "y_pos";
static const std::string ENTITY_TYPE_PARAM_NAME = "type";
static const std::string ENTITY_IS_LOCKED_PARAM_NAME = "is_locked";
static const std::string ENTITY_FOOTPRINT_X_PARAM_NAME = "footprint_x";
static const std::string ENTITY_FOOTPRINT_Y_PARAM_NAME = "footprint_y";
static const std::string ENTITY_MAXIMUM_LINEAR_VELOCITY_PARAM_NAME = "maximum_linear_velocity";
static const std::string ENTITY_MAXIMUM_ANGULAR_VELOCITY_PARAM_NAME = "maximum_angular_velocity";
static const std::string ENTITY_MAXIMUM_LINEAR_ACCELERATION_PARAM_NAME =
    "maximum_linear_acceleration";
static const std::string ENTITY_MAXIMUM_ANGULAR_ACCELERATION_PARAM_NAME =
    "maximum_angular_acceleration";
static const std::string ENTITY_MAXIMUM_HEIGHT_PARAM_NAME = "maximum_height";
static const std::string ENTITY_MAXIMUM_WEIGHT_PARAM_NAME = "maximum_total_weight";
static const std::string VERTEX_INGOING_EDGES_PARAM_NAME = "ingoing_edge_ids";
static const std::string VERTEX_INGOING_CONTAINER_IDS_PARAM_NAME = "ingoing_container_ids";
static const std::string VERTEX_OUTGOING_CONTAINER_IDS_PARAM_NAME = "outgoing_container_ids";
static const std::string VERTEX_OUTGOING_EDGES_PARAM_NAME = "outgoing_edge_ids";
static const std::string EDGE_ORIGIN_ID_PARAM_NAME = "origin_id";
static const std::string EDGE_ORIGIN_CONTAINER_ID_PARAM_NAME = "origin_container_id";
static const std::string EDGE_DESTINATION_CONTAINER_ID_PARAM_NAME = "destination_container_id";
static const std::string EDGE_DESTINATION_ID_PARAM_NAME = "destination_id";
static const std::string EDGE_LENGTH_PARAM_NAME = "length";
static const std::string EDGE_DIRECTION_PARAM_NAME = "direction";

static const std::string EXEPTION_MSG_INVALID_GIVEN_ENTITY =
    "the entity type is either wrong or not defined";
static const std::string EXEPTION_MSG_PARSING_ERROR =
    "(ReadParamException) While parsing an Exception occurred: ";

// overload the << operator for std::vector to print vector elements
template <class T> std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << "[";

  for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    if (it == v.begin())
    {
      os << *it;
    }
    else
    {
      os << ", " << *it;
    }
  }

  os << "]";

  return os;
}

template <typename T>
const T mars::topology::container::EntityParser::getParam(XmlRpc::XmlRpcValue& pEntity,
                                                          const std::string& pParamName) const
    noexcept(false)
{
  if (pEntity.hasMember(pParamName))
  {
    T value = static_cast<T>(pEntity[pParamName]);

    MARS_LOG_DEBUG("Found parameter: " + pParamName + ", value: " << value);
    return value;
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + pParamName);
  }
}

template <>
const float mars::topology::container::EntityParser::getParam<float>(
    XmlRpc::XmlRpcValue& pEntity, const std::string& pParamName) const noexcept(false)
{
  if (pEntity.hasMember(pParamName))
  {
    float value = static_cast<float>(static_cast<double>(pEntity[pParamName]));

    MARS_LOG_DEBUG("Found parameter: " + pParamName + ", value: " << value);
    return value;
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + pParamName);
  }
}

template <typename T>
const std::vector<T>
mars::topology::container::EntityParser::getParamVector(XmlRpc::XmlRpcValue& pEntity,
                                                        const std::string& pParamName) const
{
  std::vector<T> lParam;
  if (pEntity.hasMember(pParamName))
  {
    for (int i = 0; i < pEntity[pParamName].size(); ++i)
    {
      lParam.push_back(static_cast<T>(pEntity[pParamName][i]));
    }
  }
  return lParam;
}

template <>
const std::vector<float>
mars::topology::container::EntityParser::getParamVector<float>(XmlRpc::XmlRpcValue& pEntity,
                                                               const std::string& pParamName) const
{
  std::vector<float> lParam;
  if (pEntity.hasMember(pParamName))
  {
    for (int i = 0; i < pEntity[pParamName].size(); ++i)
    {
      lParam.push_back(static_cast<float>(static_cast<double>(pEntity[pParamName][i])));
    }
  }
  return lParam;
}

template <typename T>
const T mars::topology::container::EntityParser::getParam(XmlRpc::XmlRpcValue& pEntity,
                                                          const std::string& pParamName,
                                                          const T& pDefaultValue) const
{
  if (pEntity.hasMember(pParamName))
  {
    T value = T(pEntity[pParamName]);
    MARS_LOG_DEBUG("Found parameter: " + pParamName + ", value: " << value);
    return value;
  }
  else
  {
    MARS_LOG_DEBUG("Cannot find value for parameter: " + pParamName + ", assigning default: "
                   << pDefaultValue);
    return pDefaultValue;
  }
}

mars::topology::container::EntityParser::EntityParser() {}

void mars::topology::container::EntityParser::parse(
    XmlRpc::XmlRpcValue& pEntities, mars::topology::container::ContainerHandle& pContainer)
{
  // check if there are any topology entity entries
  if (pEntities.getType() == XmlRpc::XmlRpcValue::Type::TypeArray && pEntities.size() > 0)
  {
    ROS_WARN_STREAM("Number of entities " << pEntities.size() << std::endl);
    // iterate through all entities of the container
    for (int i = 0; i < pEntities.size(); i++)
    {
      // check if the entity is a map and has a member which represents the
      // entity type
      if (pEntities[i].getType() == XmlRpc::XmlRpcValue::Type::TypeStruct &&
          pEntities[i].hasMember("entity_type"))
      {

        // check which entity type is represented
        if (pEntities[i]["entity_type"] == "vertex")
        {

          pContainer.addVertex(parseVertex(pEntities[i]));
        }
        else if (pEntities[i]["entity_type"] == "edge")
        {
          pContainer.addEdge(parseEdge(pEntities[i]));
        }
        else
        {
          throw mars::common::exception::SetParamException(EXEPTION_MSG_INVALID_GIVEN_ENTITY);
        }
      }
    }
  }
}

std::shared_ptr<mars::topology::vertex::MarsVertex>
mars::topology::container::EntityParser::parseVertex(XmlRpc::XmlRpcValue& pVertex)
{
  try
  {
    const std::string lId = getParam<std::string>(pVertex, ENTITY_ID_PARAM_NAME);
    const std::string lDescription =
        getParam<std::string>(pVertex, ENTITY_ID_DESCRIPTION_PARAM_NAME);
    const std::string lFrameId = getParam<std::string>(pVertex, ENTITY_FRAME_ID_PARAM_NAME);
    const float lCoordinateX = getParam<float>(pVertex, ENTITY_X_COORDINATE_PARAM_NAME);
    const float lCoordinateY = getParam<float>(pVertex, ENTITY_Y_COORDINATE_PARAM_NAME);
    const int lType = getParam<int>(pVertex, ENTITY_TYPE_PARAM_NAME);
    const bool lIsLocked = getParam<bool>(pVertex, ENTITY_IS_LOCKED_PARAM_NAME);
    // const std::vector<float> lFootprintX={11.0, 15.0, 3.0};
    // const std::vector<float> lFootprintY={2,3,7};
    const std::vector<float> lFootprintX =
        getParamVector<float>(pVertex, ENTITY_FOOTPRINT_X_PARAM_NAME);
    const std::vector<float> lFootprintY =
        getParamVector<float>(pVertex, ENTITY_FOOTPRINT_Y_PARAM_NAME);
    const std::vector<std::string> lIngoingEdgesIds =
        getParamVector<std::string>(pVertex, VERTEX_INGOING_EDGES_PARAM_NAME);
    const std::vector<std::string> lIngoingContainerIds =
        getParamVector<std::string>(pVertex, VERTEX_INGOING_CONTAINER_IDS_PARAM_NAME);
    const std::vector<std::string> lOutgoingEdgesIds =
        getParamVector<std::string>(pVertex, VERTEX_OUTGOING_EDGES_PARAM_NAME);
    const std::vector<std::string> lOutgoingContainerIds =
        getParamVector<std::string>(pVertex, VERTEX_OUTGOING_CONTAINER_IDS_PARAM_NAME);
    const mars::topology::common::TopologyEntityRestrictions lRestrictions(
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_LINEAR_VELOCITY_PARAM_NAME),
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_ANGULAR_VELOCITY_PARAM_NAME),
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_LINEAR_ACCELERATION_PARAM_NAME),
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_ANGULAR_ACCELERATION_PARAM_NAME),
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_WEIGHT_PARAM_NAME),
        this->getParam<double>(pVertex, ENTITY_MAXIMUM_HEIGHT_PARAM_NAME), {}, {});

    std::shared_ptr<mars::topology::vertex::MarsVertex> lVertex =
        std::make_shared<mars::topology::vertex::MarsVertex>(
            lId, lDescription, lType, lIsLocked, lCoordinateX, lCoordinateY, lFootprintX,
            lFootprintY, lFrameId, mars::common::Id::createId(lOutgoingEdgesIds),
            lOutgoingContainerIds, mars::common::Id::createId(lIngoingEdgesIds),
            lIngoingContainerIds, lRestrictions);

    return lVertex;
  }
  catch (mars::common::exception::ReadParamException e)
  {
    MARS_LOG_ERROR(EXEPTION_MSG_PARSING_ERROR << e.what());
  }
  catch (XmlRpc::XmlRpcException e)
  {
    MARS_LOG_ERROR(e.getMessage() << "(" << e.getCode() << ")");
  }
}

std::shared_ptr<mars::topology::edge::MarsEdge>
mars::topology::container::EntityParser::parseEdge(XmlRpc::XmlRpcValue& pEdge)
{
  try
  {
    const std::string lID = getParam<std::string>(pEdge, ENTITY_ID_PARAM_NAME);
    const std::string lDescription = getParam<std::string>(pEdge, ENTITY_ID_DESCRIPTION_PARAM_NAME);
    const std::string lFrameId = getParam<std::string>(pEdge, ENTITY_FRAME_ID_PARAM_NAME);
    const int lType = getParam<int>(pEdge, ENTITY_TYPE_PARAM_NAME);
    const bool lisLocked = getParam<bool>(pEdge, ENTITY_IS_LOCKED_PARAM_NAME);
    const float lCoordinateX = getParam<float>(pEdge, ENTITY_X_COORDINATE_PARAM_NAME);
    const float lCoordinateY = getParam<float>(pEdge, ENTITY_Y_COORDINATE_PARAM_NAME);
    const std::vector<float> lFootprintX =
        getParamVector<float>(pEdge, ENTITY_FOOTPRINT_X_PARAM_NAME);
    const std::vector<float> lFootprintY =
        getParamVector<float>(pEdge, ENTITY_FOOTPRINT_Y_PARAM_NAME);
    const std::string lOriginId = getParam<std::string>(pEdge, EDGE_ORIGIN_ID_PARAM_NAME);
    const std::string lOriginContainer =
        getParam<std::string>(pEdge, EDGE_ORIGIN_CONTAINER_ID_PARAM_NAME);
    const std::string lTargetId = getParam<std::string>(pEdge, EDGE_DESTINATION_ID_PARAM_NAME);
    const std::string lTargetContainer =
        getParam<std::string>(pEdge, EDGE_DESTINATION_CONTAINER_ID_PARAM_NAME);
    const int lDirection = getParam<int>(pEdge, EDGE_DIRECTION_PARAM_NAME);
    const float lEdgeLength = getParam<float>(pEdge, EDGE_LENGTH_PARAM_NAME);

    const mars::topology::common::TopologyEntityRestrictions lRestrictions(
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_LINEAR_VELOCITY_PARAM_NAME),
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_ANGULAR_VELOCITY_PARAM_NAME),
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_LINEAR_ACCELERATION_PARAM_NAME),
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_ANGULAR_ACCELERATION_PARAM_NAME),
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_WEIGHT_PARAM_NAME),
        this->getParam<double>(pEdge, ENTITY_MAXIMUM_HEIGHT_PARAM_NAME), {}, {});

    std::shared_ptr<mars::topology::edge::MarsEdge> lEdge =
        std::make_shared<mars::topology::edge::MarsEdge>(
            lID, lDescription, lType, lisLocked, lCoordinateX, lCoordinateY, lFootprintX,
            lFootprintY, lFrameId, lOriginId, lOriginContainer, lTargetId, lTargetContainer,
            lDirection, lEdgeLength, lRestrictions);
    return lEdge;
  }
  catch (mars::common::exception::ReadParamException e)
  {
    MARS_LOG_ERROR(EXEPTION_MSG_PARSING_ERROR << e.what());
  }
  catch (XmlRpc::XmlRpcException e)
  {
    MARS_LOG_ERROR(e.getMessage() << "(" << e.getCode() << ")");
  }
}
