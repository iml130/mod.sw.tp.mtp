#include "mars_topology_container/ContainerVisualization.h"

static const std::string ENTITY_NAMESPACE = "mars_entity";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS = "Entity Infos";
static const std::string INTERACTIVE_MARKER_MENU__NAME_CONTAINER = "Container Infos";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS_CONTAINER_ID = "Print Container ID";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS_CONTAINER_NAME = "Print Container Name";
static const std::string INTERACTIVE_MARKER_MENU_Node = "Node";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_ID = "Print Entity ID";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_NAME = "Print Entity Name";
static const std::string INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_RESERVATIONS = "Print Entity reservations";

static const std::string CONTAINER_NAME_TEXT_INVALID = "Container name not set!";
static const std::string CONTAINER_ID_TEXT_INVALID = "Container Id not set!";
static const std::string ENTITY_NAME_TEXT_INVALID = "Name not set!";
static const std::string ENTITY_ID_TEXT_INVALID = "ID not set!";

mars::topology::container::ContainerVisualization::ContainerVisualization()
    : mEngine(std::random_device{}())
{
  this->mContainerName = CONTAINER_NAME_TEXT_INVALID;
  this->mContainerId = CONTAINER_ID_TEXT_INVALID;
  this->mInteractiveMarkerServer.reset(
      new interactive_markers::InteractiveMarkerServer("interactive_markers", "", false));

  this->mContainerIdSet = false;
  this->mMarkerMenuInitialized = false;
}

mars::topology::container::ContainerVisualization::ContainerVisualization(
    const mars::common::Id& pContainerId)
    : mEngine(std::random_device{}())
{
  this->mContainerName = pContainerId.getDescription();
  this->mContainerId = pContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);
  this->mInteractiveMarkerServer.reset(
      new interactive_markers::InteractiveMarkerServer("interactive_markers", "", false));

  this->mContainerIdSet = true;
  this->mMarkerMenuInitialized = false;

  this->initFootprintMenu();
}

void mars::topology::container::ContainerVisualization::setContainerId(
    const mars::common::Id& pContainerId)
{
  if (!this->mContainerIdSet)
  {
    this->mContainerName = pContainerId.getDescription();
    this->mContainerId = pContainerId.getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);

    this->mContainerIdSet = true;

    this->initFootprintMenu();
  }
}

void mars::topology::container::ContainerVisualization::callbackInteractiveMarkerContainerId(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_WARN_STREAM("Container Id: " << this->mContainerId);
}
void mars::topology::container::ContainerVisualization::callbackInteractiveMarkerContainerName(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_WARN_STREAM("Container Name: " << this->mContainerName);
}
void mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeId(

    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  int lIndex = atoi(feedback->marker_name.c_str());
  std::shared_ptr<mars::topology::common::TopologyEntity> lTmpEntity =
      this->mContainerEntities[lIndex];
  std::string lContainerId =
      lTmpEntity->getId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT);

  ROS_WARN_STREAM("Entity Id: " << lContainerId);
}
void mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeName(

    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  int lIndex = atoi(feedback->marker_name.c_str());
  std::shared_ptr<mars::topology::common::TopologyEntity> lTmpEntity =
      this->mContainerEntities[lIndex];
  std::string lContainerDescription = lTmpEntity->getId().getDescription();

  ROS_WARN_STREAM("Entity name: " << lContainerDescription);
}
void mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeReservations(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  int lIndex = atoi(feedback->marker_name.c_str());
  std::shared_ptr<mars::topology::common::TopologyEntity> lTmpEntity =
      this->mContainerEntities[lIndex];

  std::vector<mars::topology::common::TopologyEntityReservation*> lEntityReservations = lTmpEntity->getSortedReservations();

  for (auto reservation : lEntityReservations)
  {
    ROS_WARN_STREAM("Entity reservation: " << reservation->getAgentId().getUUIDAsString(mars::common::Id::UUIDFormat::HEXDEC_SPLIT) << "(" << reservation->getTimeInterval().getStartTime() << " - " << reservation->getTimeInterval().getEndTime() << ")" << "\n");
  }
}

void mars::topology::container::ContainerVisualization::drawDirection(
    const mars::common::Id& pContainerId,
    const std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::edge::MarsEdge>>
        pEdgesContainer,
    const std::unordered_map<mars::common::Id,
                             std::shared_ptr<mars::topology::common::TopologyEntity>>
        pContainer,
    const std::string& markerFrame, const std::string nodeNamespace)
{

  if (this->mContainerIdSet)
  {
    for (auto& it : pEdgesContainer)
    {
      std::vector<visualization_msgs::Marker> lDirections;

      // check whether the target and the origin entities at the same container
      geometry_msgs::PointStamped lTargetEntity;
      geometry_msgs::PointStamped lOriginEntity;
      std::pair<bool, bool> lServiceCalls = std::make_pair(true, true);

      if (it.second->getTargetContainer() == pContainerId)
      {
        lTargetEntity = pContainer.find(it.second->getTargetId())->second->getCoordinate();
        lServiceCalls.first = false;
      }
      if (it.second->getOriginContainer() == pContainerId)
      {
        lOriginEntity = pContainer.find(it.second->getOriginId())->second->getCoordinate();
        lServiceCalls.second = false;
      }

      lDirections = this->mEntityVisualization.createAndAddMarkerArrow(
          it.second->getCoordinate(), markerFrame, it.first, nodeNamespace,
          it.second->getOriginId(), it.second->getOriginContainer(), lOriginEntity,
          it.second->getTargetId(), it.second->getTargetContainer(), lTargetEntity,
          it.second->getDirection(), lServiceCalls);

      for (auto& markerIt : lDirections)
      {
        this->addInteractiveMarker(markerIt, it.second, markerFrame);
      }
    }
  }
  else
  {
    MARS_LOG_ERROR("Container ID was not set, marker can't be published");
  }
}

void mars::topology::container::ContainerVisualization::drawVertex(
    const std::unordered_map<mars::common::Id, std::shared_ptr<mars::topology::vertex::MarsVertex>>
        pVerticesContainer,
    const std::string& markerFrame)
{
  if (this->mContainerIdSet)
  {
    for (auto& it : pVerticesContainer)
    {
      visualization_msgs::Marker lMarker = this->mEntityVisualization.createMarkerPoint(
          it.second->getCoordinate(), markerFrame, ENTITY_NAMESPACE);
      this->addInteractiveMarker(lMarker, it.second, markerFrame);
    }
  }
  else
  {
    MARS_LOG_ERROR("Container ID was not set, marker can't be published");
  }
}

void mars::topology::container::ContainerVisualization::drawFootprint(
    const std::unordered_map<mars::common::Id,
                             std::shared_ptr<mars::topology::common::TopologyEntity>>
        pContainer,
    const ros::Publisher& markerPub, const std::string& markerFrame)
{
  visualization_msgs::MarkerArray lContainerMarker;

  for (auto& it : pContainer)
  {
    if (it.second->getFootprint().polygon.points.empty())
    {
      ROS_DEBUG_STREAM_ONCE("Footprint of Entity with ID: " << std::to_string(it.second->getId())
                                                            << "is not set/empty!");
    }
    else
    {
      lContainerMarker.markers.push_back(this->mEntityVisualization.createMarkerLineStrip(
          it.second->getFootprint(), markerFrame, ENTITY_NAMESPACE));
    }
  }

  markerPub.publish(lContainerMarker);
}

void mars::topology::container::ContainerVisualization::addInteractiveMarker(
    const visualization_msgs::Marker& pMarker,
    std::shared_ptr<mars::topology::common::TopologyEntity> pTopologyEntity,
    const std::string& pMarkerFrame)
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  std::stringstream lTmpMarkerName;

  interactiveMarker = this->makeEmptyMarker(pMarkerFrame);

  lTmpMarkerName << this->mContainerEntities.size();
  interactiveMarker.name = lTmpMarkerName.str();
  this->mContainerEntities.push_back(pTopologyEntity);

  this->makeMenuMarker(interactiveMarker, pMarker);

  this->mInteractiveMarkerServer->insert(interactiveMarker);
  this->mMenuHandler.apply(*(this->mInteractiveMarkerServer), interactiveMarker.name);
  this->mInteractiveMarkerServer->applyChanges();
}

void mars::topology::container::ContainerVisualization::initFootprintMenu()
{
  interactive_markers::MenuHandler::EntryHandle lEntryHandleTmp;

  interactive_markers::MenuHandler::EntryHandle lEntryHandleContainerInfos =
      this->mMenuHandler.insert(INTERACTIVE_MARKER_MENU__NAME_CONTAINER);
  interactive_markers::MenuHandler::EntryHandle lEntryHandleInfos =
      this->mMenuHandler.insert(INTERACTIVE_MARKER_MENU_NAME_INFOS);

  if (this->mContainerIdSet)
  {
    // Add container Id
    lEntryHandleTmp = this->mMenuHandler.insert(lEntryHandleContainerInfos,
                                                INTERACTIVE_MARKER_MENU_NAME_INFOS_CONTAINER_ID);
    lEntryHandleTmp =
        this->mMenuHandler.insert(lEntryHandleTmp, this->mContainerId,
                                  boost::bind(&mars::topology::container::ContainerVisualization::
                                                  callbackInteractiveMarkerContainerId,
                                              this, _1));

    // Add container name
    lEntryHandleTmp = this->mMenuHandler.insert(lEntryHandleContainerInfos,
                                                INTERACTIVE_MARKER_MENU_NAME_INFOS_CONTAINER_NAME);
    lEntryHandleTmp =
        this->mMenuHandler.insert(lEntryHandleTmp, this->mContainerName,
                                  boost::bind(&mars::topology::container::ContainerVisualization::
                                                  callbackInteractiveMarkerContainerName,
                                              this, _1));

    // Add entity Id menu
    lEntryHandleTmp = this->mMenuHandler.insert(
        lEntryHandleInfos, INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_ID,
        boost::bind(
            &mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeId,
            this, _1));

    // Add entity name menu
    lEntryHandleTmp = this->mMenuHandler.insert(
        lEntryHandleInfos, INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_NAME,
        boost::bind(
            &mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeName,
            this, _1));

    // Add entity reservation menu
    lEntryHandleTmp = this->mMenuHandler.insert(
        lEntryHandleInfos, INTERACTIVE_MARKER_MENU_NAME_INFOS_ENTITY_RESERVATIONS,
        boost::bind(
            &mars::topology::container::ContainerVisualization::callbackInteractiveMarkerNodeReservations,
            this, _1));

    this->mMarkerMenuInitialized = true;
  }
  else
  {
    MARS_LOG_ERROR("Container ID was not set, marken menu can't be created");
  }
}

visualization_msgs::InteractiveMarker
mars::topology::container::ContainerVisualization::makeEmptyMarker(const std::string& markerFrame)
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = markerFrame;
  interactiveMarker.scale = 1;
  interactiveMarker.pose.position.x = 0;
  interactiveMarker.pose.position.y = 0;

  return interactiveMarker;
}

void mars::topology::container::ContainerVisualization::makeMenuMarker(
    visualization_msgs::InteractiveMarker& interactiveMarker,
    const visualization_msgs::Marker& entityMarker)
{
  visualization_msgs::InteractiveMarkerControl lInteractiveMarkerControl;
  lInteractiveMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  lInteractiveMarkerControl.always_visible = true;
  lInteractiveMarkerControl.markers.push_back(entityMarker);
  interactiveMarker.controls.push_back(lInteractiveMarkerControl);
}

