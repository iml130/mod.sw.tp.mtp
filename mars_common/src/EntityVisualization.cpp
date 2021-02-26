#include "mars_common/EntityVisualization.h"

static const std::string FRAME_ID_NAME = "/mars";
static const std::string ENTITY_NAMESPACE = "mars_entity";
int mars::common::EntityVisualization::sIdCounter = 0;

mars::common::EntityVisualization::EntityVisualization()
{
}

mars::common::EntityVisualization::EntityVisualization(const EntityVisualization& other)
{
}

mars::common::EntityVisualization& mars::common::EntityVisualization::getEntityVisualizationInstance()
{
  static EntityVisualization instance;
  sIdCounter++;
  return instance;
}

void mars::common::EntityVisualization::drawEntity(geometry_msgs::PointStamped coordinate, ros::Publisher markerPub)
{
  visualization_msgs::Marker entityMarker;
  entityMarker.header.frame_id = FRAME_ID_NAME;
  entityMarker.header.stamp = ros::Time::now();
  entityMarker.ns = ENTITY_NAMESPACE;
  entityMarker.id = sIdCounter;
  entityMarker.type = visualization_msgs::Marker::SPHERE;
  entityMarker.action = visualization_msgs::Marker::ADD;
  entityMarker.pose.position.x = coordinate.point.x;
  entityMarker.pose.position.y = coordinate.point.y;
  entityMarker.pose.position.z = coordinate.point.z;
  entityMarker.scale.x = 1.0;
  entityMarker.scale.y = 1.0;
  entityMarker.scale.z = 1.0;
  entityMarker.color.b = 1.0f;
  entityMarker.color.a = 1.0;
  entityMarker.lifetime = ros::Duration();
  markerPub.publish(entityMarker);
}

void mars::common::EntityVisualization::drawEntity(geometry_msgs::PolygonStamped footprint, ros::Publisher markerPub)
{
  visualization_msgs::Marker entityMarker;
  entityMarker.header.frame_id = FRAME_ID_NAME;
  entityMarker.header.stamp = ros::Time::now();
  entityMarker.ns = ENTITY_NAMESPACE;
  entityMarker.id = sIdCounter;
  entityMarker.type = visualization_msgs::Marker::LINE_STRIP;
  entityMarker.action = visualization_msgs::Marker::ADD;
  for (auto& i : footprint.polygon.points)
  {
    // Convert Point32 to Point
    geometry_msgs::Point p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    entityMarker.points.push_back(p);
  }
  auto i = footprint.polygon.points.begin();
  geometry_msgs::Point p;
  p.x = i->x;
  p.y = i->y;
  p.z = i->z;
  entityMarker.points.push_back(p);
  entityMarker.scale.x = 0.1;
  entityMarker.color.b = 1.0f;
  entityMarker.color.a = 1.0;
  entityMarker.lifetime = ros::Duration();
  markerPub.publish(entityMarker);
}
