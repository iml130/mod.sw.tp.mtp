#ifndef MARS_COMMON_ENTITYVISUALIZATION_H
#define MARS_COMMON_ENTITYVISUALIZATION_H

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>

namespace mars
{
namespace common
{
class EntityVisualization
{
public:
  /**
   * @brief getEntityVisualizationInstance: The preferred method to access the
   * singleton.
   * @return Returns the reference of the EntityVisualization object.
   */
  static EntityVisualization& getEntityVisualizationInstance();
  /**
   * @brief publishEntity: Publishes an entity to the rviz topic.
   * @param coordinate: The coordinates of the entity.
   * @param markerPub: The marker publisher of the entity.
   */
  void drawEntity(geometry_msgs::PointStamped coordinate, ros::Publisher markerPub);
  /**
   * @brief publishEntityPublishes an entity to the rviz topic.
   * @param footprint The footprint of the entity.
   * @param markerPub The marker publisher of the entity.
   */
  void drawEntity(geometry_msgs::PolygonStamped footprint, ros::Publisher markerPub);

protected:
  /**
   * @brief EntityVisualization: Instantiation via the standard constructor is
   * not possible.
   */
  EntityVisualization();
  /**
   * @brief EntityVisualization Copy of an EntityVisualization object using the
   * copy constructor is not allowed because it is protected.
   * @param other: An object of EntityVisualization to copy.
   */
  EntityVisualization(const EntityVisualization& other);

private:
  static int sIdCounter;
};
}  // namespace common
}  // namespace mars

#endif // MARS_COMMON_ENTITYVISUALIZATION_H
