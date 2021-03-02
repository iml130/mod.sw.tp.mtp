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


#ifndef MARS_ROUTING_BASE_NODE_H
#define MARS_ROUTING_BASE_NODE_H

// ros node internal includes
#include "mars_common/Id.h"
#include "mars_routing_core/Route.h"
#include "mars_routing_core/Router.h"

// msg / srv includes
#include "mars_routing_srvs/GetRoute.h"
#include "mars_routing_srvs/RemoveRouteVisualization.h"
#include "mars_routing_srvs/SetRouteVisualizationStartTime.h"

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// C++ includes
#include <mutex>
#include <string>
#include <thread>

namespace mars
{
namespace routing
{
namespace base
{
class ROSNode
{
public:
  /**
   * @brief Node Creates an object of Node.
   */
  ROSNode();

  /**
   * @brief Node Deletes an object Node object.
   */
  ~ROSNode();

  /**
   * @brief run Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool run(void);

  // First member variables, second methods.
protected:
  // First member variables, second methods.
private:
  // ros node handles
  /**
   * @brief mNH Public ros handle.
   */
  ros::NodeHandle mNH;
  /**
   * @brief mNHPriv Private node handle.
   */
  ros::NodeHandle mNHPriv;

  /**
   * @brief mHz Contains the information about the set loop rate of the node.
   */
  int mHz;

  /**
   * @brief mNodeLogLevel Current log level of the node.
   */
  std::string mNodeLogLevel;

  /**
   * @brief mPluginName Current name of the Plugin, which should be loaded
   */
  std::string mPluginName;

  /**
   * @brief mServiceGetRoute Server for the getRoute Service
   */
  ros::ServiceServer mServiceGetRoute;

  /**
   * @brief mServiceRemoveVisualizedRoutes Server for offering the removal of a route's
   * visualization
   */
  ros::ServiceServer mServiceRemoveVisualizedRoutes;

  /**
   * @brief Service for setting start time of route visualization
   */
  ros::ServiceServer mServiceSetVisualizationStartTime;

  /**
   * @brief Publisher for rviz markers
   */
  ros::Publisher mVisualizationPub;

  /**
   * @brief Routes that have been routed and may be visualized
   */
  std::vector<mars::routing::core::IterationRoute*> mRoutes;

  /**
   * @brief Thread taking care of processing visualization and publishing marker messages
   */
  std::shared_ptr<std::thread> mVisualizationThread;

  /**
   * @brief Mutex to guard routes container so it will no have a route added to it while being
   * visualized
   */
  std::mutex mVisualizationMutex;

  /**
   * @brief Flag to indicate whether routes shall be visualized. Corresponding parameter read into
   * here
   */
  bool mRunVisualization;

  /**
   * @brief Start time of visualization. Visualization will interpolate all routes between mVisualizationStartTime and the latest time among all routes.
   */
  ros::Time mVisualizationStartTime;

  // Member variables for plugin loading
  /**
   * @brief mRouterLoader Class Loader for the router plugin
   */
  pluginlib::ClassLoader<mars::routing::core::Router> mRouterLoader;

  /**
   * @brief mRouterAlg Pointer to the dynamic loaded plugin
   */
  boost::shared_ptr<mars::routing::core::Router> mRouterAlg;

  // init methods for ros node

  /**
   * @brief init Inits the node. This method automatically calls the methods:
   * initServices(), initPublisher(), initSubsriber(), readLaunchParams() and
   * printNodeInfos() if no error occours during execution.
   *
   * @return Returns true if no error occours.
   */
  bool init(void);

  /**
   * @brief initServices Initializes the provided services.
   * @return Returns true if no error occours.
   */
  bool initServices(void);

  /**
   * @brief initPublisher Initializes the provided publishers.
   * @return Returns true if no error occours.
   */
  bool initPublisher(void);

  /**
   * @brief initSubsriber Initializes the subscribers.
   * @return Returns true if no error occours.
   */
  bool initSubscriber(void);

  /**
   * @brief initActions Initializes the provided actions.
   * @return Returns true if no error occours.
   */
  bool initActions(void);

  /**
   * @brief readLaunchParams Reads the paramters from the launch file.
   * @return Returns true if no error occours.
   */
  bool readLaunchParams(void);

  /**
   * @brief rosMainLoop Calls rosSpinOnce() with the rate you set with 'node_rate'.
   */
  void rosMainLoop(void) const;

  /**
   * @brief setNodeLogLevel Set the log level of the ros console for the node.
   * @return Return true if log level was successfully set.
   */
  bool setNodeLogLevel(void) const;

  /**
   * @brief loadRouterPlugin Loads a plugin
   * @return True if loading was successful
   */
  bool loadRouterPlugin();

  bool getRoute(mars_routing_srvs::GetRoute::Request& req,
                mars_routing_srvs::GetRoute::Response& res);

  bool removeRouteVisualization(mars_routing_srvs::RemoveRouteVisualization::Request& req,
                                mars_routing_srvs::RemoveRouteVisualization::Response& res);

  bool setRouteVisualizationStartTime(mars_routing_srvs::SetRouteVisualizationStartTime::Request& req,
                                 mars_routing_srvs::SetRouteVisualizationStartTime::Response& res);
  /**
   * Runs a loop that keeps publishing visualization messages. To be run in a separate thread
   */
  void visualizationLoop();
};
} // namespace base
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_BASE_NODE_H
