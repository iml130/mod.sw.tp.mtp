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


#ifndef MARS_YELLOW_PAGES_ROS_NODE_H
#define MARS_YELLOW_PAGES_ROS_NODE_H

// ros node internal includes
#include "mars_yellow_pages/MARSYellowPages.h"

// ros includes
#include <mars_common/exception/AdvertiseServiceException.h>
#include <mars_common/exception/ReadParamException.h>
#include <mars_topology_msgs/TopologyEntityRegistration.h>
#include <mars_topology_srvs/GetTopologyEntityContainerId.h>
#include <ros/ros.h>

// C++ includes
#include <string>

/**
 * This is a ros node template class. Please use this template for all ros c++ projects. Copy the
 * content to your new node and pay attention to all todo comments!
 *
 */
class ROSNode
{
public:
  /**
   * @brief MainClassTemplate Creates an object of MainClassTemplate.
   */
  ROSNode();

  /**
   * @brief MainClassTemplate Deletes an object MainClassTemplate object.
   */
  ~ROSNode();

  /**
   * @brief runROSNode Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool runROSNode(void);

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

  MARSYellowPages mMARSYellowPages;

  /**
   * @brief mNodeLogLevel Current log level of the node.
   */
  std::string mNodeLogLevel;

  ros::Subscriber mSubscriberRegisterContainer;

  ros::ServiceServer mServiceLookupContainer;

  void subscriberCallbackRegisterContainer(
      const mars_topology_msgs::TopologyEntityRegistration::ConstPtr& msg);

  bool
  serviceCallbackLookupContainer(mars_topology_srvs::GetTopologyEntityContainerId::Request& req,
                                mars_topology_srvs::GetTopologyEntityContainerId::Response& res);

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
   * @brief advertiseService This method is called from the initServices()
   * method. It encapsulates the advertising of the transferred service.
   * @return Returns true if the service was advertised successfully.
   * @throw If the service could not be andertised, an
   * AdvertiseServiceException is thrown.
   */
  template <class T, class MReq, class MRes>
  bool advertiseService(bool (T::*srvFunc)(MReq&, MRes&), ros::ServiceServer& serviceServer,
                        std::string serviceName, T* obj);

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
   * @brief getParam Reads the named paramter form the parameter server.
   * If the parameter could not be found a default value is assigned.
   */
  template <typename T>
  T getParam(ros::NodeHandle& nH, const std::string& paramName, const T& defaultValue);

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   */
  template <typename T> void getParam(ros::NodeHandle& nH, const std::string& paramName, T& param);

  /**
   * @brief rosMainLoop Calls rosSpinOnce() with the rate you set with 'node_rate'.
   */
  void rosMainLoop(void) const;

  /**
   * @brief setNodeLogLevel Sets the log level of the ros console for the node.
   * @return Return true if log level was successfully set.
   */
  bool setNodeLogLevel(void) const;
};

#endif // MARS_YELLOW_PAGES_ROS_NODE_H
