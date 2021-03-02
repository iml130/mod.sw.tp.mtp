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

#ifndef MARS_COMMON_LOGGER_HPP
#define MARS_COMMON_LOGGER_HPP

#include <boost/current_function.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <typeinfo>

#include <ros/ros.h>

#ifndef NDEBUG
#define DEBUG true
#else
#define DEBUG false
#endif

#define MARS_LOG(pStream, ROSLogFunction)                                                          \
  {                                                                                                \
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();                  \
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);                                 \
    ROSLogFunction("[" << std::put_time(std::localtime(&now_c), "%T") << "] ["                     \
                       << ros::this_node::getName() << ": " << BOOST_CURRENT_FUNCTION << "] "      \
                       << pStream);                                                                \
  }

#define MARS_LOG_INFO(pStream) MARS_LOG(pStream, ROS_INFO_STREAM)
#define MARS_LOG_WARN(pStream) MARS_LOG(pStream, ROS_WARN_STREAM)
#define MARS_LOG_ERROR(pStream) MARS_LOG(pStream, ROS_ERROR_STREAM)
#define MARS_LOG_DEBUG(pStream) MARS_LOG(pStream, ROS_DEBUG_STREAM)
#define MARS_LOG_FATAL(pStream) MARS_LOG(pStream, ROS_FATAL_STREAM)

#define MARS_LOG_TRACE() MARS_LOG("", ROS_INFO_STREAM)

/* PREDEFINED MESSAGES */
#define MARS_LOG_WARN_UNKNOWN_TOPOLOGY_ENTITY_TYPE() MARS_LOG_WARN("Unknown topology entity type!")

#define MARS_LOG_ERROR_GET_ROS_PARAMETER(pParameter)                                               \
  MARS_LOG_ERROR("Could not find value for ROS parameter '" << pParameter                          \
                                                            << "' on parameter server!")

#endif /* MARS_COMMON_LOGGER_HPP */