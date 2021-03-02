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

#ifndef MARS_COMMON_COLOR_H
#define MARS_COMMON_COLOR_H

#include <cmath>
#include <std_msgs/ColorRGBA.h>
/**
 * HSV - RGB conversion from
 * https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
 *
 */
namespace mars
{

namespace common
{

struct RGB
{
  double r; // a fraction between 0 and 1
  double g; // a fraction between 0 and 1
  double b; // a fraction between 0 and 1
};

struct HSV
{
  double h; // angle in degrees
  double s; // a fraction between 0 and 1
  double v; // a fraction between 0 and 1
};

HSV rgb2hsv(RGB in);
RGB hsv2rgb(HSV in);

std_msgs::ColorRGBA hsv2msg(HSV pHSV, double pAlpha = 1.0);

} // namespace common
} // namespace mars

#endif