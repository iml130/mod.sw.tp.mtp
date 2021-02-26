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