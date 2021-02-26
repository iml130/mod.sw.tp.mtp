# Profile is continuous, represented by discrete samples of constant sampling period (NYI)
uint8 CONTIUOUS = 10

# Profile is piecewise linear, represented by values at the discontinuities
uint8 PIECEWISE = 20

# No time profile information
uint8 NONE = 0

uint8 type
duration sampling_period

time[] time
geometry_msgs/Twist[] velocity