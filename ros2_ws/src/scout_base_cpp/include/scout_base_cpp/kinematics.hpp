#ifndef SCOUT_BASE_CPP__KINEMATICS_HPP_
#define SCOUT_BASE_CPP__KINEMATICS_HPP_

#include <utility>

namespace scout_base_cpp
{
struct DriveParams
{
  double track_width{0.0};
  double max_wheel_speed{0.0};
  int pwm_min{0};
};

std::pair<int, int> twist_to_pwm(double v, double w, const DriveParams & p);
int pan_rad_to_servo_deg(double rad, int pan_sign, double pan_min, double pan_max);
double servo_deg_to_pan_rad(int deg, int pan_sign);

}

#endif
