#include "scout_base_cpp/kinematics.hpp"
#include "scout_base_cpp/conversions.hpp"
#include <algorithm>
#include <cmath>

namespace scout_base_cpp
{

    namespace {
        int side_pwm(double v, const DriveParams & p){
            if (std::abs(v) < 0.005) { return 0.0; }
            int mag = p.pwm_min + (255 - p.pwm_min) * std::abs(v) / p.max_wheel_speed;
            int pwm = static_cast<int>(std::lround(mag));
            return std::clamp((v > 0 ? pwm: -pwm), -255, 255);
        }
    }

    std::pair<int, int> twist_to_pwm(double v, double w, const DriveParams &p)
    {
        double v_left = v - w * p.track_width / 2.0;
        double v_right = v + w * p.track_width / 2.0;
        double biggest = std::max(std::abs(v_left), std::abs(v_right));
        if (biggest > p.max_wheel_speed) {
            double s = biggest / p.max_wheel_speed;
            v_left /= s;
            v_right /= s;
        }

        return {side_pwm(v_left, p), side_pwm(v_right, p)};
    }

    int pan_rad_to_servo_deg(double rad, int pan_sign, double pan_min, double pan_max)
    {
        rad = std::clamp(rad, pan_min, pan_max);
        const auto deg = std::lround(90.0 + pan_sign * rad * 180.0/ kPi);
        return std::clamp(static_cast<int>(deg), 0, 180);
    }

    double servo_deg_to_pan_rad(int deg, int pan_sign)
    {
        return pan_sign * (deg - 90) * kPi / 180.0;
    }
}