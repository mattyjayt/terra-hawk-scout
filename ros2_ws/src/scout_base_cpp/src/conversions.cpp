#include "scout_base_cpp/conversions.hpp"

namespace scout_base_cpp
{
    double g_to_mps(double g)
    {
        return g * kG;
    }

    double dps_to_radps(double w_dps)
    {
        return w_dps * kPi / 180.0;
    }

    StaleDetector::StaleDetector(int limit)
    {
        limit_ = limit;
    }

    bool StaleDetector::update(const std::array<double, 6> &values)
    {
        if (last_.has_value() && *last_ == values){
            ++counter_;
        } else {
            last_ = values;
            counter_ = 1;
        }
        return counter_ >= limit_;
    }

    RateMeter::RateMeter(double window_s)
    {
        window_s_ = window_s;
    }

    void RateMeter::tick(double now)
    {
        times_.push_back(now);
    }

    double RateMeter::rate(double now)
    {
        double cutoff = now - window_s_;
        while (!times_.empty() && times_.front() < cutoff){
            times_.pop_front();
        }
        return static_cast<double>(times_.size()) / window_s_;
    }

    int link_level(bool connected, double rate_hz, double expected_hz, bool imu_frozen, double s_since_line, std::optional<double> battery_v, double battery_present_v)
    {
        std::optional<double> battery_missing;

        if (!connected || s_since_line > 0.5 || imu_frozen == true) return 2;
        if (battery_v.has_value() && *battery_v <= battery_present_v) { battery_missing = battery_v; }
        if (rate_hz < expected_hz * 0.9 || battery_missing) return 1;
        return 0;
        
    }
}