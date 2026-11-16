#ifndef SCOUT_BASE_CPP_CONVERSION_HPP_
#define SCOUT_BASE_CPP_CONVERSION_HPP_

#include <array>
#include <deque>
#include <optional>

namespace scout_base_cpp
{
    constexpr double kG = 9.80665;
    constexpr double kPi = 3.14159265358979323846;

    double g_to_mps(double g);
    double dps_to_radps(double w_dps);

    class StaleDetector
    {
        public:
            explicit StaleDetector(int limit=10);
            bool update(const std::array<double, 6> & values);

        private:
            int limit_;
            int counter_{0};
            std::optional<std::array<double, 6>> last_;
    };

    class RateMeter
    {
        public:
            explicit RateMeter(double window_s=1.0);
            void tick(double now);
            double rate(double now);

        private:
            double window_s_;
            std::deque<double> times_;
    };

    int link_level(bool connected, double rate_hz, double expected_hz, bool imu_frozen, double s_since_line, std::optional<double> battery_v, double battery_present_v);
}

#endif