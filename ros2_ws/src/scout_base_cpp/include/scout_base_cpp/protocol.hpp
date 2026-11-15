#ifndef SCOUT_BASE_CPP__PROTOCOL_HPP_
#define SCOUT_BASE_CPP__PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace scout_base_cpp
{
    struct ImuSample
    {
        std::int64_t time_ms{0};
        double accel_x{0.0};
        double accel_y{0.0};
        double accel_z{0.0};
        double gyro_x{0.0};
        double gyro_y{0.0};
        double gyro_z{0.0};
        double yaw_deg{0.0};
        int servo_deg{0};
        double batt_voltage{0.0};
    };

    struct Ack
    {
        std::string text;
    };

    struct WatchdogStop
    {

    };

    using Line = std::variant<ImuSample, Ack, WatchdogStop>;

    std::optional<Line> parse_line(std::string_view line);
    std::string format_drive(int left, int right);
    std::string format_servo(int servo_deg);
}

#endif