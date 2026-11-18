#include "scout_base_cpp/protocol.hpp"
#include <algorithm>
#include <vector>
#include <charconv>

namespace scout_base_cpp
{
namespace
{

std::string_view trim(std::string_view s)
{
  if (s.find_first_not_of(" \t\r\n") == std::string_view::npos) {
    return {};
  }
  int first = s.find_first_not_of(" \t\r\n");
  int last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::vector<std::string_view> split(std::string_view s)
{
  std::vector<std::string_view> parts;
  std::size_t i{0};

  while (i < s.size()) {
    while (i < s.size() && s[i] == ' ') {++i;}
    if (i == s.size()) {break;}
    const std::size_t start = i;
    while (i < s.size() && s[i] != ' ') {++i;}
    parts.push_back(s.substr(start, i - start));
  }

  return parts;
}

template<typename T> bool to_number(std::string_view s, T & out)
{
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
  return ec == std::errc() && ptr == s.data() + s.size();
}
}

std::optional<Line> parse_line(std::string_view line)
{
  line = trim(line);

        // Case ImuSample: If line starts with "I "
  if (line.rfind("I ", 0) == 0) {
    std::vector<std::string_view> parts = split(line);
    if (parts.size() != 11) {return std::nullopt;}
    ImuSample s;
    if(!to_number(parts[1], s.time_ms) || !to_number(parts[2], s.accel_x) || !to_number(parts[3],
        s.accel_y) || !to_number(parts[4], s.accel_z) || !to_number(parts[5],
        s.gyro_x) || !to_number(parts[6], s.gyro_y) || !to_number(parts[7],
        s.gyro_z) || !to_number(parts[8], s.yaw_deg) || !to_number(parts[9],
        s.servo_deg) || !to_number(parts[10], s.batt_voltage))
    {
      return std::nullopt;
    }
    return s;
  }
        // Case Ack: If line starts with "A "
  else if (line.rfind("A ", 0) == 0) {return Ack{std::string(line.substr(2))};}
        // Case Watchdog: If line starts with "W stop"
  else if (line == "W stop") {return WatchdogStop{};}
        // Case Unknown Option Received
  else {return std::nullopt;}
}

std::string format_drive(int left, int right)
{
  left = std::clamp(left, -255, 255);
  right = std::clamp(right, -255, 255);
  return "M " + std::to_string(left) + " " + std::to_string(right) + "\n";
}

std::string format_servo(int servo_deg)
{
  servo_deg = std::clamp(servo_deg, 0, 180);
  return "V " + std::to_string(servo_deg) + "\n";
}
}
