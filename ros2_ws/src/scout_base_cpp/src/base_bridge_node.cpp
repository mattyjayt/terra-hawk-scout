#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("base_bridge");
    RCLCPP_INFO(node->get_logger(), "scout_base_cpp builds and runs");
    rclcpp::shutdown();
    return 0;
}