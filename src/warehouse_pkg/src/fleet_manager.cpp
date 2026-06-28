#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "warehouse_pkg/srv/get_bay_number.hpp"

#define TABLE_SIZE 200

uint32_t fnv1a_32(std::string_view str) {
  constexpr uint32_t FNV_PRIME = 0x01000193;
  constexpr uint32_t FNV_OFFSET_BASIS = 0x811C9DC5;

  uint32_t hash = FNV_OFFSET_BASIS;
  for (char c : str) {
    hash ^= static_cast<uint8_t>(c);
    hash *= FNV_PRIME;
  }
  return hash % TABLE_SIZE;
}

void return_bay_number(
    const std::shared_ptr<warehouse_pkg::srv::GetBayNumber::Request> request,
    std::shared_ptr<warehouse_pkg::srv::GetBayNumber::Response> response) {
  uint32_t number = fnv1a_32(request->serial_number);
  response->bay_number = number;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request from robot %s",
              request->serial_number.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Returning bay number %d", number);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("fleet_manager_server");
  rclcpp::Service<warehouse_pkg::srv::GetBayNumber>::SharedPtr service =
      node->create_service<warehouse_pkg::srv::GetBayNumber>(
          "return_bay_number", return_bay_number);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
