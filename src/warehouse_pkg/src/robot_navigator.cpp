#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "warehouse_pkg/msg/id_pose2_d.hpp"
#include "warehouse_pkg/srv/get_bay_number.hpp"

using namespace std::chrono_literals;

class NavigatorPublisher : public rclcpp::Node {
public:
  NavigatorPublisher(int bay_number)
      : Node("navigator_publisher"), count_(0), bay_number_(bay_number) {

    this->declare_parameter("serial", "NONE");
    std::string serial = this->get_parameter("serial").as_string();

    publisher_ =
        this->create_publisher<warehouse_pkg::msg::IDPose2D>("robot_pose", 10);
    auto timer_callback = [this]() -> void {
      auto message = warehouse_pkg::msg::IDPose2D();
      message.id = bay_number_;
      message.pose.x = 0.0f;
      message.pose.y = count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing pose {%lf %lf} with id %ld",
                  message.pose.x, message.pose.y, message.id);
      this->publisher_->publish(message);
    };
    RCLCPP_INFO(this->get_logger(),
                "Robot serial number: %s; Robot bay number: %d", serial.c_str(),
                this->bay_number_);
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<warehouse_pkg::msg::IDPose2D>::SharedPtr publisher_;
  size_t count_;
  int bay_number_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto serial_node = std::make_shared<rclcpp::Node>("navigator_serial");
  serial_node->declare_parameter("serial", "NONE");
  rclcpp::Client<warehouse_pkg::srv::GetBayNumber>::SharedPtr client =
      serial_node->create_client<warehouse_pkg::srv::GetBayNumber>(
          "/return_bay_number");

  std::string serial = serial_node->get_parameter("serial").as_string();
  auto request = std::make_shared<warehouse_pkg::srv::GetBayNumber::Request>();
  request->serial_number = serial;

  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(serial_node->get_logger(),
                 "GetBayNumber not available; exiting");
    return 1;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(serial_node, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(serial_node->get_logger(), "Failed to call service");
    return 1;
  }

  auto result = future.get();
  RCLCPP_INFO(serial_node->get_logger(), "Got bay number: %u",
              result->bay_number);

  auto navigator = std::make_shared<NavigatorPublisher>(result->bay_number);
  rclcpp::spin(navigator);
  rclcpp::shutdown();
  return 0;
}
