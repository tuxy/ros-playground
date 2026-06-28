#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "warehouse_pkg/msg/id_pose2_d.hpp"

using namespace std::chrono_literals;

class TelemetrySubscriber : public rclcpp::Node {
public:
  TelemetrySubscriber() : Node("telemetry") {
    auto topic_callback = [this](warehouse_pkg::msg::IDPose2D message) -> void {
      RCLCPP_INFO(this->get_logger(), "Robot %ld is currently at {%lf, %lf}",
                  message.id, message.pose.x, message.pose.y);
    };
    subscription_ = this->create_subscription<warehouse_pkg::msg::IDPose2D>(
        "robot_pose", 10, topic_callback);
  };

private:
  rclcpp::Subscription<warehouse_pkg::msg::IDPose2D>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetrySubscriber>());
  rclcpp::shutdown();
  return 0;
}
