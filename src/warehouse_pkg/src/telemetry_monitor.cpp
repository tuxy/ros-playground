#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TelemetrySubscriber : public rclcpp::Node {
public:
  TelemetrySubscriber() : Node("telemetry") {

    this->declare_parameter<std::vector<std::string>>("robots", {});
    std::vector<std::string> namespaces =
        this->get_parameter("robots").as_string_array();

    if (namespaces.empty()) {
      RCLCPP_WARN(this->get_logger(), "No robots provided to logger");
    }

    for (std::string ns : namespaces) {
      RCLCPP_INFO(this->get_logger(), "Spawning subscriber to %s", ns.c_str());

      auto sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
          ns, 10, [this, ns](geometry_msgs::msg::Pose2D msg) -> void {
            RCLCPP_INFO(this->get_logger(), "[%s] currently at {%lf, %lf}",
                        ns.c_str(), msg.x, msg.y);
          });
      subs_.push_back(sub);
    }
  };

private:
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr>
      subs_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetrySubscriber>());
  rclcpp::shutdown();
  return 0;
}
