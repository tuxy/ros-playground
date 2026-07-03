#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "warehouse_pkg/action/robot_task.hpp"
#include "warehouse_pkg/srv/get_bay_number.hpp"

class NavigatorActionServer : public rclcpp::Node {
  using RobotTask = warehouse_pkg::action::RobotTask;
  using RobotTaskGoalHandle = rclcpp_action::ServerGoalHandle<RobotTask>;

public:
  NavigatorActionServer(
      const std::string &serial, uint32_t bay_number,
      const geometry_msgs::msg::Pose2D &initial_pose,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robot_navigator_server", options), serial_(serial),
        bay_number_(bay_number), initial_pose_(initial_pose) {
    auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
                              std::shared_ptr<const RobotTask::Goal> goal) {
      RCLCPP_INFO(
          this->get_logger(), "Received goal request (%s) with goal {%lf, %lf}",
          rclcpp_action::to_string(uuid).c_str(), goal->goal.x, goal->goal.y);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](const std::shared_ptr<RobotTaskGoalHandle> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accept =
        [this](const std::shared_ptr<RobotTaskGoalHandle> goal_handle) {
          auto execute_in_thread = [this, goal_handle]() {
            return this->execute(goal_handle);
          };
          std::thread{execute_in_thread}.detach();
        };

    this->action_server_ = rclcpp_action::create_server<RobotTask>(
        this, "robot_navigator", handle_goal, handle_cancel, handle_accept);
  }

private:
  rclcpp_action::Server<RobotTask>::SharedPtr action_server_;
  std::string serial_;
  uint32_t bay_number_;
  geometry_msgs::msg::Pose2D initial_pose_;
  void execute(const std::shared_ptr<RobotTaskGoalHandle> goal_handle) {
    auto feedback = std::make_shared<RobotTask::Feedback>();
    auto result = std::make_shared<RobotTask::Result>();

    // TODO implement the actual movement, feedback and result

    (void)this->serial_;
    (void)this->bay_number_;
    (void)this->initial_pose_;
    (void)goal_handle;
  };
};

static geometry_msgs::msg::Pose2D pose_from_bay(uint32_t bay_number) {
  geometry_msgs::msg::Pose2D p;
  p.x = static_cast<double>(bay_number) * 2.0;
  p.y = 0;
  return p;
}

static uint32_t lookup_bay(const std::string &serial) {
  auto tmp = std::make_shared<rclcpp::Node>("bay_lookup");
  auto request = std::make_shared<warehouse_pkg::srv::GetBayNumber::Request>();
  request->serial_number = serial;
  auto client = tmp->create_client<warehouse_pkg::srv::GetBayNumber>(
      "/return_bay_number");

  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(tmp->get_logger(),
                 "GetBayNumber not available; defaulting bay=0");
    return 0;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(tmp, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(tmp->get_logger(), "Failed to call service; defaulting bay=0");
    return 0;
  }

  auto result = future.get();
  RCLCPP_INFO(tmp->get_logger(), "Got bay number: %u", result->bay_number);

  return result->bay_number;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("navigator_serial");
  node->declare_parameter("serial", "NONE");
  std::string serial = node->get_parameter("serial").as_string();

  uint32_t bay = lookup_bay(serial);

  auto navigator =
      std::make_shared<NavigatorActionServer>(serial, bay, pose_from_bay(bay));
  rclcpp::spin(navigator);
  rclcpp::shutdown();
  return 0;
}
