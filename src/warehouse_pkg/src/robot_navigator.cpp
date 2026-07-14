#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "warehouse_pkg/action/robot_task.hpp"
#include "warehouse_pkg/msg/robot_status_monitor.hpp"
#include "warehouse_pkg/srv/get_bay_number.hpp"

const float SPEED = 2;

static geometry_msgs::msg::Pose2D pose_from_bay(uint32_t bay_number);
static uint32_t lookup_bay(const std::string &serial);

class NavigatorActionServer : public rclcpp::Node {
  using RobotTask = warehouse_pkg::action::RobotTask;
  using RobotTaskGoalHandle = rclcpp_action::ServerGoalHandle<RobotTask>;

public:
  NavigatorActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robot_navigator_server", options) {

    this->declare_parameter("serial", "NONE");
    serial_ = this->get_parameter("serial").as_string();
    initial_pose_ = pose_from_bay(lookup_bay(serial_));
    current_pose_ = initial_pose_;

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

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    this->action_server_ = rclcpp_action::create_server<RobotTask>(
        this, "robot_navigator", handle_goal, handle_cancel, handle_accept);
  }
  // The current_pose_ will handle the current starting point, which will change
  // when the action is stopped. the initial_pose_ is just the starting point
  // (where the bay is located)
private:
  rclcpp_action::Server<RobotTask>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string serial_;
  geometry_msgs::msg::Pose2D initial_pose_;
  geometry_msgs::msg::Pose2D current_pose_;

  void execute(const std::shared_ptr<RobotTaskGoalHandle> goal_handle) {
    rclcpp::Rate loop_rate(1);
    geometry_msgs::msg::Pose2D target = goal_handle->get_goal()->goal;

    while (rclcpp::ok()) {
      auto feedback = std::make_shared<RobotTask::Feedback>();
      auto result = std::make_shared<RobotTask::Result>();

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->status =
            warehouse_pkg::msg::RobotStatusMonitor::STATUS_TERMINATED;
        goal_handle->canceled(result);
        return;
      }

      float dx = target.x - this->current_pose_.x;
      float dy = target.y - this->current_pose_.y;
      float rot = std::atan2(dy, dx);

      float distance = std::sqrt(dx * dx + dy * dy);

      if (distance > SPEED) {
        this->current_pose_.x += (dx / distance) * SPEED;
        this->current_pose_.y += (dy / distance) * SPEED;

        RCLCPP_INFO(this->get_logger(), "Moving: currently at {%lf, %lf}",
                    current_pose_.x, current_pose_.y);
      } else {
        this->current_pose_.x = target.x;
        this->current_pose_.y = target.y;

        result->success = true;
        result->status = warehouse_pkg::msg::RobotStatusMonitor::STATUS_READY;
        result->final = current_pose_;
        goal_handle->succeed(result);
        return;
      }

      broadcast_transform(this->current_pose_, rot);
      feedback->current_heading = rot;
      feedback->current_pos = this->current_pose_;
      feedback->distance_left = distance;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }
  }

  void broadcast_transform(geometry_msgs::msg::Pose2D pos, float rot) {
    geometry_msgs::msg::TransformStamped t;
    std::string ns = this->get_namespace();

    t.header.stamp = this->now();
    t.header.frame_id = "world";
    t.child_frame_id = ns + "/base_link";

    // Set 2D Translations
    t.transform.translation.x = pos.x;
    t.transform.translation.y = pos.y;
    t.transform.translation.z = 0.0; // Flat on the warehouse floor

    // 6. Convert the flat Yaw angle into a 3D Quaternion orientation for TF2
    tf2::Quaternion q;
    q.setRPY(0, 0, rot);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }
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
  rclcpp::spin(std::make_shared<NavigatorActionServer>());
  rclcpp::shutdown();
  return 0;
}
