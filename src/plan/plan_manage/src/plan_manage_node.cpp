#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;
using namespace std::placeholders;

class PlanManageNode : public rclcpp::Node
{
public:
  PlanManageNode()
  : Node("plan_manage_node"), goal_in_progress_(false)
  {
    RCLCPP_INFO(this->get_logger(), "plan manage node started");

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 10, std::bind(&PlanManageNode::global_path_callback, this, _1));
    
    // 创建FollowPath action客户端
    follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
        this, "follow_path");
    
    RCLCPP_INFO(this->get_logger(), "Created FollowPath action client");
  }

private:
  nav_msgs::msg::Path::SharedPtr pending_path_;
  std::mutex path_mutex_;
  void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received global_path");
    
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty global_path, ignoring");
      return;
    }
    
    // 如果已经有目标在执行中，忽略新的路径
    if (goal_in_progress_.load()) {
      RCLCPP_WARN(this->get_logger(), "Goal already in progress, canceling and updating to new path");
      pending_path_ = msg;
      if (current_goal_handle_) {
        auto future = follow_path_client_->async_cancel_goal(current_goal_handle_);
        // Optionally, add a callback to handle cancel result
      }
      return;
    }
    
    // 发送FollowPath action目标
    send_follow_path_goal(*msg);
  }
  
  void send_follow_path_goal(const nav_msgs::msg::Path& path)
  {
    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available");
      goal_in_progress_ = false;  // 重置标志，因为发送失败
      return;
    }
    
    auto goal_msg = nav2_msgs::action::FollowPath::Goal();
    goal_msg.path = path;
    goal_msg.controller_id = "FollowPath";  // 使用默认控制器
    
    RCLCPP_INFO(this->get_logger(), "Sending FollowPath action goal");
    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&PlanManageNode::follow_path_result_callback, this, _1);
    send_goal_options.goal_response_callback = std::bind(&PlanManageNode::follow_path_goal_response_callback, this, _1);
    
    follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  void follow_path_goal_response_callback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath goal was rejected by server");
      goal_in_progress_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "FollowPath goal accepted by server");
      current_goal_handle_ = goal_handle;
      goal_in_progress_ = true;
    }
  }
  
  void follow_path_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "FollowPath action succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "FollowPath action was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "FollowPath action was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "FollowPath action failed");
        break;
    }
    
    // 清除当前goal handle和状态
    current_goal_handle_.reset();
    goal_in_progress_ = false;
    // 检查是否有待处理的新路径
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (pending_path_) {
      RCLCPP_INFO(this->get_logger(), "Sending pending global_path after cancel");
      send_follow_path_goal(*pending_path_);
      pending_path_.reset();
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr current_goal_handle_;
  std::atomic<bool> goal_in_progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanManageNode>());
  rclcpp::shutdown();
  return 0;
}