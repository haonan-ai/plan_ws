#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class GlobalPathNode : public rclcpp::Node
{
public:
  GlobalPathNode()
  : Node("global_path_node")
  {
    RCLCPP_INFO(this->get_logger(), "global path node started");
    
    // Subscribe to PoseArray topic
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task_manager/pose_array", 10, std::bind(&GlobalPathNode::pose_array_callback, this, _1));
    // Subscribe to PoseStamped topic
    pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task_manager/pose", 10, std::bind(&GlobalPathNode::pose_stamped_callback, this, _1));
    // Create action client for compute_path_through_poses
    action_client_poses_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(
      this, "compute_path_through_poses");
    // Create action client for compute_path_through_pose
    action_client_pose_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      this, "compute_path_to_pose");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /task_manager/PoseArray and /task_manager/pose, will call compute_path_through_poses or compute_path_to_pose action and publish to /global_path");
  }

private:
  void pose_stamped_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received PoseStamped, frame_id: '%s'", msg->header.frame_id.c_str());
    // Create action goal for compute_path_to_pose
    auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    goal_msg.goal = *msg;
    // 强制设置 frame_id
    if (goal_msg.goal.header.frame_id.empty()) {
      goal_msg.goal.header.frame_id = "map";
      RCLCPP_WARN(this->get_logger(), "Input PoseStamped frame_id is empty, set to 'map' by default");
    }
    goal_msg.use_start = false;  // Use current robot pose as start
    goal_msg.planner_id = "GridBased";
    // Send action goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GlobalPathNode::action_result_callback_pose, this, _1);
    send_goal_options.goal_response_callback = std::bind(&GlobalPathNode::action_goal_response_callback_pose, this, _1);
    send_goal_options.feedback_callback = std::bind(&GlobalPathNode::action_feedback_callback_pose, this, _1, _2);
    RCLCPP_INFO(this->get_logger(), "Sending compute_path_to_pose action goal");
    action_client_pose_->async_send_goal(goal_msg, send_goal_options);
  }
  
  // For PoseArray (multiple goals)
  void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PoseArray, ignoring");
      return;
    }
    // Convert PoseArray to vector of PoseStamped for the action
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    for (const auto& pose : msg->poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = msg->header;
      pose_stamped.pose = pose;
      goals.push_back(pose_stamped);
    }
    // Create action goal
    auto goal_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();
    goal_msg.goals = goals;
    goal_msg.use_start = false;  // Use current robot pose as start
    goal_msg.planner_id = "GridBased";  // Use GridBased planner
    // Send action goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GlobalPathNode::action_result_callback_poses, this, _1);
    send_goal_options.goal_response_callback = std::bind(&GlobalPathNode::action_goal_response_callback_poses, this, _1);
    send_goal_options.feedback_callback = std::bind(&GlobalPathNode::action_feedback_callback_poses, this, _1, _2);
    RCLCPP_INFO(this->get_logger(), "Sending compute_path_through_poses action goal");
    action_client_poses_->async_send_goal(goal_msg, send_goal_options);
  }
  
  void action_goal_response_callback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathThroughPoses goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "ComputePathThroughPoses goal accepted by server");
      current_goal_handle_poses_ = goal_handle;
    }
  }
  
  void action_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action succeeded! Path computed successfully");
        
        // Publish the computed path to /global_path topic
        if (result.result->path.poses.size() > 0) {
          // global_path_pub_->publish(result.result->path); // This line is removed
          RCLCPP_INFO(this->get_logger(), "Published path with %zu poses to /global_path", 
                     result.result->path.poses.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "Computed path is empty");
        }
        break;
        
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action was aborted");
        break;
        
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action was canceled");
        break;
        
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    
    // 清除当前goal handle
    current_goal_handle_poses_.reset();
  }
  
  void action_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::ComputePathThroughPoses::Feedback> feedback)
  {
    (void)feedback;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Action feedback received");
  }
  
  // For compute_path_to_pose action
  void action_goal_response_callback_pose(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathToPose goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "ComputePathToPose goal accepted by server");
      current_goal_handle_pose_ = goal_handle;
    }
  }
  void action_result_callback_pose(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action succeeded! Path computed successfully (to pose)");
        if (result.result->path.poses.size() > 0) {
          RCLCPP_INFO(this->get_logger(), "Published path with %zu poses to /global_path", result.result->path.poses.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "Computed path is empty (to pose)");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action was aborted (to pose)");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action was canceled (to pose)");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code (to pose)");
        break;
    }
    current_goal_handle_pose_.reset();
  }
  void action_feedback_callback_pose(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback> feedback)
  {
    (void)feedback;
    RCLCPP_INFO(this->get_logger(), "Action feedback received (to pose)");
  }
  // For compute_path_through_poses action
  void action_goal_response_callback_poses(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathThroughPoses goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "ComputePathThroughPoses goal accepted by server");
      current_goal_handle_poses_ = goal_handle;
    }
  }
  void action_result_callback_poses(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action succeeded! Path computed successfully (through poses)");
        if (result.result->path.poses.size() > 0) {
          RCLCPP_INFO(this->get_logger(), "Published path with %zu poses to /global_path", result.result->path.poses.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "Computed path is empty (through poses)");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Action was aborted (through poses)");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action was canceled (through poses)");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code (through poses)");
        break;
    }
    current_goal_handle_poses_.reset();
  }
  void action_feedback_callback_poses(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::ComputePathThroughPoses::Feedback> feedback)
  {
    (void)feedback;
    RCLCPP_INFO(this->get_logger(), "Action feedback received (through poses)");
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr action_client_poses_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_pose_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr current_goal_handle_poses_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr current_goal_handle_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPathNode>());
  rclcpp::shutdown();
  return 0;
}
