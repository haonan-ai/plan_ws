#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#define _USE_MATH_DEFINES

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    RCLCPP_INFO(this->get_logger(), "test_node started.");
    
    // Subscribe to clicked_point topic
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&TestNode::clicked_point_callback, this, std::placeholders::_1));
    
    // Subscribe to cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/plan/cmd_vel", 10, std::bind(&TestNode::cmd_vel_callback, this, std::placeholders::_1));
    
    // Create publisher for PoseArray
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/task_manager/pose_array", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      std::bind(&TestNode::broadcast_transform, this));
    
    // 初始化机器人位姿
    robot_x_ = 0.3;
    robot_y_ = 0.3;
    robot_theta_ = 0.0;
    
    // 初始化速度命令
    current_linear_x_ = 0.0;
    current_angular_z_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to /clicked_point and /plan/cmd_vel, will publish to /task_manager/PoseArray");
  }

private:
  void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received clicked point: x=%.2f, y=%.2f, z=%.2f", 
                msg->point.x, msg->point.y, msg->point.z);
    
    // Create PoseArray message
    auto pose_array_msg = geometry_msgs::msg::PoseArray();
    
    // Set header
    pose_array_msg.header.frame_id = "map";
    pose_array_msg.header.stamp = this->now();
    
    // Create poses based on the example provided
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 5.0;
    pose1.position.y = 4.3;
    pose1.position.z = 0.0;
    pose1.orientation.w = 1.0;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 9.7;
    pose2.position.y = 9.7;
    pose2.position.z = 0.0;
    pose2.orientation.w = 1.0;
    pose2.orientation.x = 0.0;
    pose2.orientation.y = 0.0;
    pose2.orientation.z = 0.0;
    
    // Add poses to the array
    pose_array_msg.poses.push_back(pose1);
    pose_array_msg.poses.push_back(pose2);
    
    // Publish the PoseArray
    pose_array_pub_->publish(pose_array_msg);
    RCLCPP_INFO(this->get_logger(), "发布PoseArray，包含 %zu 个目标点", pose_array_msg.poses.size());
    RCLCPP_INFO(this->get_logger(), "目标点1: (%.1f, %.1f), 目标点2: (%.1f, %.1f)", 
                pose1.position.x, pose1.position.y, pose2.position.x, pose2.position.y);
  }
  
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 保存最新的速度命令
    current_linear_x_ = msg->linear.x;
    current_angular_z_ = msg->angular.z;
    
    RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: linear.x=%.2f, angular.z=%.2f", 
                 current_linear_x_, current_angular_z_);
  }
  
  void broadcast_transform()
  {
    // 根据速度更新机器人位姿
    update_robot_pose();
    
    geometry_msgs::msg::TransformStamped transform;
    
    // 设置时间戳
    transform.header.stamp = this->now();
    
    // 设置坐标系
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    
    // 设置平移
    transform.transform.translation.x = robot_x_;
    transform.transform.translation.y = robot_y_;
    transform.transform.translation.z = 0.0;
    
    // 设置旋转 (从欧拉角转换为四元数)
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(robot_theta_ / 2.0);
    transform.transform.rotation.w = std::cos(robot_theta_ / 2.0);
    
    // 发布TF变换
    tf_broadcaster_->sendTransform(transform);
  }
  
  void update_robot_pose()
  {
    // 时间步长 (100ms = 0.1s)
    const double dt = 0.1;
    
    // 根据线速度和角速度更新位姿
    // 使用简单的运动学模型
    if (std::abs(current_angular_z_) > 1e-6) {
      // 有角速度时，机器人做圆弧运动
      double radius = current_linear_x_ / current_angular_z_;
      robot_theta_ += current_angular_z_ * dt;
      robot_x_ += radius * (std::sin(robot_theta_) - std::sin(robot_theta_ - current_angular_z_ * dt));
      robot_y_ += radius * (std::cos(robot_theta_ - current_angular_z_ * dt) - std::cos(robot_theta_));
    } else {
      // 无角速度时，机器人直线运动
      robot_x_ += current_linear_x_ * std::cos(robot_theta_) * dt;
      robot_y_ += current_linear_x_ * std::sin(robot_theta_) * dt;
    }
    
    // 保持角度在 [-π, π] 范围内
    while (robot_theta_ > M_PI) robot_theta_ -= 2 * M_PI;
    while (robot_theta_ < -M_PI) robot_theta_ += 2 * M_PI;
  }
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  
  // 机器人位姿状态
  double robot_x_;
  double robot_y_;
  double robot_theta_;
  
  // 当前速度命令
  double current_linear_x_;
  double current_angular_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
