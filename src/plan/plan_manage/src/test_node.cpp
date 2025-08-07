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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/occ_grid_values.hpp"
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

    // Create publisher for OccupancyGrid
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/perception/occupancy_grid_map/local_map", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      std::bind(&TestNode::broadcast_transform, this));
    
    // 声明参数
    this->declare_parameter<double>("map_width", 4);        // 地图边长，单位米
    this->declare_parameter<double>("map_resolution", 0.05);    // 分辨率，单位米/像素
    this->get_parameter("map_width", map_width_);
    this->get_parameter("map_resolution", map_resolution_);
    
    // 初始化机器人位姿
    robot_x_ = 3.78;
    robot_y_ = 3.22;
    robot_theta_ = 0.0;
    
    // 初始化速度命令
    current_linear_x_ = 0.0;
    current_angular_z_ = 0.0;
    
    // 初始化固定的占用栅格地图
    initialize_occupancy_grid();
    
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
    pose1.position.x = 4.85;
    pose1.position.y = 8.62;
    pose1.position.z = 0.0;
    pose1.orientation.w = 1.0;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = -2.51;
    pose2.position.y = 10.96;
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
    
    // 发布OccupancyGrid
    publish_occupancy_grid();
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
  
  void initialize_occupancy_grid()
  {
    // 设置消息头
    fixed_occupancy_grid_.header.frame_id = "base_link";
    
    // 计算像素宽高
    int pixel_width = static_cast<int>(map_width_ / map_resolution_);
    fixed_occupancy_grid_.info.resolution = map_resolution_;
    fixed_occupancy_grid_.info.width = pixel_width;
    fixed_occupancy_grid_.info.height = pixel_width;
    
    // 设置地图原点（左下角，以base_link坐标系为参考，机器人在地图中心）
    fixed_occupancy_grid_.info.origin.position.x = -map_width_ / 2.0;
    fixed_occupancy_grid_.info.origin.position.y = -map_width_ / 2.0;
    fixed_occupancy_grid_.info.origin.position.z = 0.0;
    fixed_occupancy_grid_.info.origin.orientation.w = 1.0;
    fixed_occupancy_grid_.info.origin.orientation.x = 0.0;
    fixed_occupancy_grid_.info.origin.orientation.y = 0.0;
    fixed_occupancy_grid_.info.origin.orientation.z = 0.0;
    
    // 初始化地图数据
    fixed_occupancy_grid_.data.resize(pixel_width * pixel_width, nav2_util::OCC_GRID_FREE);
    
    // 计算三个区域的边界
    int lower_bound = pixel_width / 3;      // 下1/3边界
    int upper_bound = 2 * pixel_width / 3;  // 上1/3边界
    
    // 设置下1/3区域为障碍物
    for (int y = 0; y < lower_bound; ++y) {
      for (int x = 0; x < pixel_width; ++x) {
        fixed_occupancy_grid_.data[y * pixel_width + x] = nav2_util::OCC_GRID_OCCUPIED;
      }
    }
    
    // 设置中间1/3区域为地面（自由空间）
    for (int y = lower_bound; y < upper_bound; ++y) {
      for (int x = 0; x < pixel_width; ++x) {
        fixed_occupancy_grid_.data[y * pixel_width + x] = nav2_util::OCC_GRID_FREE;
      }
    }
    
    // 设置上1/3区域为未知区域
    for (int y = upper_bound; y < pixel_width; ++y) {
      for (int x = 0; x < pixel_width; ++x) {
        fixed_occupancy_grid_.data[y * pixel_width + x] = nav2_util::OCC_GRID_UNKNOWN;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "固定占用栅格地图初始化完成，边长: %.2f m, 分辨率: %.2f m/像素, 像素: %d", map_width_, map_resolution_, pixel_width);
    RCLCPP_INFO(this->get_logger(), "地图布局：下1/3障碍物，中间1/3地面，上1/3未知区域");
  }
  
  void publish_occupancy_grid()
  {
    // 更新时间戳
    fixed_occupancy_grid_.header.stamp = this->now();
    // 在base_link坐标系中，地图原点固定，机器人始终在地图中心
    // 不需要更新原点位置，因为地图是相对于base_link坐标系的
    occupancy_grid_pub_->publish(fixed_occupancy_grid_);
  }
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  
  // 机器人位姿状态
  double robot_x_;
  double robot_y_;
  double robot_theta_;
  
  // 当前速度命令
  double current_linear_x_;
  double current_angular_z_;
  
  // 固定的占用栅格地图数据
  nav_msgs::msg::OccupancyGrid fixed_occupancy_grid_;
  double map_width_;       // 地图边长，单位米
  double map_resolution_;  // 地图分辨率，单位米/像素
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}

