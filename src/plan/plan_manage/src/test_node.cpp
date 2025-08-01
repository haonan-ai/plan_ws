#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/header.hpp"

class Test : public rclcpp::Node
{
public:
  Test() : Node("test_node")
  {
    RCLCPP_INFO(this->get_logger(), "test_node started.");
    
    // Subscribe to clicked_point topic
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&Test::clicked_point_callback, this, std::placeholders::_1));
    
    // Create publisher for PoseArray
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/task_manager/PoseArray", 10);
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to /clicked_point, will publish to /task_manager/PoseArray");
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
    pose1.position.x = 4.8;
    pose1.position.y = 1.5;
    pose1.position.z = 0.0;
    pose1.orientation.w = 1.0;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 1.8;
    pose2.position.y = 3.0;
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
    RCLCPP_INFO(this->get_logger(), "Published PoseArray with %zu poses", pose_array_msg.poses.size());
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}
