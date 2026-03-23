#ifndef TARGET_NAVIGATION_NODE_HPP_
#define TARGET_NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TargetNavigationNode : public rclcpp::Node
{
public:
  explicit TargetNavigationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void nav_c_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  void cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  
  void sendNavigationGoal();
  
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);
  
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);
  
  void RestartContainer();
  void clearCostmaps();

  // 订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr nav_c_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // 发布者
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr nav_pc_pub_;

  // Action客户端
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
  
  // 服务客户端
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_clear_client_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 数据成员
  geometry_msgs::msg::Point target_position_;
  nav_msgs::msg::Odometry current_odometry_;
  bool is_restart_;
  bool target_received_;
  bool last_is_restart_;
  
  // 互斥锁
  std::mutex target_mutex_;
  std::mutex odometry_mutex_;


  // 追踪当前 Action 状态
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
  geometry_msgs::msg::Point last_sent_target_;
  bool navigation_in_progress_ = false;

  // 阈值参数
  const double TARGET_DIST_THRESHOLD = 0.15; // 目标点变化超过15cm才重发
  int retry_count_ = 0;
  const int MAX_RETRIES = 3;
};

#endif  // TARGET_NAVIGATION_NODE_HPP_