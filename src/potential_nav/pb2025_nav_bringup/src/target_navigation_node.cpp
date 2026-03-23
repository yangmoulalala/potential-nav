#include "target_navigation_node.hpp"

TargetNavigationNode::TargetNavigationNode(const rclcpp::NodeOptions &options)
    : Node("target_navigation_node", options)
{
  // 初始化参数
  target_received_ = false;
  
  // 创建客户端用于清理代价地图
  local_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "local_costmap/clear_entirely_local_costmap");
  global_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "global_costmap/clear_entirely_global_costmap");
  
  
  // 创建Action客户端
  navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");
  
  // 等待Action服务器可用
  if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }
  
  // 创建订阅者
  nav_c_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/mavlink/nav/c", 1,
      std::bind(&TargetNavigationNode::nav_c_Callback, this, std::placeholders::_1));
  
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10,
      std::bind(&TargetNavigationNode::odometryCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&TargetNavigationNode::cmd_vel_Callback, this, std::placeholders::_1));

  // 发布者
  nav_pc_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mavlink/nav/pc", 1);
  
  // 创建定时器，用于定期检查是否需要发送新的目标点
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 100ms = 10Hz
      std::bind(&TargetNavigationNode::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Target navigation node initialized");
}

void TargetNavigationNode::nav_c_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) { 
    RCLCPP_WARN(this->get_logger(), "nav_c message too short, need at least 3 elements");
    return;
  }
  std::lock_guard<std::mutex> lock(target_mutex_);
  last_is_restart_ = is_restart_;
  is_restart_ = static_cast<bool>(msg.get()->data[0]);
  target_position_.x = msg.get()->data[1];
  target_position_.y = msg.get()->data[2];
  target_received_ = true;
}


void TargetNavigationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  current_odometry_ = *msg;
}

void TargetNavigationNode::cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std_msgs::msg::Float32MultiArray nav_pc_msg;
  nav_pc_msg.data.resize(5);
  nav_pc_msg.data[0] = msg.get()->linear.x;
  nav_pc_msg.data[1] = msg.get()->linear.y;
  nav_pc_msg.data[2] = current_odometry_.pose.pose.position.x;
  nav_pc_msg.data[3] = current_odometry_.pose.pose.position.y;

  double roll, pitch, yaw;
  tf2::Quaternion q(
      current_odometry_.pose.pose.orientation.x,
      current_odometry_.pose.pose.orientation.y,
      current_odometry_.pose.pose.orientation.z,
      current_odometry_.pose.pose.orientation.w
  );
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  nav_pc_msg.data[4] = yaw;

  nav_pc_pub_->publish(nav_pc_msg);

}

void TargetNavigationNode::timerCallback()
{
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!target_received_) return;

    // 计算新目标与上次发送目标的欧氏距离
    double dist = std::hypot(target_position_.x - last_sent_target_.x, 
                             target_position_.y - last_sent_target_.y);

    // 发送条件：有新目标 且 (距离变化显著 或 当前没在导航)
    if (dist > TARGET_DIST_THRESHOLD || !navigation_in_progress_) {
        sendNavigationGoal();
        last_sent_target_ = target_position_;
    }

    if (!last_is_restart_ && is_restart_){
      RestartContainer();
    }
}

void TargetNavigationNode::sendNavigationGoal()
{
    // 如果当前有任务在跑，可以考虑是否取消或直接让新 Goal 抢占
    // Nav2 默认支持抢占，所以我们直接发送
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position = target_position_;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](auto handle) {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            navigation_in_progress_ = false;
        } else {
            this->current_goal_handle_ = handle;
            navigation_in_progress_ = true;
        }
    };

    send_goal_options.result_callback = std::bind(&TargetNavigationNode::resultCallback, this, std::placeholders::_1);

    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void TargetNavigationNode::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
  }
}

void TargetNavigationNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    navigation_in_progress_ = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation reached!");
            retry_count_ = 0;
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Navigation aborted! Potential obstacle. Clearing costmaps...");
            clearCostmaps(); // 失败后自动清理代价地图
            sendNavigationGoal();
            if (retry_count_ < MAX_RETRIES) {
                retry_count_++;
                RCLCPP_INFO(this->get_logger(), "Retrying goal... (%d/%d)", retry_count_, MAX_RETRIES);
            }
            break;

        case rclcpp_action::ResultCode::CANCELED:
            retry_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            break;

        default:
            break;
    }
}

void TargetNavigationNode::RestartContainer()
{
  RCLCPP_WARN(this->get_logger(), "Restarting container...");
  exit(1);
}
void TargetNavigationNode::clearCostmaps()
{
  RCLCPP_INFO(this->get_logger(), "Sending requests to clear costmaps...");

  // 1. 清理局部代价地图
  auto local_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  
  // 使用带有 Lambda 回调的异步发送
  local_costmap_clear_client_->async_send_request(
    local_request,
    [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
      auto response = future.get();
      if (response) {
        RCLCPP_INFO(this->get_logger(), "Local costmap cleared successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear local costmap");
      }
    }
  );

  // 2. 清理全局代价地图
  auto global_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  
  global_costmap_clear_client_->async_send_request(
    global_request,
    [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future) {
      auto response = future.get();
      if (response) {
        RCLCPP_INFO(this->get_logger(), "Global costmap cleared successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear global costmap");
      }
    }
  );
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}