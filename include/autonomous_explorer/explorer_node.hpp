#ifndef AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_
#define AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <memory>

namespace autonomous_explorer
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ExplorerNode : public rclcpp::Node
{
public:
    ExplorerNode();

private:

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void check_nav2_state();

    void process_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void send_goal(geometry_msgs::msg::Point p, double yaw);

 
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_client_;
    
    //tf2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    
    nav_msgs::msg::OccupancyGrid::SharedPtr first_map_;
    rclcpp::TimerBase::SharedPtr check_nav2_timer_;
    bool is_working_;
    bool nav2_active_;

    // Exploration Memory
    std::vector<geometry_msgs::msg::Point> blacklist_;
    geometry_msgs::msg::Point current_goal_;
};

}  // namespace autonomous_explorer

#endif  // AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_