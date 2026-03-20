#include "../include/autonomous_explorer/explorer_node.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <queue>
#include <map>

namespace autonomous_explorer
{

struct Frontier
{
    uint32_t size;
    geometry_msgs::msg::Point centroid;
};

using PointInt = std::pair<int, int>;

static int grid_index(const nav_msgs::msg::OccupancyGrid &map, const PointInt &p)
{
    return p.second * map.info.width + p.first;
}

static int8_t cell_value(const nav_msgs::msg::OccupancyGrid &map, const PointInt &p)
{
    return map.data[grid_index(map, p)];
}

static bool in_bounds(const nav_msgs::msg::OccupancyGrid &map, const PointInt &p)
{
    return p.first >= 0 && p.first < static_cast<int>(map.info.width) &&
           p.second >= 0 && p.second < static_cast<int>(map.info.height);
}

static bool is_frontier_cell(const nav_msgs::msg::OccupancyGrid &map, const PointInt &p,
                             const std::map<PointInt, bool> &visited)
{
    if (!in_bounds(map, p) || cell_value(map, p) != -1) return false;
    if (visited.find(p) != visited.end()) return false;

    const int dx4[4] = {-1, 1, 0, 0};
    const int dy4[4] = {0, 0, -1, 1};
    for (int i = 0; i < 4; ++i) {
        PointInt n = {p.first + dx4[i], p.second + dy4[i]};
        if (in_bounds(map, n) && cell_value(map, n) == 0)
            return true;
    }
    return false;
}

static Frontier build_frontier(const nav_msgs::msg::OccupancyGrid &map,
                               PointInt seed,
                               std::map<PointInt, bool> &visited)
{
    const int dx8[8] = {-1,-1,-1,0,0,1,1,1};
    const int dy8[8] = {-1,0,1,-1,1,-1,0,1};

    std::queue<PointInt> q;
    q.push(seed);
    visited[seed] = true;

    uint32_t size = 0;
    double sum_x = 0.0, sum_y = 0.0;

    while (!q.empty()) {
        PointInt cur = q.front();
        q.pop();
        size++;
        sum_x += cur.first;
        sum_y += cur.second;

        for (int i = 0; i < 8; ++i) {
            PointInt nb = {cur.first + dx8[i], cur.second + dy8[i]};
            if (is_frontier_cell(map, nb, visited)) {
                visited[nb] = true;
                q.push(nb);
            }
        }
    }

    Frontier f;
    f.size = size;
    f.centroid.x = (sum_x / size) * map.info.resolution + map.info.origin.position.x;
    f.centroid.y = (sum_y / size) * map.info.resolution + map.info.origin.position.y;
    f.centroid.z = 0.0;
    return f;
}

static std::vector<Frontier> find_frontiers(const nav_msgs::msg::OccupancyGrid &map,
                                             const PointInt &robot_cell)
{
    std::map<PointInt, bool> visited;
    std::queue<PointInt> q;
    q.push(robot_cell);
    visited[robot_cell] = true;

    std::vector<Frontier> frontiers;

    while (!q.empty()) {
        PointInt cur = q.front();
        q.pop();

        const int dx4[4] = {-1,1,0,0};
        const int dy4[4] = {0,0,-1,1};
        for (int i = 0; i < 4; ++i) {
            PointInt nb = {cur.first + dx4[i], cur.second + dy4[i]};
            if (!in_bounds(map, nb)) continue;

            int8_t val = cell_value(map, nb);
            if (val >= 0 && val < 50 && visited.find(nb) == visited.end()) {
                visited[nb] = true;
                q.push(nb);
            } else if (is_frontier_cell(map, nb, visited)) {
                Frontier f = build_frontier(map, nb, visited);
                if (f.size >= 10) {          // minimum frontier size
                    frontiers.push_back(f);
                }
            }
        }
    }
    return frontiers;
}


ExplorerNode::ExplorerNode()
: Node("explorer_node"), is_working_(false), nav2_active_(false)
{
    using std::placeholders::_1;

    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&ExplorerNode::map_callback, this, _1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    lifecycle_client_ = this->create_client<lifecycle_msgs::srv::GetState>("bt_navigator/get_state");

    RCLCPP_INFO(this->get_logger(), "Explorer Node Started. Waiting for SLAM Map...");
}

void ExplorerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (is_working_) return;

    if (!nav2_active_) {
        if (!first_map_) {
            first_map_ = msg;
            RCLCPP_INFO(this->get_logger(), "First map received, waiting for Nav2...");
            check_nav2_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&ExplorerNode::check_nav2_state, this));
        }
        return;
    }

    process_map(msg);
}

void ExplorerNode::check_nav2_state()
{
    if (!lifecycle_client_->service_is_ready()) return;

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = lifecycle_client_->async_send_request(request,
        [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
            auto state = future.get()->current_state.id;
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(), "bt_navigator is now active.");
                nav2_active_ = true;
                check_nav2_timer_->cancel();
                if (first_map_) {
                    process_map(first_map_);
                    first_map_.reset();
                }
            }
        });
}

void ExplorerNode::process_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    geometry_msgs::msg::Point robot_pos;
    PointInt robot_cell;
    geometry_msgs::msg::Quaternion robot_orient;
    try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_pos.x = t.transform.translation.x;
        robot_pos.y = t.transform.translation.y;
        robot_pos.z = 0.0;
        robot_orient = t.transform.rotation;

        robot_cell.first = static_cast<int>((robot_pos.x - msg->info.origin.position.x) / msg->info.resolution);
        robot_cell.second = static_cast<int>((robot_pos.y - msg->info.origin.position.y) / msg->info.resolution);
    } catch (const std::exception &ex) {
        RCLCPP_WARN(this->get_logger(), "Waiting for TF...");
        return;
    }

    std::vector<Frontier> frontiers = find_frontiers(*msg, robot_cell);

    if (frontiers.empty()) {
        RCLCPP_WARN(this->get_logger(), "No frontiers found. Exploration might be complete!");
        return;
    }

    const double PULL_BACK = 0.4;
    geometry_msgs::msg::Point best_goal;
    double best_score = -std::numeric_limits<double>::max();
    uint32_t best_size = 0;
    double best_dist = 0.0;
    bool found = false;

    for (const auto& f : frontiers) {
        double dist = std::hypot(f.centroid.x - robot_pos.x, f.centroid.y - robot_pos.y);

        if (dist < 0.5) continue;

        double score = static_cast<double>(f.size) / (dist + 0.1);

        double angle_to_robot = std::atan2(robot_pos.y - f.centroid.y, robot_pos.x - f.centroid.x);
        geometry_msgs::msg::Point potential_goal;
        potential_goal.x = f.centroid.x + PULL_BACK * std::cos(angle_to_robot);
        potential_goal.y = f.centroid.y + PULL_BACK * std::sin(angle_to_robot);
        potential_goal.z = 0.0;
        
        bool is_free_space = false;
        int gx = static_cast<int>((potential_goal.x - msg->info.origin.position.x) / msg->info.resolution);
        int gy = static_cast<int>((potential_goal.y - msg->info.origin.position.y) / msg->info.resolution);

        if (gx >= 0 && gx < static_cast<int>(msg->info.width) &&
            gy >= 0 && gy < static_cast<int>(msg->info.height)) {
            
            int idx = gy * msg->info.width + gx;
            int8_t cost = msg->data[idx];
            if (cost >= 0 && cost < 50) { 
                is_free_space = true; // Cell is truly free space
            }
        }

        if (!is_free_space) {
            continue; // Goal is inside a wall or unknown, skip it
        }

        // Check if blacklisted
        bool is_blacklisted = false;
        for (const auto& bad_goal : blacklist_) {
            if (std::hypot(potential_goal.x - bad_goal.x, potential_goal.y - bad_goal.y) < 1.0) {
                is_blacklisted = true;
                break;
            }
        }

        if (!is_blacklisted && score > best_score) {
            best_score = score;
            best_goal = potential_goal;
            best_size = f.size;          
            best_dist = dist;             
            found = true;
        }
    }

    if (found) {
        RCLCPP_INFO(this->get_logger(), "Selected frontier with score %.3f (size %u, dist %.2f m)",
                    best_score, best_size, best_dist);
        double yaw_to_goal = std::atan2(best_goal.y - robot_pos.y, best_goal.x - robot_pos.x);
        send_goal(best_goal, yaw_to_goal);
        return;
    }

    // Fallback: all frontiers blacklisted
    RCLCPP_WARN(this->get_logger(), "All frontiers are blacklisted! Clearing blacklist and rotating in place.");
    blacklist_.clear();
    double yaw = tf2::getYaw(robot_orient) + 1.57;
    send_goal(robot_pos, yaw);
}

void ExplorerNode::send_goal(geometry_msgs::msg::Point p, double yaw)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "Action server not available!");
        return;
    }

    current_goal_ = p;   

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position = p;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto ops = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    ops.goal_response_callback = [this](const GoalHandleNav::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was REJECTED by Nav2 Server! Blacklisting and unlocking.");
            this->blacklist_.push_back(this->current_goal_);
            this->is_working_ = false; // FREE THE LOCK!
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2, navigating...");
        }
    };

    ops.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
        is_working_ = false; // Free lock on completion
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "== Goal Reached Successfully! ==");
        } else {
            RCLCPP_WARN(this->get_logger(), "!! Goal Failed/Aborted !! Blacklisting this location.");
            blacklist_.push_back(current_goal_);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    };

    is_working_ = true;
    nav_client_->async_send_goal(goal_msg, ops);
}

}  // namespace autonomous_explorer

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<autonomous_explorer::ExplorerNode>());
    rclcpp::shutdown();
    return 0;
}