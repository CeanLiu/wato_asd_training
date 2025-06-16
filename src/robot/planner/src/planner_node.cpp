#include "planner_node.hpp"
#include <unordered_map>
#include <queue>
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallBack, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallBack, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));
}

void PlannerNode::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_map_ = *msg;
    if(state_ == State::REACHING_GOAL) {
        // If we are already reaching a goal, we can start planning
        planPath();
    }
}

void PlannerNode::goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // Store the latest goal point
    goal_received_ = true;
    latest_goal_ = *msg;
    state_ = State::REACHING_GOAL;
    planPath();
}
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Store the latest odometry
    latest_odom_ = msg->pose.pose;
}

bool PlannerNode::goalReached(){
    double dx = latest_goal_.point.x - latest_odom_.position.x;
    double dy = latest_goal_.point.y - latest_odom_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::timerCallBack() {
    if(state_ == State::REACHING_GOAL) {
        if(goalReached()){
            state_ = State::WAITING_FOR_GOAL;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            goal_received_ = false;
        }else{
            planPath();
            RCLCPP_INFO(this->get_logger(), "Timeout, Replanning path....");
        }
    }
}

std::vector<PlannerNode::CellIndex> PlannerNode::getNeighbour(CellIndex current){
    std::vector<CellIndex> neighbours;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue; // Skip the current cell
            if (std::abs(dx) + std::abs(dy) > 1) continue; // Skip diagonal moves
            CellIndex neighbour(current.x + dx, current.y + dy);
            // RCLCPP_INFO(this->get_logger(), "Checking latest_map_ at (%d, %d)", neighbour.x, neighbour.y);
            // RCLCPP_INFO(this->get_logger(), "Checking latest_map_ at (%d, %d) with value %d", neighbour.x, neighbour.y, 
            //            latest_map_.data[neighbour.y * latest_map_.info.width + neighbour.x]);
            // Check if the neighbour is within bounds and not an obstacle
            if (neighbour.x >= 0 && neighbour.x < latest_map_.info.width &&
                neighbour.y >= 0 && neighbour.y < latest_map_.info.height &&
                latest_map_.data[neighbour.y * latest_map_.info.width + neighbour.x] == 0) {
                neighbours.push_back(neighbour);
            }
        }
    }
    return neighbours;
}

void PlannerNode::planPath(){
    if(goal_received_ == false || latest_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid goal or map data available for planning.");
        return;
    }
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

    double start_x = latest_odom_.position.x;
    double start_y = latest_odom_.position.y;
    double goal_x = latest_goal_.point.x;
    double goal_y = latest_goal_.point.y;
    CellIndex start = CellIndex(
        static_cast<int>((start_x - latest_map_.info.origin.position.x) / latest_map_.info.resolution),
        static_cast<int>((start_y - latest_map_.info.origin.position.y) / latest_map_.info.resolution)
    );

    CellIndex goal = CellIndex(
        static_cast<int>((goal_x - latest_map_.info.origin.position.x) / latest_map_.info.resolution),
        static_cast<int>((goal_y - latest_map_.info.origin.position.y) / latest_map_.info.resolution)
    );
    // RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", start.x, start.y, goal.x, goal.y);
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;   
    std::unordered_set<CellIndex, CellIndexHash> closed_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    open_set.push(AStarNode(start, 0.0));  
    g_score[start] = 0.0; // Cost from start to start is zero
    while (!open_set.empty()){
        // RCLCPP_INFO(this->get_logger(), "Open set size: %zu", open_set.size());
        AStarNode current_node = open_set.top();
        const CellIndex &current = current_node.index;
        open_set.pop();
        closed_set.insert(current);
        if (current == goal){
            // RCLCPP_INFO(this->get_logger(), "Goal found at (%d, %d)", goal.x, goal.y);
            break;
        }
        std::vector<CellIndex> neighbours = getNeighbour(current);
        // RCLCPP_INFO(this->get_logger(), "Current node (%d, %d) with %zu neighbours", current.x, current.y, neighbours.size());
        for (const CellIndex &neighbour : neighbours){
            // RCLCPP_INFO(this->get_logger(), "Evaluating neighbour (%d, %d)", neighbour.x, neighbour.y);
            double cost = latest_map_.data[neighbour.y * latest_map_.info.width + neighbour.x];
            if (closed_set.find(neighbour) != closed_set.end() || cost > 0) {
                continue; // Skip already evaluated nodes
            }
            double tentative_g = g_score[current] + current.getDistance(neighbour);

            if (g_score.find(neighbour) == g_score.end() || tentative_g < g_score[neighbour]) {
                g_score[neighbour] = tentative_g;
                came_from[neighbour] = current;

                double h = goal.getDistance(neighbour);
                double f = tentative_g + h;
                open_set.push(AStarNode(neighbour, f));
            }
        }
    }
    if (came_from.find(goal) == came_from.end()) {
        RCLCPP_WARN(this->get_logger(), "No path found to goal.");
        return;
    }
    CellIndex current = goal;
    while (true) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = current.x * latest_map_.info.resolution + latest_map_.info.origin.position.x;
        pose.pose.position.y = current.y * latest_map_.info.resolution + latest_map_.info.origin.position.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        // Insert at front to avoid reversing later
        path.poses.insert(path.poses.begin(), pose);

        if (current == start) break;
        current = came_from[current];
    }
    path_pub_->publish(path);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
