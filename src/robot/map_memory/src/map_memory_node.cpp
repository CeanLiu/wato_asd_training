#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x(-15), last_y(-15), distance_threshold(1.5), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    // Initialize subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    initializeGlobal();
    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);    
    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}


void MapMemoryNode::initializeGlobal(){
  // Initialize the costmap here
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.info.origin.position.x = -15;
    global_map_.info.origin.position.y = -15;
    global_map_.info.origin.position.z = 0;
    global_map_.header.frame_id = "sim_world";
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto q = msg->pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    // Compute distance traveled
    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y; 
        last_yaw = yaw;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }

}

void MapMemoryNode::integrateCostmap() {
    double resolution = latest_costmap_.info.resolution;
    double origin_x = latest_costmap_.info.origin.position.x;
    double origin_y = latest_costmap_.info.origin.position.y;

    for (int y = 0; y < latest_costmap_.info.height; ++y) {
        for (int x = 0; x < latest_costmap_.info.width; ++x) {
            int robot_index = y * latest_costmap_.info.width + x;
            if (latest_costmap_.data[robot_index] == -1) continue;

            // Convert from costmap indices to metric coordinates in robot frame
            double local_x = origin_x + x * resolution;
            double local_y = origin_y + y * resolution;

            // Rotate and translate to global coordinates (based on robot pose)
            double global_x = std::cos(last_yaw) * local_x - std::sin(last_yaw) * local_y + last_x;
            double global_y = std::sin(last_yaw) * local_x + std::cos(last_yaw) * local_y + last_y;

            // Convert metric global coords to global map indices
            int gx = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
            int gy = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);
            int global_index = gy * global_map_.info.width + gx;
            // Bounds check
            if (gx >= 0 && gx < global_map_.info.width &&
                gy >= 0 && gy < global_map_.info.height && 
                latest_costmap_.data[robot_index] > global_map_.data[global_index]) {
                global_map_.data[global_index] = latest_costmap_.data[robot_index];
            }
        }
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}