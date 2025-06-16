#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State {WAITING_FOR_GOAL, REACHING_GOAL};
    State state_ = State::WAITING_FOR_GOAL;
    bool goal_received_ = false;
    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    geometry_msgs::msg::PointStamped latest_goal_;  
    nav_msgs::msg::OccupancyGrid latest_map_;
    geometry_msgs::msg::Pose latest_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool goalReached();
    void timerCallBack();
    void planPath();
    // ------------------- Supporting Structures -------------------
    // 2D grid index
    struct CellIndex
    {
      int x;
      int y;
    
      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}
    
      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }
    
      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }

      double getDistance(const CellIndex &other) const
      {
        if(x == other.x && y == other.y) return 0.0; // Same cell
        else{
          int dx = std::abs(x - other.x);
          int dy = std::abs(y - other.y);
          return (dx + dy) + (1.4 - 2.0) * std::min(dx, dy);
        }
      }
    };

    std::vector<CellIndex> getNeighbour(CellIndex current);

    
    // Hash function for CellIndex so it can be used in std::unordered_map
    struct CellIndexHash
    {
      std::size_t operator()(const CellIndex &idx) const
      {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
      }
    };
    
    // Structure representing a node in the A* open set
    struct AStarNode
    {
      CellIndex index;
      double f_score;  // f = g + h
    
      AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
    };
    
    // Comparator for the priority queue (min-heap by f_score)
    struct CompareF
    {
      bool operator()(const AStarNode &a, const AStarNode &b)
      {
        // We want the node with the smallest f_score on top
        return a.f_score > b.f_score;
      }
    };
};

#endif 
