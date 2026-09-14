#include "nav2_astar_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node_->get_logger();

  // 声明并读取插件私有参数：YAML 里写在 GridBased: 下面
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);

  int lethal = 253;
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_threshold", rclcpp::ParameterValue(253));
  node_->get_parameter(name_ + ".lethal_threshold", lethal);
  lethal_threshold_ = static_cast<unsigned char>(lethal);

  RCLCPP_INFO(
    logger_, "AStarPlanner [%s] 已配置：allow_diagonal=%s, lethal_threshold=%d",
    name_.c_str(), allow_diagonal_ ? "true" : "false", lethal);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "清理 AStarPlanner [%s]", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "激活 AStarPlanner [%s]", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "停用 AStarPlanner [%s]", name_.c_str());
}

bool AStarPlanner::worldToMap(double wx, double wy, int & mx, int & my) const
{
  unsigned int umx, umy;
  if (!costmap_->worldToMap(wx, wy, umx, umy)) {
    return false;
  }
  mx = static_cast<int>(umx);
  my = static_cast<int>(umy);
  return true;
}

void AStarPlanner::mapToWorld(int mx, int my, double & wx, double & wy) const
{
  costmap_->mapToWorld(
    static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
}

bool AStarPlanner::isFree(int mx, int my) const
{
  if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
      my >= static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return false;
  }
  // 代价 >= lethal_threshold 视为不可通行。
  // 注意 NO_INFORMATION(255) 也会被挡掉——保守但安全。
  return costmap_->getCost(mx, my) < lethal_threshold_;
}

std::vector<AStarPlanner::Cell> AStarPlanner::aStarSearch(
  const Cell & start, const Cell & goal) const
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  auto index = [width](int x, int y) { return y * width + x; };

  struct Node { int idx; double f; };
  struct Cmp {
    bool operator()(const Node & a, const Node & b) const { return a.f > b.f; }
  };

  const double kSqrt2 = std::sqrt(2.0);
  std::vector<std::pair<int, int>> moves4 = {{1,0},{-1,0},{0,1},{0,-1}};
  std::vector<std::pair<int, int>> moves8 = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  const auto & moves = allow_diagonal_ ? moves8 : moves4;

  auto heuristic = [&](int x, int y) {
    double dx = std::abs(x - goal.x), dy = std::abs(y - goal.y);
    if (allow_diagonal_) {
      return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
    }
    return dx + dy;
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, int> came_from;

  const int start_idx = index(start.x, start.y);
  const int goal_idx = index(goal.x, goal.y);
  g_score[start_idx] = 0.0;
  open.push({start_idx, heuristic(start.x, start.y)});

  while (!open.empty()) {
    const int cur = open.top().idx;
    open.pop();

    if (cur == goal_idx) {
      std::vector<Cell> path;
      for (int i = cur; ; ) {
        path.push_back({i % width, i / width});
        auto it = came_from.find(i);
        if (it == came_from.end()) {break;}
        i = it->second;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const int cx = cur % width, cy = cur / width;
    const double cg = g_score[cur];

    for (const auto & [dx, dy] : moves) {
      const int nx = cx + dx, ny = cy + dy;
      if (!isFree(nx, ny)) {continue;}
      // 禁止斜穿两个障碍之间的缝隙
      if (dx != 0 && dy != 0 && (!isFree(cx + dx, cy) || !isFree(cx, cy + dy))) {
        continue;
      }
      const double step = (dx != 0 && dy != 0) ? kSqrt2 : 1.0;
      const int nidx = index(nx, ny);
      const double ng = cg + step;
      auto it = g_score.find(nidx);
      if (it == g_score.end() || ng < it->second) {
        g_score[nidx] = ng;
        came_from[nidx] = cur;
        open.push({nidx, ng + heuristic(nx, ny)});
      }
    }
  }

  return {};   // 无解
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id != global_frame_)
  {
    RCLCPP_ERROR(
      logger_, "起点/终点的 frame 必须是 %s", global_frame_.c_str());
    return path;
  }

  Cell s{}, g{};
  if (!worldToMap(start.pose.position.x, start.pose.position.y, s.x, s.y)) {
    RCLCPP_ERROR(logger_, "起点在代价地图之外");
    return path;
  }
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, g.x, g.y)) {
    RCLCPP_ERROR(logger_, "目标点在代价地图之外");
    return path;
  }

  const auto cells = aStarSearch(s, g);
  if (cells.empty()) {
    RCLCPP_WARN(logger_, "A* 未找到可行路径");
    return path;
  }

  path.poses.reserve(cells.size());
  for (const auto & c : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    mapToWorld(c.x, c.y, p.pose.position.x, p.pose.position.y);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  // 终点用请求的精确位姿（含朝向），而不是栅格中心
  path.poses.back() = goal;
  path.poses.back().header = path.header;

  RCLCPP_INFO(logger_, "A* 规划成功：%zu 个点", path.poses.size());
  return path;
}

}  // namespace nav2_astar_planner

PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)