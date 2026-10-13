#include "amr_global_planners/grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_global_planners
{
namespace
{

struct QueueNode
{
  std::size_t index;
  double g;
  double f;
};

struct QueueCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    // std::priority_queue is a max-heap by default.
    // Reverse the comparison so the smallest f-value is popped first.
    return a.f > b.f;
  }
};

inline std::size_t toIndex(unsigned int x, unsigned int y, unsigned int size_x)
{
  return static_cast<std::size_t>(y) * size_x + x;
}

inline void fromIndex(
  std::size_t index,
  unsigned int size_x,
  unsigned int & x,
  unsigned int & y)
{
  x = static_cast<unsigned int>(index % size_x);
  y = static_cast<unsigned int>(index / size_x);
}

bool isTraversable(
  const nav2_costmap_2d::Costmap2D & costmap,
  int x,
  int y,
  bool allow_unknown)
{
  if (x < 0 || y < 0 ||
      x >= static_cast<int>(costmap.getSizeInCellsX()) ||
      y >= static_cast<int>(costmap.getSizeInCellsY()))
  {
    return false;
  }

  const unsigned char cost = costmap.getCost(
    static_cast<unsigned int>(x),
    static_cast<unsigned int>(y));

  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }

  // INSCRIBED_INFLATED_OBSTACLE (253) and LETHAL_OBSTACLE (254)
  // are not valid robot-center cells.
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// Octile distance is admissible for an 8-connected grid where straight moves
// cost 1 and diagonal moves cost sqrt(2). Therefore A* remains optimal with
// respect to the same edge-cost model used by Dijkstra.
double octileHeuristic(
  unsigned int x,
  unsigned int y,
  unsigned int goal_x,
  unsigned int goal_y)
{
  const double dx = std::abs(
    static_cast<double>(goal_x) - static_cast<double>(x));
  const double dy = std::abs(
    static_cast<double>(goal_y) - static_cast<double>(y));

  const double diagonal = std::min(dx, dy);
  const double straight = std::max(dx, dy) - diagonal;

  return diagonal * std::sqrt(2.0) + straight;
}

}  // namespace

GridSearchResult runGridSearch(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned int start_x,
  unsigned int start_y,
  unsigned int goal_x,
  unsigned int goal_y,
  bool use_heuristic,
  bool allow_unknown,
  double cost_penalty)
{
  GridSearchResult result;

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const std::size_t total_cells =
    static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);

  if (size_x == 0 || size_y == 0) {
    return result;
  }

  if (!isTraversable(costmap, start_x, start_y, allow_unknown) ||
      !isTraversable(costmap, goal_x, goal_y, allow_unknown))
  {
    return result;
  }

  const std::size_t start_index = toIndex(start_x, start_y, size_x);
  const std::size_t goal_index = toIndex(goal_x, goal_y, size_x);

  const double inf = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(total_cells, inf);
  std::vector<std::size_t> parent(total_cells, total_cells);
  std::vector<bool> closed(total_cells, false);

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;

  g_score[start_index] = 0.0;
  const double start_h = use_heuristic ?
    octileHeuristic(start_x, start_y, goal_x, goal_y) : 0.0;
  open.push(QueueNode{start_index, 0.0, start_h});

  // 8-connected neighborhood.
  constexpr int directions[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    if (closed[current.index]) {
      continue;
    }

    // Skip stale priority-queue entries created before a better g-score arrived.
    if (current.g > g_score[current.index]) {
      continue;
    }

    closed[current.index] = true;
    ++result.expanded_nodes;

    if (current.index == goal_index) {
      result.success = true;
      break;
    }

    unsigned int current_x = 0;
    unsigned int current_y = 0;
    fromIndex(current.index, size_x, current_x, current_y);

    for (const auto & direction : directions) {
      const int nx = static_cast<int>(current_x) + direction[0];
      const int ny = static_cast<int>(current_y) + direction[1];

      if (!isTraversable(costmap, nx, ny, allow_unknown)) {
        continue;
      }

      const bool diagonal = direction[0] != 0 && direction[1] != 0;

      // Do not let the robot center cut through an obstacle corner.
      if (diagonal) {
        if (!isTraversable(
              costmap,
              static_cast<int>(current_x) + direction[0],
              static_cast<int>(current_y),
              allow_unknown) ||
            !isTraversable(
              costmap,
              static_cast<int>(current_x),
              static_cast<int>(current_y) + direction[1],
              allow_unknown))
        {
          continue;
        }
      }

      const unsigned int neighbor_x = static_cast<unsigned int>(nx);
      const unsigned int neighbor_y = static_cast<unsigned int>(ny);
      const std::size_t neighbor_index = toIndex(neighbor_x, neighbor_y, size_x);

      if (closed[neighbor_index]) {
        continue;
      }

      const double geometric_step = diagonal ? std::sqrt(2.0) : 1.0;

      const unsigned char raw_cost = costmap.getCost(neighbor_x, neighbor_y);
      double normalized_cost = 0.0;
      if (raw_cost == nav2_costmap_2d::NO_INFORMATION) {
        normalized_cost = allow_unknown ? 1.0 : 0.0;
      } else {
        // 252 is the largest non-inscribed traversable cost.
        normalized_cost = static_cast<double>(raw_cost) / 252.0;
      }

      const double tentative_g =
        g_score[current.index] + geometric_step + cost_penalty * normalized_cost;

      if (tentative_g >= g_score[neighbor_index]) {
        continue;
      }

      g_score[neighbor_index] = tentative_g;
      parent[neighbor_index] = current.index;

      const double h = use_heuristic ?
        octileHeuristic(neighbor_x, neighbor_y, goal_x, goal_y) : 0.0;

      open.push(QueueNode{
        neighbor_index,
        tentative_g,
        tentative_g + h
      });
    }
  }

  if (!result.success) {
    return result;
  }

  // Reconstruct goal -> start through the parent array, then reverse it.
  std::vector<std::pair<unsigned int, unsigned int>> reversed_path;
  std::size_t cursor = goal_index;

  while (true) {
    unsigned int x = 0;
    unsigned int y = 0;
    fromIndex(cursor, size_x, x, y);
    reversed_path.emplace_back(x, y);

    if (cursor == start_index) {
      break;
    }

    if (parent[cursor] == total_cells) {
      result.success = false;
      result.cells.clear();
      return result;
    }

    cursor = parent[cursor];
  }

  std::reverse(reversed_path.begin(), reversed_path.end());
  result.cells = std::move(reversed_path);
  return result;
}

}  // namespace amr_global_planners
