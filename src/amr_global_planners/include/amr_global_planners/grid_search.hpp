#ifndef AMR_GLOBAL_PLANNERS__GRID_SEARCH_HPP_
#define AMR_GLOBAL_PLANNERS__GRID_SEARCH_HPP_

#include <cstddef>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace amr_global_planners
{

/**
 * @brief Result returned by the common grid-search engine.
 *
 * Dijkstra and A* deliberately share the same search engine so that the
 * benchmark changes only one important variable: whether a heuristic is used.
 */
struct GridSearchResult
{
  bool success{false};
  std::vector<std::pair<unsigned int, unsigned int>> cells;
  std::size_t expanded_nodes{0};
};

/**
 * @brief Run an 8-connected grid search on a Nav2 Costmap2D.
 *
 * @param costmap         Nav2 global costmap used by both planners.
 * @param start_x/y       Start cell.
 * @param goal_x/y        Goal cell.
 * @param use_heuristic   false -> Dijkstra, true -> A*.
 * @param allow_unknown   Whether NO_INFORMATION cells may be traversed.
 * @param cost_penalty    Additional penalty for high-cost inflated cells.
 *
 * The two planners therefore use identical:
 * - map / costmap
 * - connectivity
 * - diagonal cost
 * - obstacle rule
 * - corner-cutting rule
 * - cost penalty
 *
 * and differ only in the heuristic term.
 */
GridSearchResult runGridSearch(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned int start_x,
  unsigned int start_y,
  unsigned int goal_x,
  unsigned int goal_y,
  bool use_heuristic,
  bool allow_unknown,
  double cost_penalty);

}  // namespace amr_global_planners

#endif  // AMR_GLOBAL_PLANNERS__GRID_SEARCH_HPP_
