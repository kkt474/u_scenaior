#include "hybrid_a_star.h"
#include <cmath>

namespace U_turn_scenario{
namespace hybrid_a_star{

HybridAStarSolver::HybridAStarSolver(const GridInfo &map_info, const MotionConfig &motion_config):
      map_info_(map_info),
      motion_config_(motion_config)
      {
       /**/
      }  

double HybridAStarSolver::Distance(double x1, double y1, double x2, double y2)
{
      return std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2));
}

// 搜索主函数
std::vector<State> HybridAStarSolver::Solve(const Point &start, const Point &goal,
                                             bool *goal_reached){
      start_ = start;
      goal_  = goal;
      theta_start_to_goal_ = std::atan2(goal.y - start.y, goal.x - start.x);
      distance_start_to_goal_ = Distance(star.x, start.y, goal.x, goal.y);

      // explored configuration space
      ExploreMap explored;
      explored.resize(map_info_.theta_resolution);
      // parent state for reconstruction
      StateMap parent;
      parent.resize(map_info_.theta_resolution);

      // open_set
      StateHeap open_set;
      State init;
      init.x = start.x;
      init.y = start.y;
      init.theta = start.theta;
      init.actual_cost = 0.;
      init.estimated_cost = 0.;



}



  }
}