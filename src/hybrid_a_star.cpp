#include "hybrid_a_star.h"

namespace U_turn_scenario{
namespace hybrid_a_star{

HybridAStarSolver::HybridAStarSolver(const GridInfo &map_info, const MotionConfig &motion_config):
      map_info_(map_info),
      motion_config_(motion_config)
      {

      }  

// 搜索主函数
std::vector<State> HybridAStarSolver::Solve(const Point &start, const Point &goal,
                                             bool *goal_reached){
      start_ = start;
      goal_  = goal;
      theta_start_to_goal_ = std::atan2(goal.y - start.y, goal.x - start.x);

}



  }
}