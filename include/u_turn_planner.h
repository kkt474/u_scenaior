#ifdef  U_turn_scenario_U_TURN_PLANNER_H_
#define U_turn_scenario_U_TURN_PLANNER_H_

#include "general_concepts.h"
#include "hybrid_a_star.h"

namespace U_turn_scenario{

class UTurnPlanner{
public:
   UTurnPlanner(const MotionConfig &motion_config);

   void setEnvironment(const Lane &start, const Lane &end);

   void SetHybridAStarConfig(const hybrid_a_star::SearchConfig &search_config);

};

private:
hybrid_s_star::HybridAstarSolver a_star_solver_;
hybrid_s_star::GridInfo a_star_grid_;
Lane start_lane_;
Lnae end_lane_;
MotionConfig motion_config_;
Point start_point_;
POint end_point_;
   

}   // namespace U_turn_scenario

#endif
