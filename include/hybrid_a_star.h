#ifndef U_turn_scenario_HYBRID_A_STAR_HYBRID_A_STAR_H
#define U_turn_scenario_HYBRID_A_STAR_HYBRID_A_STAR_H

#include "general_concepts.h"

namespace U_turn_scenario{
namespace hybrid_a_star{

struct GridInfo{
    double xy_resolution;
    int theta_resolution;
};

struct State{
    double x;
    double y;
    double theta;
    double actual_cost;
    double estimate_cost;
};

struct SearchConfig{
    double steering_angle_step;
    double time_step;
};

class HybridAStarSolver{

public:
    HybridAStarSolver(const GridInfo &map_info, const MotionConfig &motion_config);

    std::vector<State> Solve(const Point &start, const Point &goal, bool *goal_reached);

private:
    double Distance(double x1, double y1, double x2, double y2);

private:
    GridInfo map_info_;
    MotionConfig motion_config_;
    Point start_;
    Point end_;
    double distance_start_to_goal_;
    double theta_start_to_goal_;
   

};






  }   // namespace hybrid_a_star
}    //  namespace U_turn_scenario


#endif