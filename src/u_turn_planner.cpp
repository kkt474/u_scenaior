#include "u_turn_planner.h"

namespace U_turn_scenario{

// 初始化
UTurnPlanner::UTurnPlanner(const MotionConfig &motion_config):
              motion_config_(motion_config),
              a_star_solver_(){{motion_config.desired_speed,40},motion_config}{}

void UTurnPlanner::setEnvironment(const Lane &start_lane, const Lane &end_lane){
    start_lane_ = start_lane;
    start_point = start_lane.center_point.back();
    end_lane_ = end_lane;
    end_point_ = end_lane.center_point.front();
    a_star_grid_.xy_resolution = motion_config_.desired_speed;
}

void UTurnPlanner::SetHybridAStarConfig(const hybrid_a_star::SearchConfig &search_config){
    a_star_solver_.SetSearchConfig(search_config);
}

std::vector<Point> UTurnPlanner::Plan(std::vector<U_turn_scenario::Point>* a_star_output, bool* goal_reached){
    // 搜索路径
    std::vector<hybrid_a_star::State> a_star_state_traj = 
                                      a_star_solver_.Solve(start_point_, end_point_, goal_reached);
}


}