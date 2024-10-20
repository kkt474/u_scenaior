#include <iostream>
#include <cmath>


#include "general_concepts.h"
#include "hybrid_a_star.h"
#include "u_turn_planner.h"


using namespace std;

int main(){
    // load planner config
    U_turn_scenario::MotionConfig motion_config = {
           1.0,
           3,
           40.0,
           -40.0
    };
    U_turn_scenario::hybrid_a_star::SearchConfig search_config = {
           40,
           1
    };
    // load lane env from file
    U_turn_scenario::Lane start_lane;
    U_turn_scenario::Lane end_lane;
    start_lane.center_point = {{0.,100.,M_PI/2}};
    end_lane.center_point = {{8,100.,-M_PI/2}};

    // init planner --  plan
    U_turn_scenario::UTurnPlanner u_turn_planner(motion_config);

    u_turn_planner.setEnvironment(start_lane, end_lane);

    u_turn_planner.SetHybridAStarConfig(search_config);

    std::vector<U_turn_scenario::Point> a_star_trajectory;

    std::chrono::system_clock::system_clock::time_point start_time = 
       std::chrono::system_clock::system_clock::now();

    bool goal_reached = false;

    std::vector<U_turn_scenario::Point> trajectory = u_turn_planner.Plan(&a_star_trajectory, &goal_reached);

    std::chrono::system_clock::system_clock::time_point end_time = 
       std::chrono::system_clock::system_clock::now();
    std::chrono::curation<double> elapsed_seconds = end_time - start_time;
    std::cout << "time spend: " << elapsed_seconds.count() << " s" << std::endl;

    if (!goal_reached){
        std::cout << "can not reach target goal, may be you need move a little bit" << std::endl;
    }

    // plot
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> u;
    std::vector<double> v;


    return 0;
}
 
