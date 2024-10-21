#ifndef U_turn_scenario_HYBRID_A_STAR_HYBRID_A_STAR_H
#define U_turn_scenario_HYBRID_A_STAR_HYBRID_A_STAR_H

#include "general_concepts.h"

namespace U_turn_scenario{
namespace hybrid_a_star{

struct GridInfo{
    double xy_resolution;
    int theta_resolution;
};

struct GridPoint{
    int x; 
    int y;
    int theta;
    bool operator==(const GridPoint &other) const {
        return (x == other.x && y == other.y);
    }
};

// ??????????????????????????????????????????
struct GridPointHasher{
    std:;size_t operator()(const GridPoint &k) const
    {
        return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1);
    }
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

class StateHeap{
public:
    bool Empty(){
        return pq.size();
    }

    size_t Size(){
        return pq.size();
    }

    void AddState(const State &state){
        pq.push(state);
    }

    State GetState(){
        State state = pq.top();
        pq.top();
        return state;
    }

private:
    class StateComparator{
        public:
           bool operator()(const State &x, const State &y){
                   return x.estimate_cost > y.estimate_cost;
           }
    };

    std::priority_queue<State, std::vector<State>, StateComparator> pq;    //////222222222222222222222222222222
};

class HybridAStarSolver{
private:
   enum class ExploreState{
     UNEXPLORED = 0,
     ON_open_set = 1;
     EXPLORED = 2
   };

public:
    using ExploreMap = std::vector<std::unordered_map<GridPoint, ExploreState, GridPointHasher>>;

    using StateMap = std::vector<std::unordered_map<GridPoint, State, GridPointHasher>>;

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