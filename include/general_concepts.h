#ifndef U_turn_scenario_GENERAL_CONCEPTS_H
#define U_turn_scenario_GENERAL_CONCEPTS_H

#include <vector>

namespace U_turn_scenario{

struct Point
{
    double x;
    double y;
    double theta;
    double kappa;
};
    
struct MotionConfig
{
    double desired_speed;
    double wheelbase;
    double max_steering_angle;
    double min_steering_angle;
};

struct Lane
{
    std::vector<Point> center_point;
};










}   // namespace U_turn_scenario

#endif