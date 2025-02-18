#include <iostream>
#include "behavior_planner/behavior_planner.h"

// namespace apollo {
// namespace BehaviorPlanner {



int main() {
    apollo::BehaviorPlanner::Planner planner;
    planner.Init();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}

// } // namespace BehaviorPlanner
// } // namespace apollo  