
#include <iostream>
#include "behavior_planner/behavior_planner.h"

int main() {
    apollo::BehaviorPlanner::Planner planner;
    planner.Init();

    TestInputParams input_params;
    planner.ConstructTestInput(input_params);
    planner.MakeDecision();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
