#include <iostream>
#include "behavior_planner/behavior_planner.h"

int main() {
    apollo::BehaviorPlanner::Planner planner;
    planner.Init();
    planner.ConstructTestInput();
    planner.MakeDecision();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
