
#include <iostream>
#include "behavior_planner/behavior_planner.h"
#include <nlohmann/json.hpp>
#include <fstream>

int main() {
    std::vector<std::pair<double, double>> time_cost_record;
    for(int i = 0; i < 1; i++) {
        apollo::BehaviorPlanner::Planner planner;
        planner.Init();

        double time_cost = 0.0;
        TestInputParams input_params;
        planner.ConstructTestInput(input_params);
        planner.MakeDecision(time_cost);
        time_cost_record.push_back(std::make_pair(i, time_cost));
        std::cout << "The " << i <<" round simulate finished" << "Time cost is " << time_cost<< std::endl;
    }
    // nlohmann::json time_cost_json;
    // for(const auto& record : time_cost_record){
    //     time_cost_json.push_back({{"round", record.first}, {"time_cost", record.second}});
    // }
    // std::ofstream out_file("time_cost_record.json");
    // out_file << time_cost_json.dump(4);
    // out_file.close();

    // std::cout << "Time cost record saved to time_cost_record.json" << std::endl;
    return 0;
}
