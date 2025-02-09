# pragma once

#include <vector>
#include <iostream>
#include <string>

namespace BehaviorPlanner{

struct RewardInfo {
  double w_ref = 0.0;
  double w_eff = 0.0;
  double w_acc = 0.0;
  double w_safe = 0.0;
  double w_pred = 0.0;
  double w_cons_act = 0.0;
  double w_cons_his = 0.0;
  double w_intention = 0.0;
  double invalid_penalty = 0.0;
};

class Planner{

public:
    ~Planner(){}

    bool MakeDecision();
    void InitParam();
    


private:
    std::vector<std::string> behaviors_;
};




}// namespace BehaviorPlanner