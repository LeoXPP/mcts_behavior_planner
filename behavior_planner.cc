#include "behavior_planner.h"
#include <algorithm>
#include <iostream>
#include <chrono>

namespace apollo {
namespace BehaviorPlanner {
bool Planner::MakeDecision() {
  if (!BuildGamingInfo()) {
    return false;
  }
  auto start_t = std::chrono::high_resolution_clock::now();
  

  return true;
}

bool Planner::BuildGamingInfo() {
  if (is_first_entry_) {
    is_first_entry_ = false;
  }

  if (!UpdateDecisionParams()) {
    return false;
  }

  mcts_param_.last_modified_trajectories.clear();

  // xiepanpan: temp set constant value
  mcts_param_.ego_target_speed = std::min(mcts_param_.veh_param.max_vel, 10.0);

  return true;
}

void Planner::LoadParams(const TaskConfig& config) {
    // ATRACE << "xiepanpan: XICABehaviorDecider begin to LoadParams .";
    config_ = config.x_ica_behavior_decider_config();
    mcts_dynamics_ = config_.mcts_dynamics();
  
    // Vehicle param
    VehConfig veh_param;
    veh_param.length = config_.veh_ego_length();
    veh_param.width = config_.veh_ego_width();
    veh_param.max_vel = mcts_dynamics_.max_v();
    veh_param.max_lat_acc = mcts_dynamics_.max_lat_acc();
    veh_param.max_ddkappa = mcts_dynamics_.max_ddkappa();
    veh_param.max_kappa = mcts_dynamics_.max_kappa();
    veh_param.max_acc = config_.veh_max_acc();
    veh_param.min_acc = config_.veh_min_acc();
    veh_param.max_delta_l = config_.max_delta_l();
  
    // MCTS param
    XICAMCTSParam mcts_param;
    mcts_param.max_search_iter = config_.mcts_max_search_iter();
    mcts_param.max_search_time = config_.mcts_max_search_time();
    mcts_param.pool_size = config_.mcts_node_pool_size();
    mcts_param.veh_param = veh_param;
    mcts_param.gamma = config_.gamma();
    mcts_param.c = config_.c();
    mcts_param.left_turn_range = config_.left_turn_range();
    mcts_param.heading_range = config_.heading_range();
    mcts_param.min_valid_node_num = config_.min_valid_node_num();
    mcts_param.lk_lat_range = config_.lk_lat_range();
    mcts_param.lat_expand_factor = config_.lat_expand_factor();
    mcts_param.long_expand_factor = config_.long_expand_factor();
    mcts_param.min_decision_distance = config_.min_decision_distance();
    mcts_param.min_decision_delta_t = config_.min_decision_delta_t();
    mcts_param.xica_diff_dis_max = config_.xica_diff_dis_max();
    mcts_param.xica_diff_v_max = config_.xica_diff_v_max();
    mcts_param.xica_diff_a_max = config_.xica_diff_a_max();
  
    mcts_param.ego_agent_reward_adjust = config_.ego_agent_reward_adjust();
  
    mcts_param.use_ref_pre_construct = config_.use_ref_pre_construct();
    mcts_param.xica_need_preconstruct = config_.xica_need_preconstruct();
    mcts_param.xica_need_ego_idm = config_.xica_need_ego_idm();
  
    // ATRACE << "xiepanpan: xica_need_ego_idm = " << mcts_param.xica_need_ego_idm;
    
    mcts_param.occ_bound_max = config_.occ_bound_max();
    // mcts_param.xica_max_d_theta = config_.xica_max_d_theta();
  
    mcts_param.reward_info.w_acc = config_.w_acc();
    mcts_param.reward_info.w_ref = config_.w_ref();
    mcts_param.reward_info.w_eff = config_.w_eff();
    mcts_param.reward_info.w_safe = config_.w_safe();
    mcts_param.reward_info.w_pred = config_.w_pred();
    mcts_param.reward_info.w_cons_act = config_.w_cons_act();
    mcts_param.reward_info.w_cons_his = config_.w_cons_his();
    mcts_param.reward_info.invalid_penalty = config_.invalid_penalty();
  
    mcts_param.xica_reward_info.w_acc = config_.w_acc();
    mcts_param.xica_reward_info.w_ref = config_.w_ref();
    mcts_param.xica_reward_info.w_eff = config_.w_eff();
    mcts_param.xica_reward_info.w_safe = config_.w_safe();
    mcts_param.xica_reward_info.w_pred = config_.w_pred();
    mcts_param.xica_reward_info.w_cons_act = config_.w_cons_act();
    mcts_param.xica_reward_info.w_cons_his = config_.w_cons_his();
    mcts_param.xica_reward_info.invalid_penalty = config_.invalid_penalty();
  
    mcts_param.xica_reward_info.xica_w_eff = config_.xica_w_eff();
    mcts_param.xica_reward_info.xica_w_acc = config_.xica_w_acc();
  
    mcts_param.xica_reward_info.xica_w_safe = config_.xica_w_safe();
  
    mcts_param.xica_reward_info.xica_w_occ = config_.xica_w_occ();
  
    mcts_param.xica_reward_info.xica_w_cons_his = config_.xica_w_cons_his();
  
    mcts_param.xica_idm_param.max_acc_ = config_.max_acc_();
    mcts_param.xica_idm_param.comfort_acc_ = config_.comfort_acc_();
    mcts_param.xica_idm_param.acc_exp_ = config_.acc_exp_();
    mcts_param.xica_idm_param.idm_min_dist_ = config_.idm_min_dist_();
    mcts_param.xica_idm_param.idm_desired_time_ = config_.idm_desired_time_();
    mcts_param.xica_idm_param.idmepsilon = config_.idmepsilon();
  
    mcts_param.xica_reward_info.xica_w_refline = config_.xica_w_refline();
    mcts_param_ = mcts_param;
  }
  





int main() {
    Planner planner;
    XICABehaviorDeciderConfig config;
  
    // 假设你有一个配置解析器来解析配置文件
    if (!ParseConfig("path/to/config/file", &config)) {
      std::cerr << "Failed to parse config file." << std::endl;
      return -1;
    }
    planner.MakeDecision();

    
    return 0;
  }

} // namespace BehaviorPlanner
} // namespace apollo
