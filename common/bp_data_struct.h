#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "vehicle_state/vehicle_state.h"

namespace apollo {
namespace BehaviorPlanner {

// Sample 结构体
struct Sample {
    double t_;
    std::vector<double> lat_action_;
    std::vector<double> lon_action_;

    // Getter 函数（名称中不带下划线）
    double t() const { return t_; }
    const std::vector<double>& latAction() const { return lat_action_; }
    const std::vector<double>& lonAction() const { return lon_action_; }
};

// Sampler 结构体
struct Sampler {
    std::vector<Sample> samples_;

    // Getter 函数
    const std::vector<Sample>& samples() const { return samples_; }
};

// MCTSDynamics 结构体
struct MCTSDynamics {
    Sampler sampler_;
    double max_ddkappa_;
    double max_jerk_;
    double max_kappa_;
    double max_lat_acc_;
    double max_v_;

    // Getter 函数
    const Sampler& sampler() const { return sampler_; }
    double max_ddkappa() const { return max_ddkappa_; }
    double max_jerk() const { return max_jerk_; }
    double max_kappa() const { return max_kappa_; }
    double max_lat_acc() const { return max_lat_acc_; }
    double max_v() const { return max_v_; }
};


class XICABehaviorDeciderConfig {
public:
    // MCTS 参数
    int mcts_max_search_iter;
    double mcts_max_search_time;
    int mcts_node_pool_size;
    MCTSDynamics mcts_dynamics;

    // 车辆参数
    double veh_max_acc;
    double veh_min_acc;
    double veh_ego_length;
    double veh_ego_width;
    std::vector<double> jerk_action;

    // 车道保持 dkappa 参数
    std::vector<double> lk_dkappa_action;
    std::vector<double> right_dkappa_action;
    std::vector<double> left_dkappa_action;

    // 时间步长
    std::vector<double> coarse_time_step;
    std::vector<double> fine_time_step;

    // 其他参数
    double gamma;
    double c;
    double left_turn_range;
    int max_delta_l;
    double min_decision_distance;
    double heading_range;
    double lk_lat_range;
    double lat_expand_factor;
    double long_expand_factor;
    int min_valid_node_num;
    double vru_max_vel;
    bool is_opposite_collision_func_open;
    bool is_opposite_left_turn_func_open;
    bool is_debug_info_open;
    double invalid_penalty;
    double vel_limit_factor;
    double w_acc;
    double w_eff;
    double w_ref;
    double w_safe;
    double w_pred;
    double w_cons_act;
    double w_cons_his;
    bool use_x_ica_behavior_decider;
    bool use_neighbor_back_obs;
    int lookhead_distance;
    bool use_virtual_obs;
    int nudge_obstacles_num;
    int curr_lane_obstacles_num;
    bool always_use_neighbor_obs;
    int left_obstacles_num;
    int right_obstacles_num;
    bool is_tree_pre_constructed;
    double ego_agent_reward_adjust;
    double xica_w_eff;
    double xica_w_acc;
    double xica_w_safe;
    double xica_w_occ;
    double max_acc_;
    double comfort_acc_;
    double acc_exp_;
    double idm_min_dist_;
    double idm_desired_time_;
    double idmepsilon;
    bool use_ref_pre_construct;
    bool xica_need_preconstruct;
    bool xica_need_ego_idm;
    double occ_bound_max;
    double xica_w_cons_his;
    double min_decision_delta_t;
    double xica_diff_dis_max;
    double xica_diff_v_max;
    double xica_diff_a_max;
    double rel_lat_thresh;
    double xica_w_refline;
    // 注释掉 xica_max_d_theta: 0.8

    // 模拟 protobuf 的 getter 函数（去掉函数名末尾下划线）
    int mcts_max_search_iter() const { return mcts_max_search_iter; }
    double mcts_max_search_time() const { return mcts_max_search_time; }
    int mcts_node_pool_size() const { return mcts_node_pool_size; }
    MCTSDynamics mcts_dynamics() const { return mcts_dynamics; }
    double veh_max_acc() const { return veh_max_acc; }
    double veh_min_acc() const { return veh_min_acc; }
    double veh_ego_length() const { return veh_ego_length; }
    double veh_ego_width() const { return veh_ego_width; }
    std::vector<double> jerk_action() const { return jerk_action; }
    std::vector<double> lk_dkappa_action() const { return lk_dkappa_action; }
    std::vector<double> right_dkappa_action() const { return right_dkappa_action; }
    std::vector<double> left_dkappa_action() const { return left_dkappa_action; }
    std::vector<double> coarse_time_step() const { return coarse_time_step; }
    std::vector<double> fine_time_step() const { return fine_time_step; }
    double gamma() const { return gamma; }
    double c() const { return c; }
    double left_turn_range() const { return left_turn_range; }
    int max_delta_l() const { return max_delta_l; }
    double min_decision_distance() const { return min_decision_distance; }
    double heading_range() const { return heading_range; }
    double lk_lat_range() const { return lk_lat_range; }
    double lat_expand_factor() const { return lat_expand_factor; }
    double long_expand_factor() const { return long_expand_factor; }
    int min_valid_node_num() const { return min_valid_node_num; }
    double vru_max_vel() const { return vru_max_vel; }
    bool is_opposite_collision_func_open() const { return is_opposite_collision_func_open; }
    bool is_opposite_left_turn_func_open() const { return is_opposite_left_turn_func_open; }
    bool is_debug_info_open() const { return is_debug_info_open; }
    double invalid_penalty() const { return invalid_penalty; }
    double vel_limit_factor() const { return vel_limit_factor; }
    double w_acc() const { return w_acc; }
    double w_eff() const { return w_eff; }
    double w_ref() const { return w_ref; }
    double w_safe() const { return w_safe; }
    double w_pred() const { return w_pred; }
    double w_cons_act() const { return w_cons_act; }
    double w_cons_his() const { return w_cons_his; }
    bool use_x_ica_behavior_decider() const { return use_x_ica_behavior_decider; }
    bool use_neighbor_back_obs() const { return use_neighbor_back_obs; }
    int lookhead_distance() const { return lookhead_distance; }
    bool use_virtual_obs() const { return use_virtual_obs; }
    int nudge_obstacles_num() const { return nudge_obstacles_num; }
    int curr_lane_obstacles_num() const { return curr_lane_obstacles_num; }
    bool always_use_neighbor_obs() const { return always_use_neighbor_obs; }
    int left_obstacles_num() const { return left_obstacles_num; }
    int right_obstacles_num() const { return right_obstacles_num; }
    bool is_tree_pre_constructed() const { return is_tree_pre_constructed; }
    double ego_agent_reward_adjust() const { return ego_agent_reward_adjust; }
    double xica_w_eff() const { return xica_w_eff; }
    double xica_w_acc() const { return xica_w_acc; }
    double xica_w_safe() const { return xica_w_safe; }
    double xica_w_occ() const { return xica_w_occ; }
    double max_acc() const { return max_acc_; }
    double comfort_acc() const { return comfort_acc_; }
    double acc_exp() const { return acc_exp_; }
    double idm_min_dist() const { return idm_min_dist_; }
    double idm_desired_time() const { return idm_desired_time_; }
    double idmepsilon() const { return idmepsilon; }
    bool use_ref_pre_construct() const { return use_ref_pre_construct; }
    bool xica_need_preconstruct() const { return xica_need_preconstruct; }
    bool xica_need_ego_idm() const { return xica_need_ego_idm; }
    double occ_bound_max() const { return occ_bound_max; }
    double xica_w_cons_his() const { return xica_w_cons_his; }
    double min_decision_delta_t() const { return min_decision_delta_t; }
    double xica_diff_dis_max() const { return xica_diff_dis_max; }
    double xica_diff_v_max() const { return xica_diff_v_max; }
    double xica_diff_a_max() const { return xica_diff_a_max; }
    double rel_lat_thresh() const { return rel_lat_thresh; }
    double xica_w_refline() const { return xica_w_refline; }

};

class TaskConfig {
public:
    std::string task_type;
    XICABehaviorDeciderConfig xica_config;

    // 模拟 protobuf getter
    XICABehaviorDeciderConfig x_ica_behavior_decider_config() const {
        return xica_config;
    }
};

// 车辆配置
struct VehConfig {
    double length;
    double width;
    double max_vel;
    double max_lat_acc;
    double max_ddkappa;
    double max_kappa;
    double max_acc;
    double min_acc;
    int max_delta_l;
};

// reward 信息结构体
struct RewardInfo {
    double w_acc;
    double w_ref;
    double w_eff;
    double w_safe;
    double w_pred;
    double w_cons_act;
    double w_cons_his;
    double invalid_penalty;
};

struct XICARewardInfo : public RewardInfo {
    double xica_w_eff = 0.3;
    double xica_w_acc = 0.2;
    double xica_w_safe = 1.0;
    double xica_w_occ = 1.0;
    double xica_w_cons_his = 0.5;
    double xica_w_refline = 1.0;
};


// IDM 参数
struct XICAIDMParam {
    double max_acc_;
    double comfort_acc_;
    double acc_exp_;
    double idm_min_dist_;
    double idm_desired_time_;
    double idmepsilon;
};

// MCTS 参数
struct XICAMCTSParam : public MCTSParam {
    XICARewardInfo xica_reward_info;
    XICAIDMParam xica_idm_param;
    double ego_agent_reward_adjust = 1.0;
    bool use_ref_pre_construct = true;
    bool xica_need_preconstruct = true;
    bool xica_need_ego_idm = true;
    double occ_bound_max = -1.0;
    double ego_target_speed = 10.0;
    double min_decision_delta_t = 0.5;
    double xica_diff_dis_max = 5.0;
    double xica_diff_v_max = 3.0;
    double xica_diff_a_max = 2.0;
    double xica_max_d_theta = 0.8;
    std::unordered_map<std::string, apollo::BehaviorPlanner::XICAObsType> obs_if_cut_in_config;
    std::vector<VehicleAction> ego_lat_action;
    std::vector<std::unordered_map<std::string, VehicleAction>> ego_agent_action;
};





} // namespace BehaviorPlanner
} // namespace apollo