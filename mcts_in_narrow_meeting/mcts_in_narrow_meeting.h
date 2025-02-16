#pragma once

#include "../mcts_base/mcts_base.h"
#include "../tree_node/tree_node.h"
#include "common/vec2d.h"
#include "common/trajectory_point.h"

#include <algorithm>


namespace apollo {
namespace BehaviorPlanner {

enum ObstacleType { OppoLeftTurn = 0, OppoCollide = 1 };

struct XICARewardInfo : public RewardInfo {
  double xica_w_eff = 0.3;
  double xica_w_acc = 0.2;
  double xica_w_safe = 1.0;
  double xica_w_occ = 1.0;
  double xica_w_cons_his = 0.5;
  double xica_w_refline = 1.0;
};

enum XICAObsType { XICACutIn = 0, LaneKeep = 1};

struct XICAIDMParam {
  double max_acc_ = 1.3;
  double comfort_acc_ = 1.0;
  double acc_exp_ = 2.0;
  double idm_min_dist_ = 2.5;
  double idm_desired_time_ = 0.9;
  double idmepsilon = 1e-5;

  // double idm_action_dist_ = 60.0;
  // double idm_sampling_time_ = 0.5;
  // double use_idm_max_time_ = 30.0;
  // double obs_min_acc_ = -3.0;
};
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
  std::unordered_map<std::string, XICAObsType> obs_if_cut_in_config;
  std::vector<VehicleAction> ego_lat_action;
  std::vector<std::unordered_map<std::string, VehicleAction>> ego_agent_action;

};
class XICAMCTSFunction : public BehaviorMCTSFunctionBase {
 public:
  XICAMCTSFunction(const XICAMCTSParam &mcts_param, const ObstacleType &obs_type)
      : BehaviorMCTSFunctionBase(mcts_param), mcts_param_(mcts_param), obs_type_(obs_type){};
  ~XICAMCTSFunction() override = default;
  
  protected:
  const XICAMCTSParam &mcts_param_;


 public:
  void PreConstructTree(MCTSNode *root, bool is_pre_construct = false) override;
  void ChooseAction(MCTSNode *node, std::unordered_map<std::string, VehicleAction> &selected_action) override;
  double RewardFun(MCTSNode *node) override;
  bool StateChange(MCTSNode *node, const std::unordered_map<std::string, VehicleAction> &action,
                   MCTSNode *new_node) override;
  void Prepuring(MCTSNode *new_node) override;
  bool ExpandNode(MCTSNode *node, MCTSNode *new_node) override;
  void InitNode(MCTSNode *parent_node, const std::unordered_map<std::string, VehicleAction> &action,
                MCTSNode *new_node) override;
  
  double OccReward(const VehicleState &next_state);

  double ActionConsistencyReward_(const VehicleAction &veh_action, const VehicleAction &last_veh_action);
  double TargetSpeedRward(const VehicleState &next_state, const std::string &id);
  double AccReward(const VehicleState &veh_state, const std::string &id);
  double XICAEfficiencyReward(const VehicleState &veh_state, const std::string &id);

  bool XICAIDMModel(const VehicleState &cur_state, VehicleState &next_state, double dt,
                    const VehicleState *leader_state);

  bool EgoSearchIDMModel(const VehicleAction &action, const VehicleState &cur_state, VehicleState &next_state,
                         const double dt, const VehicleState *leader_state);

  bool GetLeaderState(const std::unordered_map<std::string, VehicleState> &cur_state, const std::string &id,
                      const VehicleState *&leader_state);

  double XICAIDMVelocityReward(const VehicleState &next_state);
  double SmoothnessReward(const VehicleState &cur_state, const VehicleState &his_state);
  double XICAHistoryConsistencyReward(const VehicleState &veh_state, const double cur_t,
                                      const prediction::Trajectory &last_modified_traj);
  double XICAPredictionReward(const VehicleState &veh_state, const double cur_t,
                              const apollo::prediction::PredictionObstacle &pred_obs);
  double XICAReflineReward(const std::unordered_map<std::string, VehicleState> &cur_state, const std::string &id);

  double XICASafetyReward(const std::unordered_map<std::string, VehicleState> &cur_state);

  double StateConsistencyReward(const VehicleState &cur_state, const double cur_t,
                                const apollo::prediction::PredictionObstacle &pred_obs);

  inline void InitWorldView(WorldViewPtr world_view) { world_view_ = world_view; };
  inline void InitReferenceLineInfo(const ReferenceLineInfoPtr reference_line_info) {
    reference_line_info_ = reference_line_info;
  };

 public:
  bool AccModel(const VehicleAction &action, const VehicleState &cur_state, VehicleState &next_state, const double dt,
                const std::string id);

  bool BoundaryCheck(MCTSNode *node, const VehicleState &next_state, const std::string &id);

  bool XICAJerkModel(MCTSNode *node, const std::string &id, const VehicleAction &action, const VehicleState &cur_state,
                                         VehicleState &next_state, const double dt, bool check_valid);


 private:
  ObstacleType obs_type_;
  WorldViewPtr world_view_ = nullptr;
  ReferenceLineInfoPtr reference_line_info_ = nullptr;
};




} // namespace BehaviorPlanner
} // namespace apollo