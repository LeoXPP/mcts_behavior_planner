#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <random>

#include "../tree_node/tree_node.h"
#include "../vehicle_state/vehicle_state.h"
#include "../common/vec2d.h"


namespace apollo{
namespace BehaviorPlanner {

constexpr float kEpsilon = 1e-8f;
constexpr float kMaxStopTime = 100.f;
constexpr float kMinStopTime = -100.f;

enum DecisionType { EgoSearch = 0, EgoPlanning = 1, ObsSearch = 2, ObsPrediction = 3, EgoIDM = 4};


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

struct MCTSParam {
  int max_search_iter = 0;
  double max_search_time = 0.0;
  double c = 0.0;
  double gamma = 0.0;
  int pool_size = 0;
  int action_size = 0;
  std::vector<double> time_step;
  int max_iter = 0;
  std::unordered_map<std::string, DecisionType> decision_type;
  std::unordered_map<std::string, PredictionObstacle> pred_obs;
  std::unordered_map<std::string, DiscretizedPath> obs_path;
  std::unordered_map<std::string, apollo::common::CurveBase> obs_path_frenet;
  std::unordered_map<std::string, std::shared_ptr<ReferenceLineInfo>> obs_refline_info;
  std::unordered_map<std::string, std::vector<common::SLPoint>> obs_ref_frenet_project;
  std::unordered_map<std::string, double> obs_target_speed;

  std::vector<TrajectoryPoint> ego_traj_points;
  std::unordered_map<std::string, Trajectory> last_modified_trajectories;
  std::unordered_map<std::string, double> vel_limit;
  VehConfig veh_param;
  RewardInfo reward_info;
  double left_turn_range = 0.0;
  double heading_range = 0.0;
  double lk_lat_range = 0.0;
  double lat_expand_factor = 0.0;
  double long_expand_factor = 0.0;
  double min_decision_distance = 0.0;
  double max_reward_distance = 0.0;
  int min_valid_node_num = 0;
  std::vector<VehicleAction> pred_action;
  std::vector<VehicleAction> norm_action;
  std::vector<VehicleAction> lk_action;
  std::vector<std::unordered_map<std::string, VehicleAction>> joint_action;
};

class BehaviorMCTSFunctionBase {
 public:
  BehaviorMCTSFunctionBase(const MCTSParam &mcts_param) : mcts_param_(mcts_param) {
    std::random_device rd;
    gen_ = std::mt19937(rd());
  };
  virtual ~BehaviorMCTSFunctionBase() = default;

 public:
  virtual void PreConstructTree(MCTSNode *root, bool is_pre_construct = false) = 0;
  virtual void ChooseAction(MCTSNode *node, std::unordered_map<std::string, VehicleAction> &selected_action) = 0;
  virtual double RewardFun(MCTSNode *node) = 0;
  virtual bool StateChange(MCTSNode *node, const std::unordered_map<std::string, VehicleAction> &action,
                           MCTSNode *new_node) = 0;
  virtual void Prepuring(MCTSNode *new_node) = 0;
  virtual bool ExpandNode(MCTSNode *node, MCTSNode *new_node) = 0;
  virtual void InitNode(MCTSNode *parent_node, const std::unordered_map<std::string, VehicleAction> &action,
                        MCTSNode *new_node) = 0;
  // virtual void InitWorldView(WorldViewPtr world_view) { UNUSED(world_view); }; 
  // virtual void InitReferenceLineInfo(const ReferenceLineInfoPtr reference_line_info) { UNUSED(reference_line_info);};
 public:
  bool SetSearchEnv(SearchEnvironmentPtr search_env);
  bool JerkModel(const VehicleAction &action, const VehicleState &cur_state, VehicleState &next_state, const double dt,
                 bool check_valid = false);
  bool PredictionModel(const VehicleAction &action, const VehicleState &cur_state, VehicleState &next_state,
                       const double dt, const DiscretizedPath &obs_path, bool check_valid = false);
  inline const MCTSParam &mcts_param() { return mcts_param_; }
  inline SearchEnvironmentPtr search_env() { return search_env_; }
  inline MCTSParam &get_mcts_param() { return mcts_param_; }

 protected:
  void Backpropagate(MCTSNode *node);
  double SafetyReward(const std::unordered_map<std::string, VehicleState> &cur_state);
  double EfficiencyReward(const VehicleState &cur_state);
  double AccelerationReward(const VehicleState &veh_state);
  double ReferenceLineReward(const VehicleState &cur_state, const apollo::common::TrajectoryPoint &ego_traj);
  double PredictionReward(const VehicleState &cur_state, const double cur_t,
                          const apollo::prediction::PredictionObstacle &pred_obs);
  double ActionConsistencyReward(const VehicleAction &veh_action, const VehicleAction &last_veh_action);
  double HistoryConsistencyReward(const VehicleState &veh_state, const double cur_t,
                                  const prediction::Trajectory &last_modified_traj);

 protected:
  MCTSParam mcts_param_;
  std::mt19937 gen_;
  SearchEnvironmentPtr search_env_;
};





} // namespace BehaviorPlanner
} // namespace apollo