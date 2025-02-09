#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <unordered_map>

#include "vehicle_state.h"

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
  std::unordered_map<std::string, apollo::prediction::PredictionObstacle> pred_obs;
  std::unordered_map<std::string, DiscretizedPath> obs_path;
  std::unordered_map<std::string, apollo::common::CurveBase> obs_path_frenet;
  std::unordered_map<std::string, std::shared_ptr<ReferenceLineInfo>> obs_refline_info;
  std::unordered_map<std::string, std::vector<common::SLPoint>> obs_ref_frenet_project;
  std::unordered_map<std::string, double> obs_target_speed;

  std::vector<TrajectoryPoint> ego_traj_points;
  std::unordered_map<std::string, prediction::Trajectory> last_modified_trajectories;
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


} // namespace BehaviorPlanner
