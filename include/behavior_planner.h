#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/bp_data_struct.h"
#include "common/trajectory_point.h"
#include "mcts_base/mcts_base.h"
#include "mcts_in_narrow_meeting/mcts_in_narrow_meeting.h"
#include "tree_node/tree_node.h"
#include "vehicle_state/vehicle_state.h"
#include "mcts_tree/mcts_tree.h"


namespace apollo {
namespace BehaviorPlanner {

class Planner {

public:
  ~Planner() {}

  void Init();

  bool LoadParams();

  bool ConstructTestInput();
  
  bool MakeDecision();

  bool BuildGamingInfo();

  bool SetGamingAgentInfo();

  bool UpdateDecisionParams();

  bool ConstructMCTree();

  bool InterpolateResult(bool need_extend = false);

private:
  XICABehaviorDeciderConfig config_;

  // std::shared_ptr<apollo::executor::XICAContext> context_ = nullptr;

  // ReferenceLineInfoPtr reference_line_info_ = nullptr;

  // WorldViewPtr world_view_ = nullptr;

  bool is_first_entry_ = true;

  // MCTS dynamics
  MCTSDynamics mcts_dynamics_;

  std::unordered_set<std::string> opposite_collision_obs_set_;

  std::unordered_map<std::string, PredictionObstacle> pred_cipv_;

  std::unordered_map<std::string, DecisionType> decision_type_;

  std::unordered_map<std::string, PredictionObstacle> pred_obs_;

  DiscretizedTrajectory ego_traj_;

  XICAMCTSParam mcts_param_;

  BehaviorMCTSFunctionBase *mcts_func_ = nullptr;

  std::shared_ptr<MCTSTree> mcts_tree_;

  std::vector<MCTSNode *> decision_res_;

  std::unordered_map<std::string, Trajectory *> modified_trajectories_;

  // planning::MCTSGamingInfoSet *debug_info_set_ = nullptr;

  // for choose gaming obs
  // std::vector<const ObstacleInfo *> left_neighbor_obstacles_;

  // std::vector<const ObstacleInfo *> right_neighbor_obstacles_;

  // std::vector<const ObstacleInfo *> current_lane_obstacles_;

  // std::vector<const ObstacleInfo *> nudge_obstacles_;

  // ObstacleInfo *nearest_cipv_obstacle_ = nullptr;

  std::unordered_set<std::string> roi_dynamic_obs_list_;

  double occ_vaild_dis_ = 0.0;
};

} // namespace BehaviorPlanner
} // namespace apollo