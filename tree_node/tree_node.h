#pragma once

#include "../vehicle_state/vehicle_state.h"
#include <cassert>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>


namespace apollo {
namespace BehaviorPlanner {
enum NodeType { NORM = 0, ACC = 1, DEC = 2, PRED = 3, NEWPRED = 4 };
struct VehicleRewardDetails {
  double xica_efficiency_reward = 0.0;
  double acc_reward = 0.0;
  double safety_reward = 0.0;
  double idm_velocity_reward = 0.0;
  double history_consistency_reward = 0.0;
  double prediction_reward = 0.0;
  double action_consistency_reward = 0.0;
  double occupancy_reward = 0.0;
  double refline_reward = 0.0;
  double state_cons_reward = 0.0;
  // Add additional reward components as needed

  // Optional: Method to calculate total reward
  double total_reward() const {
    return xica_efficiency_reward + acc_reward + safety_reward +
           idm_velocity_reward + history_consistency_reward +
           prediction_reward + action_consistency_reward + occupancy_reward +
           refline_reward + state_cons_reward;
  }
};

struct VehicleStateDetails {
  double lateral_dis_to_prediction = 0.0;
  float dis_to_occ = 0.0;
  double new_kappa = 0.0;
  double new_acc = 0.0;
  double stop_time = 0.0;
  float lat_acc = 0.0;
  double new_vel = 0.0;
};

class MCTSNode {
public:
  MCTSNode() = default;
  MCTSNode(int iter, std::unordered_map<std::string, VehicleState> &state,
           MCTSNode *parent = nullptr)
      : iter_(iter), cur_state_(state), parent_(parent) {
    visits_ = 0;
    reward_ = 0.0;
    static_reward_ = 0.0;
  };
  ~MCTSNode() = default;

  std::unordered_map<std::string, VehicleRewardDetails> vehicle_rewards;
  std::unordered_map<std::string, VehicleStateDetails> vehicle_states;

public:
  void AddChild(MCTSNode *child);
  bool IsFullyExpanded() const;
  MCTSNode *SelectBestChild(double c) const;
  void Update(double G, int number_of_threads);

  inline int iter() const {
    // assert(this != nullptr && "MCTSNode object is null");
    if(this == nullptr) {
      std::cout << "MCTSNode object is null" << std::endl;
      return 0;
    }
    return iter_;
  }

  inline const std::string id() const { return id_; }
  inline const MCTSNode *parent() const { return parent_; }
  inline const std::vector<MCTSNode *> &children() const { return children_; }
  inline const std::unordered_map<std::string, VehicleState> &state() const {
    return cur_state_;
  }
  inline const std::unordered_map<std::string, VehicleAction> &
  selected_action() const {
    return selected_action_;
  }
  inline const std::unordered_map<std::string, VehicleReward> &
  agent_reward() const {
    return agent_reward_;
  }
  inline double visits() const { return visits_; }
  inline double reward() const { return reward_; }
  inline double static_reward() const { return static_reward_; }
  inline int size() const { return children_.size(); }
  inline double relative_time() const { return relative_time_; }
  inline bool is_valid() const { return is_valid_; }
  inline int max_size() const { return max_size_; }
  inline int expanded_num() const { return expanded_num_; }
  inline NodeType node_type() const { return node_type_; }
  inline int untried_actions_idx() { return untried_actions_idx_; }
  inline bool is_optimal_node() const { return is_optimal_node_; }

  inline std::unordered_map<std::string, VehicleState> &get_state() {
    return cur_state_;
  }
  inline std::unordered_map<std::string, VehicleReward> &get_agent_reward() {
    return agent_reward_;
  }
  inline MCTSNode *get_parent() { return parent_; }

  inline void set_id(const std::string &id) { id_ = id; }
  inline void set_iter(int iter) { iter_ = iter; }
  inline void
  set_state(const std::unordered_map<std::string, VehicleState> &state) {
    cur_state_ = state;
  }
  inline void set_parent(MCTSNode *parent) { parent_ = parent; }
  inline void
  set_action(const std::unordered_map<std::string, VehicleAction> &action) {
    selected_action_ = action;
  }
  inline void set_static_reward(double reward) { static_reward_ = reward; }
  inline void set_relative_time(double relative_time) {
    relative_time_ = relative_time;
  }
  inline void set_valid(bool is_valid) { is_valid_ = is_valid; }
  inline void set_reward(double reward) { reward_ = reward; }
  inline void set_visits(unsigned int visits) { visits_ = visits; }
  inline void set_max_size(int max_size) { max_size_ = max_size; }
  inline void set_expanded_num(int expanded_num) {
    expanded_num_ = expanded_num;
  }
  inline void set_node_type(NodeType node_type) { node_type_ = node_type; }
  inline void set_untried_actions_idx(int untried_actions_idx) {
    untried_actions_idx_ = untried_actions_idx;
  }
  inline void set_agent_reward_by_id(const std::string &id,
                                     const VehicleReward &reward) {
    agent_reward_[id] = reward;
  }
  inline void set_optimal(bool is_optimal) { is_optimal_node_ = is_optimal; }

  void DebugString() const;

protected:
  std::string id_ = "0";
  int iter_ = 0;
  std::unordered_map<std::string, VehicleState> cur_state_;
  MCTSNode *parent_;
  std::vector<MCTSNode *> children_;
  std::unordered_map<std::string, VehicleAction> selected_action_;
  std::unordered_map<std::string, VehicleReward> agent_reward_;
  int untried_actions_idx_ = 0;
  unsigned int visits_ = 0;
  double reward_ = 0.0;
  double static_reward_ = 0.0;
  double relative_time_ = 0.0;
  bool is_valid_ = true;
  bool is_optimal_node_ = false;
  int max_size_ = 0;
  int expanded_num_ = 0;
  NodeType node_type_ = NodeType::NORM;
};

} // namespace BehaviorPlanner
} // namespace apollo