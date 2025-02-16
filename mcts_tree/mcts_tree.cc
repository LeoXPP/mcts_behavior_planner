#include "mcts_tree/mcts_tree.h"

namespace apollo {
namespace BehaviorPlanner {

// Terminal(leaf) node: max_iter node or invalid node
bool MCTSTree::CheckTerminalNode(MCTSNode *node) const {
  if (node->iter() == mcts_func_->mcts_param().max_iter) {
    return true;
  }
  if (!node->is_valid()) {
    return true;
  }
  return false;
}

MCTSNode *MCTSTree::Select(double c) {
  MCTSNode *node = root_;
  while (node && !CheckTerminalNode(node)) {
    if (node->IsFullyExpanded()) {
      node = node->SelectBestChild(c);
    } else {
      return node;
    }
  }
  return node;
}

bool MCTSTree::UctSearch() {
  MCTSNode *node = nullptr;
  double duration = 0.0;
  auto start_t = time::Time::Now();
  int max_node_iter = 0;
  int iter_num = 0;
  for (int iter = 0; iter < mcts_func_->mcts_param().max_search_iter; iter++) {
    iter_num++;
    node = Select(); // return a node to expand / a leaf node
    // iter_num = iter;
    if (node->iter() > max_node_iter) {
      max_node_iter = node->iter();
    }
    if (!node) {
      AERROR << "Error: Select node is nullptr";
      return false;
    }

    if (CheckTerminalNode(node)) {
      DefaultPolicy(node);
    } else {
      MCTSNode *new_node = Expand(node);
      DefaultPolicy(new_node);
    }

    duration = (time::Time::Now() - start_t).toSec() * 1000.0;

    // Early stop
    if (duration > mcts_func_->mcts_param().max_search_time) {
      AWARN << "Behavior search " << (iter + 1) << " iters in " << duration
            << " ms.";
      break;
    }
    if (node->visits() / mcts_func_->mcts_param().max_search_iter > 0.5) {
      ATRACE << "Early stopping: Search converged to one node";
      break;
    }
    // if (iter > 1000 && iter / size_ > 2.0) {
    //   ATRACE << "Early stopping: Search converged to certain branch";
    //   break;
    // }
  }
  ATRACE << "xiepanpan: iter_num is" << iter_num;
  if (root_->size() == 0) {
    ATRACE << "No valid action found";
    return false;
  }
  if (valid_size_ < mcts_func_->mcts_param().min_valid_node_num) {
    // ATRACE << "xiepanpan: max_node_iter in all node is: " << max_node_iter;
    // ATRACE << "xiepanpan: Unreasonable valid node size: " << valid_size_;
    return false;
  }
  return true;
}

void MCTSTree::DefaultPolicy(MCTSNode *node) {
  if (node == nullptr)
    return;
  MCTSNode *terminal_node = Rollout(node);
  Backpropagate(terminal_node, 1);
}

void MCTSTree::Backpropagate(MCTSNode *node, int number_of_threads) {
  double G = 0.0;
  // teminal node
  if (CheckTerminalNode(node)) {
    G = 2 * mcts_func_->RewardFun(node);
  } else {
    double child_reward = 0.0;
    int count = 0;
    for (MCTSNode *child : node->children()) {
      child_reward += child->reward();
      count++;
    }
    if (count > 0) {
      child_reward /= count;
    }
    // Root node
    if (node->parent() == nullptr) {
      G = child_reward;
    }
    // Other nodes
    else if (node->visits() == 0) {
      node->set_static_reward(mcts_func_->RewardFun(node));
    }
    G = node->static_reward() + mcts_func_->mcts_param().gamma * child_reward;
  }
  node->Update(G, number_of_threads);
  if (node->parent() != nullptr) {
    Backpropagate(node->get_parent(), number_of_threads);
  }
}

MCTSNode *MCTSTree::Expand(MCTSNode *node) {
  if (node->IsFullyExpanded()) {
    AERROR << "Warning: Cannot expand this node any more!";
    return nullptr;
  }
  MCTSNode *new_node = tree_node_pool_->GetTreeNode();
  if (mcts_func_->ExpandNode(node, new_node)) {
    // Check validality
    mcts_func_->Prepuring(new_node);
    node->AddChild(new_node);
    size_++;
    if (new_node->is_valid()) {
      valid_size_++;
    }
    return new_node;
  }
  return nullptr;
}

MCTSNode *MCTSTree::Rollout(MCTSNode *node) {
  MCTSNode *cur_node = node;
  MCTSNode *temp_node = cur_node;
  // ensure the node is not null for backpropagate
  while (temp_node && !CheckTerminalNode(temp_node)) {
    temp_node = Expand(cur_node);
    if (temp_node) {
      cur_node = temp_node;
    }
  }
  return cur_node;
}

std::string GetCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  auto ms_part = std::chrono::duration_cast<std::chrono::milliseconds>(
                     now.time_since_epoch()) %
                 1000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << "_"
     << std::setfill('0') << std::setw(3) << ms_part.count();
  return ss.str();
}

bool MCTSTree::SaveTreeVisualization(const std::string &base_filename,
                                     const int &seq_num) {
  try {
    nlohmann::json j;
    j["nodes"] = nlohmann::json::array();
    if (!root_) {
      AWARN << "Root is null. Cannot save visualization.";
      return false;
    }
    // BFS traversal
    std::queue<std::pair<MCTSNode *, int>> q;
    q.push({root_, 0});
    int max_depth = 0;
    while (!q.empty()) {
      auto [node, depth] = q.front();
      q.pop();

      if (node == nullptr) {
        AWARN << "Encountered null node at depth: " << depth;
        continue;
      }

      if (depth > max_depth) {
        max_depth = depth;
      }

      for (const auto &state_pair : node->state()) {
        const std::string &vehicle_id = state_pair.first;
        const VehicleState &state = state_pair.second;

        nlohmann::json node_json;
        node_json["depth"] = depth;
        node_json["vehicle_id"] = vehicle_id;
        node_json["x"] = state.x();
        node_json["y"] = state.y();
        node_json["s"] = state.s();
        node_json["l"] = state.l();
        node_json["theta"] = state.theta();
        node_json["vel"] = state.vel();
        node_json["acc"] = state.acc();
        node_json["kappa"] = state.kappa();
        node_json["reward"] = node->reward();

        node_json["iter"] = node->iter();
        node_json["id"] = node->id();

        auto it = node->vehicle_rewards.find(vehicle_id);
        if (it != node->vehicle_rewards.end()) {
          const VehicleRewardDetails &reward_details = it->second;
          nlohmann::json vehicle_reward_json;
          vehicle_reward_json["xica_efficiency_reward"] =
              reward_details.xica_efficiency_reward;
          vehicle_reward_json["acc_reward"] = reward_details.acc_reward;
          vehicle_reward_json["safety_reward"] = reward_details.safety_reward;
          vehicle_reward_json["idm_velocity_reward"] =
              reward_details.idm_velocity_reward;
          vehicle_reward_json["history_consistency_reward"] =
              reward_details.history_consistency_reward;
          vehicle_reward_json["prediction_reward"] =
              reward_details.prediction_reward;
          vehicle_reward_json["action_consistency_reward"] =
              reward_details.action_consistency_reward;
          vehicle_reward_json["occupancy_reward"] =
              reward_details.occupancy_reward;
          vehicle_reward_json["refline_reward"] = reward_details.refline_reward;
          vehicle_reward_json["state_cons_reward"] =
              reward_details.state_cons_reward;
          node_json["vehicle_rewards"] = vehicle_reward_json;
        } else {
          node_json["vehicle_rewards"] = nlohmann::json::object();
          AWARN << "No reward details found for vehicle_id: " << vehicle_id
                << " in node_id: " << node->id();
        }
        auto is_state = node->vehicle_states.find(vehicle_id);
        if (is_state != node->vehicle_states.end()) {
          const VehicleStateDetails &state_details = is_state->second;
          nlohmann::json vehicle_state_json;
          vehicle_state_json["lat_to_pred"] =
              state_details.lateral_dis_to_prediction;
          vehicle_state_json["dis_to_occ"] = state_details.dis_to_occ;
          vehicle_state_json["new_kappa"] = state_details.new_kappa;
          vehicle_state_json["new_acc"] = state_details.new_acc;
          vehicle_state_json["stop_time"] = state_details.stop_time;
          vehicle_state_json["lat_acc"] = state_details.lat_acc;
          vehicle_state_json["new_vel"] = state_details.new_vel;
          node_json["vehicle_states"] = vehicle_state_json;
        } else {
          node_json["vehicle_states"] = nlohmann::json::object();
          // AWARN << "No state details found for vehicle_id: " << vehicle_id <<
          // " in node_id: " << node->id();
        }

        node_json["visits"] = node->visits();
        node_json["static_reward"] = node->static_reward();
        node_json["size"] = node->size();
        node_json["relative_time"] = node->relative_time();
        node_json["is_valid"] = node->is_valid();
        node_json["max_size"] = node->max_size();
        node_json["expanded_num"] = node->expanded_num();
        j["nodes"].push_back(node_json);
      }

      // in case nullptr
      for (auto child : node->children()) {
        if (child != nullptr) {
          q.push({child, depth + 1});
        } else {
          AWARN << "Encountered null child of node_id: " << node->id();
        }
      }
    }

    std::string timestamp = GetCurrentTimestamp();
    std::string filename = base_filename;
    if (filename.size() >= 5 &&
        filename.substr(filename.size() - 5) == ".json") {
      filename = filename.substr(0, filename.size() - 5);
    }
    std::string seq_num_str = std::to_string(seq_num);
    filename += "_" + seq_num_str + "_" + timestamp + ".json";
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
      ATRACE << "Failed to open file: " << filename;
      return false;
    }
    ofs << j.dump(4);
    ofs.close();
    ATRACE << "MCTS tree visualization saved to " << filename;
    return true;
  } catch (const std::exception &e) {
    AWARN << "Exception during SaveTreeVisualization: " << e.what();
    return false;
  }
}

bool MCTSTree::SearchBestStateSeq() {
  best_state_seq_.clear();
  MCTSNode *cur_node = root_;
  while (cur_node && !CheckTerminalNode(cur_node)) {
    best_state_seq_.push_back(cur_node->state());
    cur_node = cur_node->SelectBestChild(0.0);
    cur_node->set_optimal(true);
    if (!cur_node->is_valid()) {
      AWARN << "Invalid node found in best action sequence";
      return false;
    }
  }
  if (cur_node) {
    best_state_seq_.push_back(cur_node->state());
  }
  int result_size = best_node_seq_.size();
  if (result_size < mcts_func_->mcts_param().max_iter) {
    AWARN << "No valid action sequence found";
    return false;
  }
  return true;
}

bool MCTSTree::SearchBestNodeSeq() {
  best_node_seq_.clear();
  MCTSNode *cur_node = root_;
  while (cur_node && !CheckTerminalNode(cur_node)) {
    best_node_seq_.push_back(cur_node);
    cur_node = cur_node->SelectBestChild(0.0);
    cur_node->set_optimal(true);
    if (!cur_node->is_valid()) {
      AWARN << "Invalid node found in best action sequence";
      return false;
    }
  }
  if (cur_node) {
    best_node_seq_.push_back(cur_node);
  }
  int result_size = best_node_seq_.size();
  if (result_size < mcts_func_->mcts_param().max_iter) {
    AWARN << "No valid action sequence found";
    return false;
  }
  return true;
}

void MCTSTree::DebugString() const {
  // ATRACE << "Tree size: " << size();
  ATRACE << "Valid node size: " << valid_size_;
  root_->DebugString();
}

} // namespace BehaviorPlanner
} // namespace apollo