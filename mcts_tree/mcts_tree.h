#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <queue>
#include <sstream>
#include <vector>

#include "../mcts_base/mcts_base.h"
#include "../tree_node/tree_node_pool.h"
#include <chrono>
#include <iostream>
#include <string>

namespace apollo {
namespace BehaviorPlanner {

using TreeNodePoolPtr = std::shared_ptr<TreeNodePool>;

class MCTSTree {
public:
  MCTSTree() = delete;
  MCTSTree(MCTSNode *root, BehaviorMCTSFunctionBase *mcts_func)
      : root_(root), mcts_func_(mcts_func) {
    if (root == nullptr) {
      std::cout << "Error: Root node cannot be nullptr";
      return;
    }
    if (mcts_func == nullptr) {
      std::cout << "Error: MCTS function cannot be nullptr";
      return;
    }
    tree_node_pool_ = std::make_shared<TreeNodePool>(
        mcts_func_->mcts_param().pool_size, root_->state());
  };
  ~MCTSTree() {
    delete root_;
    root_ = nullptr;
  };

public:
  MCTSNode *Select(double c = 1.41);
  bool UctSearch();
  void DefaultPolicy(MCTSNode *node);
  bool SaveTreeVisualization(const std::string &base_filename,
                             const int &seq_num);
  bool SearchBestStateSeq();
  bool SearchBestNodeSeq();
  inline const MCTSNode *root() const { return root_; }
  inline MCTSNode *get_root() { return root_; }
  inline const std::vector<std::unordered_map<std::string, VehicleState>> &
  best_state_seq() const {
    return best_state_seq_;
  }
  inline const std::vector<MCTSNode *> &best_node_seq() const {
    return best_node_seq_;
  }
  unsigned int size() const { return size_; }

  void DebugString() const;

private:
  bool CheckTerminalNode(MCTSNode *node) const;
  void Backpropagate(MCTSNode *node, int number_of_threads);
  MCTSNode *Expand(MCTSNode *node);
  MCTSNode *Rollout(MCTSNode *node);

private:
  MCTSNode *root_;
  std::vector<std::unordered_map<std::string, VehicleState>> best_state_seq_;
  std::vector<MCTSNode *> best_node_seq_;
  BehaviorMCTSFunctionBase *mcts_func_ = nullptr;
  TreeNodePoolPtr tree_node_pool_ = nullptr;
  int size_ = 1;
  int valid_size_ = 0;
};

} // namespace BehaviorPlanner
} // namespace apollo
