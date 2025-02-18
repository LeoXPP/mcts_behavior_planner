
#pragma once
#include "../tree_node/tree_node.h"
#include "../vehicle_state/vehicle_state.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace BehaviorPlanner {

class TreeNodePool {
public:
  // 构造函数：根据 pool_size 预先分配节点，并用 init_state 初始化每个节点
  TreeNodePool(int pool_size,
               const std::unordered_map<std::string, VehicleState> &init_state)
      : pool_size_(pool_size) {
    pool_.reserve(pool_size_);
    // free_nodes_ 用于维护当前可用的节点列表
    for (int i = 0; i < pool_size_; ++i) {
      // 注意：这里调用 MCTSNode 的构造函数，传入初始状态和 nullptr 作为父节点
      MCTSNode *node = new MCTSNode(
          0,
          const_cast<std::unordered_map<std::string, VehicleState> &>(
              init_state),
          nullptr);
      pool_.push_back(node);
      free_nodes_.push_back(node);
    }
  }

  ~TreeNodePool() {
    // 删除所有预先分配的节点
    for (auto node : pool_) {
      delete node;
    }
  }

  // 从节点池中获取一个空闲节点，如果池中没有空闲节点则返回 nullptr
  MCTSNode *GetNode() {
    if (free_nodes_.empty()) {
      return nullptr; // 节点池耗尽，可根据需要选择扩容或者返回错误
    }
    MCTSNode *node = free_nodes_.back();
    free_nodes_.pop_back();
    return node;
  }

  // 新增的GetTreeNode函数，功能与GetNode相同
  MCTSNode *GetTreeNode() {
    return GetNode(); // 直接复用现有GetNode的逻辑
  }

  // 将使用完的节点归还到节点池中
  void ReleaseNode(MCTSNode *node) {
    // 如果需要，可在这里对节点进行重置（清除子节点、状态恢复初始值等）
    free_nodes_.push_back(node);
  }

  // 重置整个节点池：将所有节点标记为空闲状态
  void Reset() {
    free_nodes_.clear();
    for (auto node : pool_) {
      // 如果 MCTSNode 提供 Reset() 方法，可调用 node->Reset();
      free_nodes_.push_back(node);
    }
  }

private:
  int pool_size_;
  std::vector<MCTSNode *> pool_;       // 存储所有预分配的节点
  std::vector<MCTSNode *> free_nodes_; // 当前可用的空闲节点
};

} // namespace BehaviorPlanner
} // namespace apollo
