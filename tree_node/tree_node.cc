#include "tree_node.h"


namespace apollo{
namespace BehaviorPlanner {

void MCTSNode::AddChild(MCTSNode *child) {
  if (child == nullptr) {
    return;
  }
  child->set_parent(this);
  children_.push_back(child);
}

bool MCTSNode::IsFullyExpanded() const { return expanded_num_ >= max_size_; }

MCTSNode *MCTSNode::SelectBestChild(double c) const {
  if (children_.empty())
    return nullptr;
  else if (children_.size() == 1)
    return children_.at(0);
  else {
    double uct = -100000000000.0, max = -10000000000000.0;
    MCTSNode *argmax = nullptr;
    for (MCTSNode *child : children_) {
      double child_score = child->reward_;
      if (c > 0) {
        uct = child_score + c * sqrt(log(this->visits_) / child->visits_);
      } else {
        uct = child_score;
      }
      if (uct > max) {
        max = uct;
        argmax = child;
      }
    }
    return argmax;
  }
}

void MCTSNode::Update(double G, int number_of_threads) {
  if (std::isnan(G) || std::isinf(G)) {
    // ATRACE << "G is invalid: " << G;
    return;
  }
  visits_ += number_of_threads;
  reward_ = reward_ + 1.0 / visits_ * (G - reward_);
}

void MCTSNode::DebugString() const {
  if (visits_ == 0) {
    std::cout << "Tree not expanded yet";
    return;
  }
  std::cout << "Rewards: " << reward() << ", Visits: " << visits() << ", Size: " << size();
};

} // namespace BehaviorPlanner
} // namespace apollo