#include "mcts_base.h"



namespace apollo{
namespace BehaviorPlanner{



static bool SolveQuadraticEquationForRealRoots(const float a, const float b, const float c, float *root1,
                                               float *root2) {
  if (root1 == nullptr || root2 == nullptr) {
    // ATRACE << "xiepanpan: root1 or root2 is nullptr";
    return false;
  }

  if (std::fabs(a) < 1e-8f) {
    if (std::fabs(b) < 1e-8f) {
      // ATRACE << "xiepanpan: a is zero" << a << "b is not good" << b;
      return false;
    }
    *root1 = -c / b;
    return true;
  }

  float discriminant = b * b - 4.f * a * c;
  if (discriminant < -1e-8f) {
    // ATRACE << "xiepanpan: discriminant is negative" << discriminant;
    return false;
  }

  if (discriminant < 1e-8f) {
    *root1 = -b / (2.f * a);
    return true;
  }

  *root1 = (-b + std::sqrt(discriminant)) / (2.f * a);
  *root2 = (-b - std::sqrt(discriminant)) / (2.f * a);
  return true;
}

bool BehaviorMCTSFunctionBase::SetSearchEnv(SearchEnvironmentPtr search_env) {
  if (search_env == nullptr) {
    // ATRACE << "xiepanpan: search_env is nullptr";
    return false;
  }
  search_env_ = search_env;
  return true;
}

bool BehaviorMCTSFunctionBase::JerkModel(const VehicleAction &action, const VehicleState &cur_state,
                                         VehicleState &next_state, const double dt, bool check_valid) {
  // Bicycle jerk model
  double dt_sq = dt * dt;

  // dkappa
  double new_dkappa = action.dkappa() * get_max_dkappa(cur_state.vel());

  // kappa_1 = kappa_0 + dkappa * dt
  double new_kappa = cur_state.kappa() + new_dkappa * dt;
  // check kappa
  if (check_valid && std::fabs(new_kappa) > mcts_param_.veh_param.max_kappa) {
    // ATRACE << "xiepanpan: new_kappa is out of range" << new_kappa;
    return false;
  }
  next_state.set_kappa(new_kappa);

  // a_1 = a_0 + jerk * dt
  double new_jerk = action.jerk();
  double new_acc = cur_state.acc() + new_jerk * dt;
  // check acc
  if (check_valid && new_acc > mcts_param_.veh_param.max_acc) {
    // ATRACE << "xiepanpan: acc is too large" << new_acc;
    return false;
  }
  if (new_acc < mcts_param_.veh_param.min_acc) {
    new_acc = common::math::Clamp(new_acc, mcts_param_.veh_param.min_acc, mcts_param_.veh_param.max_acc);
    new_jerk = (new_acc - cur_state.acc()) / (dt + kEpsilon);
    // check jerk: last acc is already too large or too small
    if (check_valid && std::fabs(new_jerk) < kEpsilon) {
      // ATRACE << "xiepanpan: jerk is too small" << new_jerk;
      return false;
    }
  }

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2 -> Get stop time for deceleration
  float stop_time = kMaxStopTime;
  float root1_ = kMaxStopTime;
  float root2_ = kMaxStopTime;
  if (cur_state.vel() <= std::fabs(dt * mcts_param_.veh_param.min_acc)) {
    if (std::fabs(cur_state.vel()) < kEpsilon &&
        (cur_state.acc() < -kEpsilon || (std::fabs(cur_state.acc()) < kEpsilon && new_jerk < -kEpsilon))) {
      // Already stopped, but sampled a deceleration action
      // ATRACE << "xiepanpan: Already stopped, but sampled a deceleration";
      return false;
    } else {
      if (SolveQuadraticEquationForRealRoots(0.5f * new_jerk, new_acc, cur_state.vel(), &root1_, &root2_)) {
        for (const float root : {root1_, root2_}) {
          if (root > kEpsilon && root < dt) {
            stop_time = (new_jerk > kEpsilon) ? std::min(root, kMaxStopTime) : std::max(root, kMinStopTime);
          }
        }
      }
    }
  }

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2
  double ds = 0.0;
  double new_vel = 0.0;
  double new_theta = 0.0;
  if (stop_time <= dt + kEpsilon && stop_time >= -kEpsilon) {
    ds = cur_state.vel() * stop_time + 0.5f * cur_state.acc() * stop_time * stop_time +
         0.16667f * new_jerk * stop_time * stop_time * stop_time;
    new_vel = 0.f;
    new_acc = 0.f;
    new_jerk = 0.f;
    new_theta =
        cur_state.theta() + cur_state.kappa() * ds + 0.5f * new_dkappa * cur_state.vel() * stop_time * stop_time;
  } else {
    new_vel = cur_state.vel() + cur_state.acc() * dt + 0.5f * new_jerk * dt_sq;
    float lat_acc = new_vel * new_vel * std::abs(new_kappa);
    // check lat acc
    if (check_valid && (lat_acc > mcts_param_.veh_param.max_lat_acc)) {
      // ATRACE << "xiepanpan: lat acc is out of range" << lat_acc;
      return false;
    }
    ds = cur_state.vel() * dt + 0.5f * cur_state.acc() * dt_sq + 0.16667f * new_jerk * dt * dt_sq;
    new_theta = cur_state.theta() + cur_state.kappa() * ds + 0.5f * new_dkappa * cur_state.vel() * dt_sq;
  }
  // check vel
  if (check_valid && new_vel > mcts_param_.veh_param.max_vel) {
    // ATRACE << "xiepanpan: vel is out of range" << new_vel << "  ,  and the max vel is: " << mcts_param_.veh_param.max_vel;
    return false;
  }
  next_state.set_vel(new_vel);
  next_state.set_acc(new_acc);
  next_state.set_s(cur_state.s() + ds);

  // mid_theta = 1/2 * (theta_0 + theta_1)
  float mid_theta = cur_state.theta() + new_theta;
  mid_theta = common::math::NormalizeAngle(mid_theta * 0.5f);
  next_state.set_theta(common::math::NormalizeAngle(new_theta));

  // x_1 = x_0 + s * cos(theta_mid); y_1 = y_0 + s * sin(theta_mid)
  next_state.set_x(cur_state.x() + std::cos(mid_theta) * ds);
  next_state.set_y(cur_state.y() + std::sin(mid_theta) * ds);
  return true;
}

// Prediction model
bool BehaviorMCTSFunctionBase::PredictionModel(const VehicleAction &action, const VehicleState &cur_state,
                                               VehicleState &next_state, const double dt,
                                               const DiscretizedPath &obs_path, bool check_valid) {
  // Longitude jerk model
  double dt_sq = dt * dt;

  // a_1 = a_0 + jerk * dt
  double new_jerk = action.jerk();
  double new_acc = cur_state.acc() + new_jerk * dt;
  // check acc
  if (check_valid && new_acc > mcts_param_.veh_param.max_acc) {
    // ATRACE << "xiepanpan: acc is out of range" << new_acc;
    return false;
  }
  if (new_acc < mcts_param_.veh_param.min_acc) {
    new_acc = common::math::Clamp(new_acc, mcts_param_.veh_param.min_acc, mcts_param_.veh_param.max_acc);
    new_jerk = (new_acc - cur_state.acc()) / (dt + kEpsilon);
    // check jerk: last acc is already too large or too small
    if (check_valid && std::fabs(new_jerk) < kEpsilon) {
      // ATRACE << "xiepanpan: jerk is out of range" << new_jerk;
      return false;
    }
  }

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2 -> Get stop time for deceleration
  float stop_time = kMaxStopTime;
  float root1_ = kMaxStopTime;
  float root2_ = kMaxStopTime;
  if (cur_state.vel() <= std::fabs(dt * mcts_param_.veh_param.min_acc)) {
    if (std::fabs(cur_state.vel()) < kEpsilon &&
        (cur_state.acc() < -kEpsilon || (std::fabs(cur_state.acc()) < kEpsilon && new_jerk < -kEpsilon))) {
      // Already stopped, but sampled a deceleration action
      // ATRACE << "xiepanpan: stop time is out of range" << "cur_state.vel()=" << cur_state.vel() << ", new_acc=" << new_acc << ", new_jerk=" << new_jerk;
      return false;
    } else {
      if (SolveQuadraticEquationForRealRoots(0.5f * new_jerk, new_acc, cur_state.vel(), &root1_, &root2_)) {
        for (const float root : {root1_, root2_}) {
          if (root > kEpsilon && root < dt) {
            stop_time = (new_jerk > kEpsilon) ? std::min(root, kMaxStopTime) : std::max(root, kMinStopTime);
          }
        }
      }
    }
  }

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2
  double ds = 0.0;
  double new_vel = 0.0;
  if (stop_time <= dt + kEpsilon && stop_time >= -kEpsilon) {
    ds = cur_state.vel() * stop_time + 0.5f * cur_state.acc() * stop_time * stop_time +
         0.16667f * new_jerk * stop_time * stop_time * stop_time;
    new_vel = 0.f;
    new_acc = 0.f;
    new_jerk = 0.f;
  } else {
    new_vel = cur_state.vel() + cur_state.acc() * dt + 0.5f * new_jerk * dt_sq;
    ds = cur_state.vel() * dt + 0.5f * cur_state.acc() * dt_sq + 0.16667f * new_jerk * dt * dt_sq;
  }
  // check vel
  if (check_valid && new_vel > mcts_param_.veh_param.max_vel) {
    // ATRACE << "xiepanpan: vel is out of range" << new_vel;
    return false;
  }
  next_state.set_vel(new_vel);
  next_state.set_acc(new_acc);
  next_state.set_s(cur_state.s() + ds);
  apollo::common::PathPoint path_point = obs_path.Evaluate(next_state.s());

  // position
  next_state.set_x(path_point.x());
  next_state.set_y(path_point.y());
  next_state.set_theta(path_point.theta());
  next_state.set_kappa(path_point.kappa());

  return true;
}

// Safety reward
double BehaviorMCTSFunctionBase::SafetyReward(const std::unordered_map<std::string, VehicleState> &cur_state) {
  double dis_lat = mcts_param_.max_reward_distance;
  auto &ego_state = cur_state.at("ego");
  for (auto &[id, other_state] : cur_state) {
    if (mcts_param_.decision_type.at(id) != DecisionType::ObsSearch) {
      continue;
    }
    double dx = ego_state.x() - other_state.x();
    double dy = ego_state.y() - other_state.y();
    dis_lat = std::min(std::fabs(dx * std::sin(-ego_state.theta()) - dy * std::cos(-ego_state.theta())), dis_lat);
  }
  if (dis_lat >= mcts_param_.max_reward_distance) {
    return 1.0;
  } else {
    ATRACE << "XIEPANPAN: SafetyReward" << std::min(dis_lat, mcts_param_.max_reward_distance) / mcts_param_.max_reward_distance;
    return std::min(dis_lat, mcts_param_.max_reward_distance) / mcts_param_.max_reward_distance;
  }
}

// Efficiency reward
double BehaviorMCTSFunctionBase::EfficiencyReward(const VehicleState &veh_state) {
  double vel_prefect = mcts_param_.veh_param.max_vel;
  return std::min(veh_state.vel(), vel_prefect) / (vel_prefect + kEpsilon);
}

// Acceleration reward
double BehaviorMCTSFunctionBase::AccelerationReward(const VehicleState &veh_state) {
  double lat_acc = veh_state.vel() * veh_state.vel() * veh_state.kappa();
  return 1 - 0.5 * std::fabs(veh_state.acc()) / (mcts_param_.veh_param.max_acc + kEpsilon) -
         0.5 * std::fabs(lat_acc) / (mcts_param_.veh_param.max_lat_acc + kEpsilon);
}

// Prediction Trajectory reward
double BehaviorMCTSFunctionBase::PredictionReward(const VehicleState &veh_state, const double cur_t,
                                                  const apollo::prediction::PredictionObstacle &pred_obs) {
  double dis = mcts_param_.max_reward_distance;
  int point_idx = std::min(static_cast<int>(cur_t / 0.1), pred_obs.trajectory()[0].trajectory_point().size() - 1);
  auto &traj_point = pred_obs.trajectory()[0].trajectory_point()[point_idx];
  double target_x = traj_point.path_point().x();
  double target_y = traj_point.path_point().y();
  dis = std::min(std::hypot(veh_state.x() - target_x, veh_state.y() - target_y), dis);
  return 1 - dis / (mcts_param_.max_reward_distance + kEpsilon);
}

// Reference line reward
double BehaviorMCTSFunctionBase::ReferenceLineReward(const VehicleState &veh_state,
                                                     const apollo::common::TrajectoryPoint &ego_traj) {
  double target_x = ego_traj.path_point().x();
  double target_y = ego_traj.path_point().y();
  double dis =
      std::min(std::hypot(veh_state.x() - target_x, veh_state.y() - target_y), mcts_param_.max_reward_distance);
  return 1 - dis / (mcts_param_.max_reward_distance + kEpsilon);
}

// Action Consistency reward
double BehaviorMCTSFunctionBase::ActionConsistencyReward(const VehicleAction &veh_action,
                                                         const VehicleAction &last_veh_action) {
  double max_jerk_diff = 6.0;
  double max_dkappa_diff = 1.0;
  double jerk_diff = std::fabs(veh_action.jerk() - last_veh_action.jerk());
  double dkappa_diff = std::fabs(veh_action.dkappa() - last_veh_action.dkappa());
  return 1 - 0.5 * jerk_diff / (max_jerk_diff + kEpsilon) - 0.5 * dkappa_diff / (max_dkappa_diff + kEpsilon);
}

// History Consistency reward
double BehaviorMCTSFunctionBase::HistoryConsistencyReward(const VehicleState &veh_state, const double cur_t,
                                                          const prediction::Trajectory &last_modified_traj) {
  if (last_modified_traj.trajectory_point().empty()) {
    return 0.0;
  }
  int cur_point_idx = std::min(static_cast<int>(cur_t / 0.2), last_modified_traj.trajectory_point().size() - 1);
  auto obs_traj_t = last_modified_traj.trajectory_point()[cur_point_idx];
  double target_x = obs_traj_t.path_point().x();
  double target_y = obs_traj_t.path_point().y();
  double dis =
      std::min(std::hypot(veh_state.x() - target_x, veh_state.y() - target_y), mcts_param_.max_reward_distance);
  return 1 - dis / (mcts_param_.max_reward_distance + kEpsilon);
}

void BehaviorMCTSFunctionBase::Backpropagate(MCTSNode *node) {
  double G = 0.0;
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
    node->set_static_reward(RewardFun(node));
  }
  G = node->static_reward() + mcts_param_.gamma * child_reward;
  node->Update(G, 1);
  if (node->parent() != nullptr) {
    Backpropagate(node->get_parent());
  }
}

    
} // namespace BehaviorPlanner
} // namespace apollo