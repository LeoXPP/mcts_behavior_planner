#include "mcts_in_narrow_meeting.h"
#include <limits>


namespace apollo {
namespace BehaviorPlanner {

static bool SolveQuadraticEquationForRealRoots(const float a, const float b, const float c, float *root1,
                                               float *root2) {
  if (root1 == nullptr || root2 == nullptr) {
    // // ATRACE << "xiepanpan: root1 or root2 is nullptr";
    return false;
  }

  if (std::fabs(a) < 1e-8f) {
    if (std::fabs(b) < 1e-8f) {
      // // ATRACE << "xiepanpan: a is zero" << a << "b is not good" << b;
      return false;
    }
    *root1 = -c / b;
    return true;
  }

  float discriminant = b * b - 4.f * a * c;
  if (discriminant < -1e-8f) {
    // // ATRACE << "xiepanpan: discriminant is negative" << discriminant;
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

// Initial setting for a new node (NOT for root)
void XICAMCTSFunction::InitNode(MCTSNode *parent_node, const std::unordered_map<std::string, VehicleAction> &action,
                                   MCTSNode *new_node) {
  if (!parent_node || !new_node) {
    // AERROR << "Root node cannot be initialized.";
    return;
  }
  // Set max size according to node type
  new_node->set_node_type(NodeType::NORM);
  // new_node->set_max_size(mcts_param_.norm_action.size());
  new_node->set_max_size(mcts_param_.ego_agent_action.size());
  // Set untried action index according to max size
  std::uniform_int_distribution<> dis(0, new_node->max_size() - 1);
  new_node->set_untried_actions_idx(dis(gen_));
  // Update iter and relative time
  new_node->set_iter(parent_node->iter() + 1);
  new_node->set_relative_time(parent_node->relative_time() + mcts_param_.time_step[parent_node->iter()]);
  new_node->set_action(action);
}

bool XICAMCTSFunction::ExpandNode(MCTSNode *node, MCTSNode *new_node) {
  if (!node || !new_node) {
    // // AERROR << "Node is null, cannot be expanded.";
    return false;
  }
  std::unordered_map<std::string, VehicleAction> selected_action;
  while (!node->IsFullyExpanded()) {
    ChooseAction(node, selected_action);
    if (!StateChange(node, selected_action, new_node)) {
      continue;
    }
    return true;
  }
  if (node->IsFullyExpanded() && node->size() == 0) {
    node->set_reward(-999.0);
    node->set_valid(false);
  }
  return false;
}

void XICAMCTSFunction::ChooseAction(MCTSNode *node, std::unordered_map<std::string, VehicleAction> &selected_action) {
  if (!node) {
    // // AERROR << "Node is null, cannot choose action.";
    return;
  }
  int untried_actions_idx = node->untried_actions_idx();
  // for (auto &[id, decision_type] : mcts_param_.decision_type) {
  //   if (decision_type == DecisionType::ObsSearch) {
  //     selected_action[id] = mcts_param_.ego_agent_action.at(untried_actions_idx);
  //   }
  //   if (decision_type == DecisionType::EgoSearch) {
  //     // int ego_lat_max_size = mcts_param_.ego_lat_action.size();
  //     selected_action[id] = mcts_param_.ego_agent_action.at(untried_actions_idx);
  //   }
  // }    

  selected_action = mcts_param_.ego_agent_action.at(untried_actions_idx);
  // Update untried actions index
  untried_actions_idx = (untried_actions_idx + 1) % node->max_size();
  node->set_untried_actions_idx(untried_actions_idx);
  // Increment expanded num
  node->set_expanded_num(node->expanded_num() + 1);
}

bool XICAMCTSFunction::StateChange(MCTSNode *node, const std::unordered_map<std::string, VehicleAction> &action,
                                      MCTSNode *new_node) {
  if (!node || !new_node) {
    // AERROR << "Node is null, cannot change state.";
    return false;
  }
  auto &cur_state = node->state();
  auto &next_state = new_node->get_state();
  node->vehicle_states.clear();
  for (auto &[id, decision_type] : mcts_param_.decision_type) {
    if (next_state.find(id) == next_state.end()) {
      // AERROR << "Vehicle state not found, id: " << id;
      return false;
    }
    // Search
    if (decision_type == DecisionType::EgoSearch || decision_type == DecisionType::ObsSearch) {
      bool is_action_chosen = false;
      if (action.find(id) == action.end()) {
        // AERROR << "Action not found, id: " << id;
        return false;
      }
      if (mcts_param_.decision_type.at("ego") == DecisionType::EgoSearch) {
        // ATRACE << "xiepanpan: Begin to EgoSearch.";
        const VehicleState *leader_state = nullptr;
        if(!GetLeaderState(cur_state, id, leader_state)){
          // AERROR << "xiepanpan: GetLeaderState failed";
          return false;
        }
        // if(node->iter() == 0){IDM or IMD+dkappa}
        is_action_chosen = EgoSearchIDMModel(action.at(id), cur_state.at(id), next_state.at(id),
                                             mcts_param_.time_step[node->iter()], leader_state);
                          //  BoundaryCheck(node, next_state.at(id), id);
      } else {
        is_action_chosen =
            XICAJerkModel(node, id, action.at(id), cur_state.at(id), next_state.at(id), mcts_param_.time_step[node->iter()], true) &&
            BoundaryCheck(node, next_state.at(id), id);
      }
      if (!is_action_chosen) {
        return false;
      }
    }
    // Ego planning
    else if (decision_type == DecisionType::EgoPlanning) {
      int ego_traj_idx = std::min(node->iter() + 1, static_cast<int>(mcts_param_.ego_traj_points.size() - 1));
      const auto &ego_traj = mcts_param_.ego_traj_points.at(ego_traj_idx);
      next_state.at(id).set_x(ego_traj.path_point().x());
      next_state.at(id).set_y(ego_traj.path_point().y());
      next_state.at(id).set_s(ego_traj.path_point().s());
      next_state.at(id).set_theta(ego_traj.path_point().theta());
      next_state.at(id).set_vel(ego_traj.v());
      next_state.at(id).set_acc(ego_traj.a());
      next_state.at(id).set_kappa(ego_traj.path_point().kappa());
    }
    // Ego planning with IDM
    else if (decision_type == DecisionType::EgoIDM) {
      const VehicleState *leader_state = nullptr;
      if(!GetLeaderState(cur_state, id, leader_state)){
        // AERROR << "xiepanpan: GetLeaderState failed";
        return false;
      }
      if (!XICAIDMModel(cur_state.at(id), next_state.at(id), mcts_param_.time_step[node->iter()], leader_state)) {
        // AERROR << "xiepanpan: XICAIDMModel failed";
        return false;
      }
    }
    // Obs Prediction
    else if (decision_type == DecisionType::ObsPrediction) {
      int point_idx = static_cast<int>((node->relative_time() + mcts_param_.time_step[node->iter()]) * 10);
      point_idx = std::min(point_idx, mcts_param_.pred_obs.at(id).trajectory()[0].trajectory_point().size() - 1);
      auto &obs_traj = mcts_param_.pred_obs.at(id).trajectory()[0].trajectory_point()[point_idx];
      next_state.at(id).set_x(obs_traj.path_point().x());
      next_state.at(id).set_y(obs_traj.path_point().y());
      next_state.at(id).set_s(obs_traj.path_point().s());
      next_state.at(id).set_theta(obs_traj.path_point().theta());
      next_state.at(id).set_vel(obs_traj.v());
      next_state.at(id).set_acc(obs_traj.a());
      next_state.at(id).set_kappa(obs_traj.path_point().kappa());
    }
  }
  InitNode(node, action, new_node);
  return true;
}

bool XICAMCTSFunction::GetLeaderState(const std::unordered_map<std::string, VehicleState> &cur_state,
                                      const std::string &id, const VehicleState *&leader_state) {
  double min_longitudinal_dist = 100.0;  // Initialize with a max threshold distance
  // std::string leader_id = "";

  // Find the leader vehicle
  for (auto &[id_, decision_type] : mcts_param_.decision_type) {
    if (decision_type == DecisionType::ObsSearch) {
      auto it = cur_state.find(id_);
      if (it == cur_state.end()) {
        // // AERROR << "xiepanpan not found id_ when find nearest agent: " << id_;
        return false;
      }
      const VehicleState &candidate_state = it->second;
      const VehicleState &ego_state = cur_state.at(id);
      apollo::common::math::Vec2d ego_xy(ego_state.x(), ego_state.y());
      apollo::common::SLPoint candidate_sl_point;
      if (mcts_param_.obs_path_frenet.find(id_) == mcts_param_.obs_path_frenet.end()) {
        // // AERROR << "xiepanpan: not found ego id when  ind the leader vehicle" << id;
        return false;
      }
      mcts_param_.obs_path_frenet.at(id_).XYToSL(ego_xy, &candidate_sl_point);

      double longitudinal_dist = std::fabs(candidate_sl_point.s());
      double lateral_dist = std::fabs(candidate_sl_point.l());

      if (lateral_dist < 2.0 && longitudinal_dist < min_longitudinal_dist) {
        min_longitudinal_dist = longitudinal_dist;
        leader_state = &candidate_state;
        // leader_id = id_;
      }
    }
  }
  // No suitable leader was found within 100m and lateral distance of 2m
  if (!leader_state || min_longitudinal_dist > 100.0) {
    // todo: Fallback to default leader
    leader_state = nullptr;
  }
  return true;
}

bool XICAMCTSFunction::XICAIDMModel(const VehicleState &cur_state, VehicleState &next_state, double dt,
                                    const VehicleState *leader_state) {
  XICAIDMParam idm_param = mcts_param_.xica_idm_param;
  double dist = std::numeric_limits<int>::max();
  double dist = std::numeric_limits<double>::max();
  
  if (leader_state) {
    double delta_vel = std::fabs(cur_state.vel() - leader_state->vel());
    safe_dist +=
        std::max(cur_state.vel() * delta_vel / (2.0 * std::sqrt(idm_param.max_acc_ * idm_param.comfort_acc_)), 0.0);
    apollo::common::math::Vec2d leader_xy(leader_state->x(), leader_state->y());
    apollo::common::SLPoint sl_point;
    if(mcts_param_.obs_path_frenet.find("ego") == mcts_param_.obs_path_frenet.end()){
      // // AERROR << "xiepanpan No frenet path found for ego vehicle.";
      return false;
    }
    // // ATRACE << "xiepanpan leader_state found.";
    mcts_param_.obs_path_frenet.at("ego").XYToSL(leader_xy, &sl_point);
    dist = sl_point.s() + idm_param.idmepsilon;  // in case /0
  }

  double ego_target_speed_ = mcts_param_.ego_target_speed;
  if (mcts_param_.ego_target_speed < 1) {
    ego_target_speed_ = 1.0;
    // // AERROR << "xiepanpan Ego target speed is too small reset to " << ego_target_speed_;
  } else {
    ego_target_speed_ = mcts_param_.ego_target_speed;
  }
  double acc = idm_param.max_acc_ * (1.0 - std::pow(cur_state.vel() / ego_target_speed_, idm_param.acc_exp_) -
                                     std::pow(safe_dist / dist, 2));
  acc = std::max(cur_state.acc() - dt * mcts_param_.veh_param.comfort_jerk,
                 std::min(acc, cur_state.acc() + dt * mcts_param_.veh_param.comfort_jerk));
  // in case velocity is negative
  double new_vel = cur_state.vel() + acc * dt;
  if (new_vel < 0) {
    new_vel = 0;
    acc = 0;
  }
  next_state.set_acc(acc);
  next_state.set_vel(cur_state.vel() + acc * dt);
  double new_s = cur_state.s() + cur_state.vel() * dt + 0.5f * acc * dt * dt;
  next_state.set_s(new_s);
  PathPoint path_point = mcts_param_.obs_path.at("ego").Evaluate(next_state.s());
  next_state.set_x(path_point.x());
  next_state.set_y(path_point.y());
  next_state.set_theta(path_point.theta());
  next_state.set_kappa(path_point.kappa());
  return true;
}

void XICAMCTSFunction::PreConstructTree(MCTSNode *root, bool is_pre_construct) {
  if (!is_pre_construct) {
    root->set_node_type(NodeType::NORM);
    // root->set_max_size(mcts_param_.norm_action.size());
    root->set_max_size(mcts_param_.ego_agent_action.size());
    std::uniform_int_distribution<> dis(0, root->max_size() - 1);
    root->set_untried_actions_idx(dis(gen_));
    // // ATRACE << "Finish root init without tree pre-constructed.";
    return;
  }

  // Prediction
  MCTSNode *node = root;
  node->set_node_type(NodeType::PRED);
  while (node->iter() < mcts_param_.max_iter) {
    std::unordered_map<std::string, VehicleState> next_state = node->state();
    std::unordered_map<std::string, VehicleAction> selected_action;

    int ego_traj_idx = std::min(node->iter() + 1, static_cast<int>(mcts_param_.ego_traj_points.size() - 1));
    const auto &ego_traj = mcts_param_.ego_traj_points.at(ego_traj_idx);
    next_state.at("ego").set_x(ego_traj.path_point().x());
    next_state.at("ego").set_y(ego_traj.path_point().y());
    next_state.at("ego").set_s(ego_traj.path_point().s());
    next_state.at("ego").set_theta(ego_traj.path_point().theta());
    next_state.at("ego").set_vel(ego_traj.v());
    next_state.at("ego").set_acc(ego_traj.a());
    next_state.at("ego").set_kappa(ego_traj.path_point().kappa());
    if (!mcts_param_.use_ref_pre_construct) {
      for (auto &[id, obstacle] : mcts_param_.pred_obs) {
        int point_idx = static_cast<int>((node->relative_time() + mcts_param_.time_step[node->iter()]) * 10);
        point_idx = std::min(point_idx, static_cast<int>(obstacle.trajectory()[0].trajectory_point().size() - 1));
        auto &obs_traj = obstacle.trajectory()[0].trajectory_point()[point_idx];
        next_state.at(id).set_x(obs_traj.path_point().x());
        next_state.at(id).set_y(obs_traj.path_point().y());
        next_state.at(id).set_s(obs_traj.path_point().s());
        next_state.at(id).set_theta(obs_traj.path_point().theta());
        next_state.at(id).set_vel(obs_traj.v());
        next_state.at(id).set_acc(obs_traj.a());
        next_state.at(id).set_kappa(obs_traj.path_point().kappa());
        selected_action[id] = VehicleAction(obs_traj.da(), obs_traj.path_point().dkappa());
      }
      // // ATRACE << "xiepanpan: Pre-constructed prediction trajectory.";
    } else {
      // Project obstacle_prediction onto reference line
      for (auto &[id, obstacle] : mcts_param_.pred_obs) {
        if (mcts_param_.obs_refline_info.find(id) == mcts_param_.obs_refline_info.end()) {
          // AERROR << "xiepanpan No refline info found for obstacle: " << id;
          continue;
        }
        const auto &ref_line_info = mcts_param_.obs_refline_info.at(id);
        const auto &ref_line = ref_line_info->reference_line_ptr();

        int point_idx = static_cast<int>((node->relative_time() + mcts_param_.time_step[node->iter()]) * 10);
        point_idx = std::min(point_idx, obstacle.trajectory()[0].trajectory_point().size() - 1);
        auto &obs_traj = obstacle.trajectory()[0].trajectory_point()[point_idx];

        double accumulate_s = 0.0;
        double lateral = 0.0;
        common::math::Vec2d obs_point(obs_traj.path_point().x(), obs_traj.path_point().y());
        if (!ref_line->GetProjection(obs_point, &accumulate_s, &lateral)) {
          // AERROR << "Failed to project obstacle point onto reference line: " << id;
          continue;
        }
        const auto &projected_ref_point = ref_line->GetPathPoint(accumulate_s);
        next_state.at(id).set_x(projected_ref_point.x());
        next_state.at(id).set_y(projected_ref_point.y());
        next_state.at(id).set_s(projected_ref_point.s());
        next_state.at(id).set_theta(projected_ref_point.theta());
        next_state.at(id).set_vel(obs_traj.v());  // TODO(xiepanpan): find better v.
        next_state.at(id).set_acc(obs_traj.a());  // TODO(xiepanpan): find better a.
        next_state.at(id).set_kappa(projected_ref_point.kappa());
        // TODO(xiepanpan): find better da/dkappa.
        selected_action[id] = VehicleAction(obs_traj.da(), projected_ref_point.dkappa());
      }
      // // ATRACE << "xiepanpan: Pre-constructed refline";
    }

    // New node setting
    MCTSNode *new_node = new MCTSNode(node->iter() + 1, next_state);
    InitNode(node, selected_action, new_node);
    new_node->set_node_type(NodeType::PRED);
    Prepuring(new_node);

    // Parent node setting
    node->AddChild(new_node);
    node = new_node;
  }
  Backpropagate(node);
  root->set_max_size(root->max_size() + 1);
  root->set_expanded_num(root->expanded_num() + 1);
  // ATRACE << "xiepanpan: set_max_size is : " << root->max_size();

  // Straight dec action: for opposite left turn only
  if (obs_type_ == ObstacleType::OppoLeftTurn) {
    root->set_node_type(NodeType::NORM);
    for (size_t i = 0; i < mcts_param_.lk_action.size(); i++) {
      if (mcts_param_.lk_action[i].jerk() > 0.f) {
        continue;
      }
      std::unordered_map<std::string, VehicleState> next_state = root->state();
      std::unordered_map<std::string, VehicleAction> selected_action;
      for (auto &[id, decision_type] : mcts_param_.decision_type) {
        if (decision_type == DecisionType::ObsSearch) {
          selected_action[id] = VehicleAction(mcts_param_.lk_action[i]);
        }
      }
      MCTSNode *new_node = new MCTSNode(root->iter() + 1, next_state);
      if (StateChange(root, selected_action, new_node)) {
        InitNode(root, selected_action, new_node);
        Prepuring(new_node);
        // Root setting for pre-construct tree
        root->AddChild(new_node);
        root->set_max_size(root->max_size() + 1);
        root->set_expanded_num(root->expanded_num() + 1);
        Backpropagate(new_node);
      }
    }
    // // ATRACE << "Pre-constructed default trajectory.";
  }
}

void XICAMCTSFunction::Prepuring(MCTSNode *new_node) {
  auto &next_state = new_node->state();
  // // ATRACE << "xiepanpan: beginning to prepruning.";
  for (auto &[id, decision_type] : mcts_param_.decision_type) {
    if (next_state.find(id) == next_state.end()) {
      // AERROR << "State not found, id: " << id;
      return;
    }
    // // ATRACE << "xiepanpan: obstacle id is:" << id << "the decision type is:" <<decision_type;
    if (decision_type == DecisionType::ObsSearch) {
      apollo::common::math::Box2d obs_box({next_state.at(id).x(), next_state.at(id).y()}, next_state.at(id).theta(),
                                          mcts_param_.pred_obs.at(id).perception_obstacle().length(),
                                          mcts_param_.pred_obs.at(id).perception_obstacle().width());
      // Collision check: ego
      double dis = std::hypot(next_state.at(id).x() - next_state.at("ego").x(),
                              next_state.at(id).y() - next_state.at("ego").y());
      if (dis < 10.0) {
        apollo::common::math::Box2d ego_box({next_state.at("ego").x(), next_state.at("ego").y()},
                                            next_state.at("ego").theta(),
                                            mcts_param_.long_expand_factor * mcts_param_.veh_param.length,
                                            mcts_param_.lat_expand_factor * mcts_param_.veh_param.width);
        if (ego_box.HasOverlap(obs_box)) {
          new_node->set_reward(-101.0);
          new_node->set_valid(false);
          // // ATRACE << "xiepanpan: Ego collide with obstacle." << "the obstacle id is:" <<id;
          return;
        }
      }
      // Collision check: CIPV
      for (auto &[cipv_id, cipv_decision_type] : mcts_param_.decision_type) {
        if (cipv_decision_type == DecisionType::ObsPrediction) {
          dis = std::hypot(next_state.at(id).x() - next_state.at(cipv_id).x(),
                           next_state.at(id).y() - next_state.at(cipv_id).y());
          if (dis < 10.0) {
            apollo::common::math::Box2d cipv_box({next_state.at(cipv_id).x(), next_state.at(cipv_id).y()},
                              next_state.at(cipv_id).theta(), 1.2 * mcts_param_.veh_param.length,
                                                 1.2 * mcts_param_.veh_param.width);
            if (obs_box.HasOverlap(cipv_box)) {
              new_node->set_reward(-101.0);
              new_node->set_valid(false);
              // // ATRACE << "xiepanpan: cipv_box Collision.";
              return;
            }
          }
        }
      }
    }
  }
  // // ATRACE << "xiepanpan: after prepruning and node is setted valid";
  new_node->set_valid(true);
  return;
}

bool XICAMCTSFunction::EgoSearchIDMModel(const VehicleAction &action,
                                         const VehicleState &cur_state,
                                         VehicleState &next_state,
                                         double dt,
                                         const VehicleState *leader_state) {
    // Perform longitudinal state transition using the IDM model
    if (!XICAIDMModel(cur_state, next_state, dt, leader_state)) {
        return false;  // Invalid longitudinal action
    }

    // Calculate lateral state transition
    double current_velocity = cur_state.vel();
    double dkappa = action.dkappa() * get_max_dkappa(current_velocity);
    double new_kappa = cur_state.kappa() + dkappa * dt;

    // Validate new kappa against maximum allowed value
    if (std::abs(new_kappa) > mcts_param_.veh_param.max_kappa) {
        // ATRACE << "kappa is out of range: " << new_kappa;
        return false;
    }
    next_state.set_kappa(new_kappa);

    // Calculate change in position (ds) using current velocity and acceleration
    double acceleration = next_state.acc();  // Assuming acc() returns acceleration
    double ds = current_velocity * dt + 0.5 * acceleration * dt * dt;

    // Update theta (heading angle) based on new kappa and dkappa
    double new_theta = cur_state.theta() + cur_state.kappa() * ds + 0.5 * dkappa * current_velocity * dt * dt;
    double mid_theta = common::math::NormalizeAngle((cur_state.theta() + new_theta) / 2.0);

    next_state.set_theta(common::math::NormalizeAngle(new_theta));
    next_state.set_s(cur_state.s() + ds);

    // Safely evaluate the path point for the new s-coordinate
    std::optional<PathPoint> path_point_opt;
    auto path_it = mcts_param_.obs_path.find("ego");
    if (path_it != mcts_param_.obs_path.end()) {
        path_point_opt = path_it->second.Evaluate(next_state.s());
    } else {
        // ATRACE << "Path information for 'ego' not found.";
        return false;
    }

    // Update x and y coordinates based on mid_theta
    next_state.set_x(cur_state.x() + std::cos(mid_theta) * ds);
    next_state.set_y(cur_state.y() + std::sin(mid_theta) * ds);

    // Validate velocity constraints
    double new_velocity = next_state.vel();
    if (new_velocity > mcts_param_.veh_param.max_vel) {
        // ATRACE << "velocity is out of range: " << new_velocity;
        return false;
    }

    return true;
}

bool XICAMCTSFunction::XICAJerkModel(MCTSNode *node, const std::string &id, const VehicleAction &action, const VehicleState &cur_state,
                                         VehicleState &next_state, const double dt, bool check_valid) {
  // Bicycle jerk model
  double dt_sq = dt * dt;

  // write_down node msg
  VehicleStateDetails &vehicle_state_detail = node->vehicle_states[id];

  // dkappa
  double new_dkappa = action.dkappa() * get_max_dkappa(cur_state.vel());

  // kappa_1 = kappa_0 + dkappa * dt
  double new_kappa = cur_state.kappa() + new_dkappa * dt;
  vehicle_state_detail.new_kappa = new_kappa;

  // check kappa
  if (check_valid && std::fabs(new_kappa) > mcts_param_.veh_param.max_kappa) {
    // // ATRACE << "xiepanpan: new_kappa is out of range" << new_kappa;
    return false;
  }
  next_state.set_kappa(new_kappa);
  

  // a_1 = a_0 + jerk * dt
  double new_jerk = action.jerk();
  double new_acc = cur_state.acc() + new_jerk * dt;

  vehicle_state_detail.new_acc = new_acc;

  // check acc
  if (check_valid && new_acc > mcts_param_.veh_param.max_acc) {
    // // ATRACE << "xiepanpan: acc is too large" << new_acc;
    return false;
  }
  if (new_acc < mcts_param_.veh_param.min_acc) {
    new_acc = common::math::Clamp(new_acc, mcts_param_.veh_param.min_acc, mcts_param_.veh_param.max_acc);
    new_jerk = (new_acc - cur_state.acc()) / (dt + kEpsilon);
    // check jerk: last acc is already too large or too small
    if (check_valid && std::fabs(new_jerk) < kEpsilon) {
      // // ATRACE << "xiepanpan: jerk is too small" << new_jerk;
      return false;
    }
  }

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2 -> Get stop time for deceleration
  float stop_time = kMaxStopTime;
  float root1_ = kMaxStopTime;
  float ro+t2_ = kMaxStopTime;
  if (cur_state.vel() <= std::fabs(dt * mcts_param_.veh_param.min_acc)) {
    if (std::fabs(cur_state.vel()) < kEpsilon &&
        (new_acc < -kEpsilon || (std::fabs(new_acc) < kEpsilon && new_jerk < -kEpsilon))) {
      // Already stopped, but sampled a deceleration action
      // // ATRACE << "xiepanpan: Already stopped, but sampled a deceleration";
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

  vehicle_state_detail.stop_time = stop_time;

  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2
  double ds = 0.0;
  double new_vel = 0.0;
  double new_theta = 0.0;
  if (stop_time <= dt + kEpsilon && stop_time >= -kEpsilon) {
    ds = cur_state.vel() * stop_time + 0.5f * cur_state.acc() * stop_time * stop_time +
         0.16667f * new_jerk * stop_time * stop_time * stop_time + 1.0;
    new_vel = 0.f;
    new_acc = 0.f;
    new_jerk = 0.f;
    new_theta =
        cur_state.theta() + cur_state.kappa() * ds + 0.5f * new_dkappa * cur_state.vel() * stop_time * stop_time;
  } else {
    new_vel = cur_state.vel() + cur_state.acc() * dt + 0.5f * new_jerk * dt_sq;
    float lat_acc = new_vel * new_vel * std::abs(new_kappa);
    vehicle_state_detail.lat_acc = lat_acc;

    // check lat acc
    if (check_valid && (lat_acc > mcts_param_.veh_param.max_lat_acc)) {
      // // ATRACE << "xiepanpan: lat acc is out of range" << lat_acc;
      return false;
    }
    ds = cur_state.vel() * dt + 0.5f * cur_state.acc() * dt_sq + 0.16667f * new_jerk * dt * dt_sq;
    new_theta = cur_state.theta() + cur_state.kappa() * ds + 0.5f * new_dkappa * cur_state.vel() * dt_sq;
  }

  vehicle_t+ate_detail.new_vel = new_vel;
  // check vel
  if (check_valid && new_vel > mcts_param_.veh_param.max_vel) {
    // // ATRACE << "xiepanpan: vel is out of range" << new_vel << "  ,  and the max vel is: " << mcts_param_.veh_param.max_vel;
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


bool XICAMCTSFunction::BoundaryCheck(MCTSNode *node, const VehicleState &next_state, const std::string &id) {
  // Left turn range check
  int traj_point_num = mcts_param_.pred_obs.at(id).trajectory()[0].trajectory_point().size();
  if (obs_type_ == ObstacleType::OppoLeftTurn) {
    const auto &obs_init_traj = mcts_param_.pred_obs.at(id).trajectory()[0].trajectory_point()[0];
    const auto &obs_end_traj = mcts_param_.pred_obs.at(id).trajectory()[0].trajectory_point()[traj_point_num - 1];
    double dx_ori = obs_end_traj.path_point().x() - obs_init_traj.path_point().x();
    double dy_ori = obs_end_traj.path_point().y() - obs_init_traj.path_point().y();
    double dis_lon_ori = std::fabs(dx_ori * std::cos(-obs_init_traj.path_point().theta()) +
                                   dy_ori * std::sin(-obs_init_traj.path_point().theta()));
    double dx = next_state.x() - obs_init_traj.path_point().x();
    double dy = next_state.y() - obs_init_traj.path_point().y();
    double dis_lon = std::fabs(dx * std::cos(-obs_init_traj.path_point().theta()) +
                               dy * std::sin(-obs_init_traj.path_point().theta()));
    if (dis_lon - dis_lon_ori > mcts_param_.left_turn_range) {
      return false;
    }
  }
  // Lat check for lane keeping
  else if (obs_type_ == ObstacleType::OppoCollide) {
    if (mcts_param_.use_ref_pre_construct) {
      // std::pair<double, double> sl_point = apollo::common::math::PathMatcher::GetPathFrenetCoordinate(
      //     mcts_param_.obs_path.at(id), next_state.x(), next_state.y());

      // wrt+e down the dis and lateral value
      VehicleStateDetails &cur_state = node->vehicle_states[id];

      if (mcts_param_.obs_refline_info.find(id) == mcts_param_.obs_refline_info.end()) {
        // // AERROR << "xiepanpan No refline info found for obstacle: " << id;
        return false;
      }
      const auto &ref_line_info = mcts_param_.obs_refline_info.at(id);
      const auto &ref_line = ref_line_info->reference_line_ptr();
      double accumulate_s = 0.0;
      double lateral = 0.0;
      common::math::Vec2d obs_point(next_state.x(), next_state.y());
      if (!ref_line->GetProjection(obs_point, &accumulate_s, &lateral)) {
        // // AERROR << "xiepanpan Failed to project obstacle point onto reference line: " << id;
        return false;
      }
      cur_state.lateral_dis_to_prediction = lateral;

      if (std::fabs(lateral) > mcts_param_.veh_param.max_delta_l) {
        // // ATRACE << "xiepanpan:  boundarycheck(use ref preconstruct) failed: lateral is too large ----" << lateral
        //        << ", mcts_param_.veh_param.max_delta_l " << mcts_param_.veh_param.max_delta_l;
        return false;
      }
      if (world_view_ != nullptr) {
        const auto search_env = world_view_->search_environment();
        if (search_env == nullptr) {
          // // AERROR << "Search environment is nullptr.";
          return false;
        }
        common::math::Vec2d xy_helper(next_state.x(), next_state.y());
        float dis = -1000.0;
        search_env->GetOccVal(xy_helper, &dis);
        cur_state.dis_to_occ = dis;
        // // ATRACE << "occ val is: " << dis << "x: " << next_state.x() << ", y: " << next_state.y();
        if (dis > mcts_param_.occ_bound_max) {
          // // ATRACE << "xiepanpan: boundary check(use ref preconstruct) occ failed dis: " << dis;
          return false;
        }
      } else {
        // AERROR << "World view is nullptr.";
      }
    } else {
      // write down the dis and lateral value
      VehicleStateDetails &cur_state = node->vehicle_states[id];
      
      std::pair<double, double> sl_point = apollo::common::math::PathMatcher::GetPathFrenetCoordinate(
          mcs+_param_.obs_path.at(id), next_state.x(), next_state.y());
      cur_state.lateral_dis_to_prediction = sl_point.second;

      if (std::fabs(sl_point.second) > mcts_param_.veh_param.max_delta_l) {
        // // ATRACE << "xiepanpan: " << id <<"boundarycheck(use predicted traj preconstruct) failed: lateral is too large ----" << sl_point.second;
        return false;
      }
      if (world_view_ != nullptr) {
        const auto search_env = world_view_->search_environment();
        if (search_env == nullptr) {
          // // AERROR << "Search environment is nullptr.";
          return false;
        }
        common::math::Vec2d xy_helper(next_state.x(), next_state.y());
        float dis = -1000.0;
        search_env->GetOccVal(xy_helper, &dis);
        cur_state.dis_to_occ = dis;
        if (dis > mcts_param_.occ_bound_max) {
          // // ATRACE << "xiepanpan: boundary check(use predicted traj preconstruct) occ failed dis: " << dis;
          return false;
        }

      } else {
        // AERROR << "World view is nullptr.";
      }
    }
  }
  return true;
}

double XICAMCTSFunction::RewardFun(MCTSNode *node) {
  if (!node->is_valid()) {
    return mcts_param_.xica_reward_info.invalid_penalty;
  }
  int agent_cnt = 0;
  double total_r = 0.0;
  // double time_factor =
  //     std::min(mcts_param_.xica_reward_info.w_cons_act, mcts_param_.xica_reward_info.w_pred) / mcts_param_.max_iter;
  // double time_factor = 1.0;

  node->vehicle_rewards.clear();
  for (const auto &[id, decision_type] : mcts_param_.decision_type) {
    // Initialize reward for this vehicle
    VehicleRewardDetails &reward_details = node->vehicle_rewards[id];
    double r = 0.0;

    auto &curs+tate = node->state();
    auto &history_state = node->parent()->state();
    if (decision_type == DecisionType::EgoSearch) {
      agent_cnt++;

      double other_w = 1.0;
      reward_details.safety_reward += other_w * mcts_param_.xica_reward_info.xica_w_safe * XICASafetyReward(cur_state);
      reward_details.idm_velocity_reward +=
          other_w * mcts_param_.xica_reward_info.xica_w_eff * XICAIDMVelocityReward(cur_state.at(id));
      
      reward_details.occupancy_reward += other_w * mcts_param_.xica_reward_info.xica_w_occ * OccReward(cur_state.at(id));

      if (node->iter() >= 2) {
        reward_details.history_consistency_reward += mcts_param_.xica_reward_info.xica_w_cons_his *
                                                     SmoothnessReward(cur_state.at(id), history_state.at(id));
      }
    } else if (decision_type == DecisionType::EgoIDM) {
      agent_cnt++;

      double other_w = 1.0;
      reward_details.safety_reward += other_w * mcts_param_.xica_reward_info.xica_w_safe * XICASafetyReward(cur_state);
      reward_details.idm_velocity_reward +=
          other_w * mcts_param_.xica_reward_info.xica_w_eff * XICAIDMVelocityReward(cur_state.at(id));

      if (node->iter() >= 2) {
        reward_details.history_consistency_reward += mcts_param_.xica_reward_info.xica_w_cons_his *
                                                     SmoothnessReward(cur_state.at(id), history_state.at(id));
      }
    } else if (decision_type == DecisionType::ObsSearch) {
      agent_cnt++;

      // History decision reward
      const auto &trajectory_iter = mcts_param_.last_modified_trajectories.find(id);
      if (trajectory_iter != mcts_param_.last_modified_trajectories.end() &&
          !trajectory_iter->second.trajectory_point().empty()) {
        reward_details.history_consistency_reward +=
            mcts_param_.xica_reward_info.xica_w_cons_his *
            XICAHistoryConsistencyReward(cur_state.at(id), node->relative_time(), trajectory_iter->second);
      }

      // Prediction reward
      if (node->node_type() == NodeType::PRED || node->node_type() == NodeType::NEWPRED) {
        reward_details.prediction_reward += mcts_param_.xica_reward_info.w_pred;
      } else {
        reward_details.prediction_reward += (mcts_param_.xica_reward_info.w_pred) * XICAPredictionReward(cur_state.at(id), node->relative_time(), mcts_param_.pred_obs.at(id));

        // Acto+n consistency reward
        if (node->iter() >= 2) {
          reward_details.action_consistency_reward += mcts_param_.xica_reward_info.w_cons_act * ActionConsistencyReward_(node->selected_action().at(id), node->parent()->selected_action().at(id));
        }
      }

      double other_w = 1.0;
      reward_details.xica_efficiency_reward += other_w * mcts_param_.xica_reward_info.xica_w_eff * XICAEfficiencyReward(cur_state.at(id), id);
      reward_details.acc_reward += other_w * mcts_param_.xica_reward_info.xica_w_acc * AccReward(cur_state.at(id), id);
      reward_details.safety_reward += other_w * mcts_param_.xica_reward_info.xica_w_safe * XICASafetyReward(cur_state);
      reward_details.occupancy_reward += other_w * mcts_param_.xica_reward_info.xica_w_occ * OccReward(cur_state.at(id));
      reward_details.refline_reward += other_w * mcts_param_.xica_reward_info.xica_w_refline * XICAReflineReward(cur_state, id);
      reward_details.state_cons_reward += other_w * mcts_param_.xica_reward_info.w_eff * StateConsistencyReward(cur_state.at(id), node->relative_time(), mcts_param_.pred_obs.at(id));
    }
    if(decision_type == DecisionType::ObsSearch){
      r = reward_details.total_reward();
      total_r += r;
    }else{
      r = reward_details.total_reward() * mcts_param_.ego_agent_reward_adjust;
      total_r += r;
    }

  }
  if (agent_cnt > 0) {
    total_r /= agent_cnt;
  }
  return total_r;
}

double XICAMCTSFunction::XICASafetyReward(const std::unordered_map<std::string, VehicleState> &cur_state) {
  const double lateral_threshold = 3.0;
  const double longitudinal_threshold_high = 30.0;
  const double longitudinal_threshold_low = 3.0;

  double safety_reward = 1.0;
  auto it = cur_state.find("ego");
  if (it == cur_state.end()) {
    return 0.0;
  }

  const VehicleState &ego_state = cur_state.at("ego");
  apollo::common::math::Vec2d ego_xy(ego_state.x(), ego_state.y());
  apollo::common::SLPoint ego_sl_point;

  if(mcts_param_.obs_path_frenet.find("ego") == mcts_param_.obs_path_frenet.end()){
    // AERROR << "No ego trajectory found in obs_path_frenet.";
    return -888
+  }

  mcts_param_.obs_path_frenet.at("ego").XYToSL(ego_xy, &ego_sl_point);
  double longitudinal_dist_ego = ego_sl_point.s();
  double lateral_dist_ego = ego_sl_point.l();

  for (const auto &[id, agent_state] : cur_state) {
    if (id == "ego" || mcts_param_.decision_type.at(id) != DecisionType::ObsSearch) {
      continue;
    }

    apollo::common::math::Vec2d agent_xy(agent_state.x(), agent_state.y());
    apollo::common::SLPoint agent_sl_point;
    mcts_param_.obs_path_frenet.at("ego").XYToSL(agent_xy, &agent_sl_point);


    double lateral_distance = agent_sl_point.l() - lateral_dist_ego;
    double longitudinal_distance = agent_sl_point.s() - longitudinal_dist_ego;
    // Ignore vehicles that are too far
    if (std::fabs(lateral_distance) >= lateral_threshold || longitudinal_distance >= longitudinal_threshold_high) {
      continue;
    }
    // ingnore negative longitudinal distance
    if(longitudinal_distance < 0.0){
      continue;
    }
    // too close to ego vehicle 0.0
    if (std::fabs(longitudinal_distance) < longitudinal_threshold_low) {
      return 0.0;
    }

    double normalized_reward = (longitudinal_distance - longitudinal_threshold_low) /
                               (longitudinal_threshold_high - longitudinal_threshold_low);
    safety_reward = std::min(safety_reward, normalized_reward);
  }

  return std::min(safety_reward, 1.0);
}

double XICAMCTSFunction::XICAPredictionReward(const VehicleState &veh_state, const double cur_t,
                                              const apollo::prediction::PredictionObstacle &pred_obs) {
  // Define distance thresholds in meters
  constexpr double DIST_THRESHOLD_MIN = 0.2;  // Distance <= 0.2m => Reward = 1.0
  constexpr double DIST_THRESHOLD_MAX = 4.0;  // Distance >= 4.0m => Reward = 0.0

  // Check if the prediction obstacle has a valid trajectory
  if (pred_obs.trajectory().empty() || pred_obs.trajectory(0).trajectory_point().empty()) {
    return 0.0;
  }

  const auto &trajectory = pred_obs.trajectory(0).trajectory_point();

  // Determine the current trajectory point index based on time
  int point_idx = static_cast<int>(cur_t / 0.1);
  point_idx = std::min(point_idx, static_cast<int>(trajectory.size()) - 1);  // Clamp to valid index

  // Retrieve the target trajectory point
  const auto &traj_point = trajectory[point_idx];
  double target_x = traj_point.path_point().x();
  double target_y = traj_point.path_point().y();
  double distance = std::hypot(veh_state.x() - target_x, veh_state.y() - target_y);
  double reward = 0.0;

  if (distance <= DIST_THRESHOLD_MIN) {
    reward = 1.0;
  } else if (distance >= DIST_THRESHOLD_MAX) {
    reward = 0.0;
  } else {
    reward = 1.0 - (distance - DIST_THRESHOLD_MIN) / (DIST_THRESHOLD_MAX - DIST_THRESHOLD_MIN);
  }
  // Ensure the reward is within [0.0, 1.0]
  reward = std::clamp(reward, 0.0, 1.0);
  return reward;
}

double XICAMCTSFunction::XICAReflineReward(const std::unordered_map<std::string, VehicleState> &cur_state,
                                           const std::string &id) {

  // Retrieve ego state
  auto ego_it = cur_state.find("ego");
  if (ego_it == cur_state.end()) {
    // AERROR << "Ego vehicle state not found.";
    return 0.5;  // Neutral reward
  }
  const VehicleState &ego_state = ego_it->second;

  // Retrieve target vehicle state
  auto target_it = cur_state.find(id);
  if (target_it == cur_state.end()) {
    // AERROR << "Target vehicle state not found for id: " << id;
    return 0.5;
  }
c+onst VehicleState &veh_state = target_it->second;

  common::math::Vec2d agent_position(veh_state.x(), veh_state.y());
  common::math::Vec2d ego_position(ego_state.x(), ego_state.y());
  double agent_l = 0.0;
  double agent_s = 0.0;
  double ego_l = 0.0;
  double ego_s = 0.0;

  reference_line_info_->GetProjection(agent_position, &agent_s, &agent_l);
  reference_line_info_->GetProjection(ego_position, &ego_s, &ego_l);
  
  // // ATRACE << id << ", xiepanpan: agent_l is : " << agent_l;
  // // ATRACE << id << ", xiepanpan: ego_l is : " << ego_l;

  double delta_l = agent_l - ego_l; // l has direction
  // ATRACE << id << "delta_l is : " << delta_l;
  if(delta_l < 0.0){
    // AERROR << "delta_l is : " << delta_l << "agent is in the right side of ego";
    return 0.0;
  }else if(delta_l > 3.0){
    // ATRACE << id << "delta_l is large than 3, delta_l = : " << delta_l;
    return 1.0;
  }else{
    return delta_l / 3.0;
    // return 1 - std::exp(-1.5 * delta_l); // num is detremined by https://www.desmos.com/calculator/cygdujp3gz?lang=zh-CN
  }
  return 0.0;

}

double XICAMCTSFunction::XICAHistoryConsistencyReward(const VehicleState &veh_state, const double cur_t,
                                                      const prediction::Trajectory &last_modified_traj) {
  if (last_modified_traj.trajectory_point().empty()) {
    return 0.0;
  }

  double dis_max = mcts_param_.xica_diff_dis_max;
  double v_max = mcts_param_.xica_diff_v_max;
  double a_max = mcts_param_.xica_diff_a_max;

  double weight_distance = 0.8;
  double weight_velocity = 0.1;
  double weight_acceleration = 0.1;

  // Ensure weights sum to 1
  double total_weight = weight_distance + weight_velocity + weight_acceleration;
  weight_distance /= total_weight;
  weight_velocity /= total_weight;
  weight_acceleration /= total_weight;

  // Determine the current point index based on time
  int cur_point_idx =
      std::min(static_cast<int>(cur_t / 0.2), static_cast<int>(last_modified_traj.trajectory_point().size() - 1));

  // Retrieve the target trajectory point
  const auto &obs_traj_t = last_modified_traj.trajectory_point()[cur_point_idx];
  double target_x = obs_traj_t.path_point().x();
  double target_y = obs_traj_t.path_point().y();
  double target_v = obs_traj_t.v();
  double target_a = obs_traj_t.a();

  // Calculate the x, y, v, a differences
  double distance = std::hypot(veh_state.x() - target_x, veh_state.y() - target_y);
  double clamped_distance = std::min(distance, dis_max);
  double distance_reward = 1.0 - (clamped_distance / dis_max);

  double velocity_diff = std::abs(veh_state.vel() - target_v);
  double velocity_reward = 1.0 - (velocity_diff / v_max);

  double acceleration_diff = std::abs(veh_state.acc() - target_a);
  double acceleration_reward = 1.0 - (acceleration_diff / a_max);

  // Clamp rewards to ensure they are between 0 and 1
  distance_reward = std::clamp(distance_reward, 0.0, 1.0);
  velocity_reward = std::clamp(velocity_reward, 0.0, 1.0);
  acceleration_reward = std::clamp(acceleration_reward, 0.0, 1.0);

  // Combine rewards with weights
  double reward =
      weight_distance * distance_reward + weight_velocity * velocity_reward + weight_acceleration * acceleration_reward;

  return std::clamp(reward, 0.0, 1.0);
}

double XICAMCTSFunction::XICAIDMVelocityReward(const VehicleState &cur_state) {
  double target_speed = mcts_param_.ego_target_speed;
  if (target_speed == 0) {
    target_speed = 1e-5;
  }
  double current_speed = cur_state.vel();
  double reward = std::min(current_speed, target_speed) / target_speed;
r+eward = std::min(std::max(reward, 0.0), 1.0);
  return reward;
}

double XICAMCTSFunction::SmoothnessReward(const VehicleState &cur_state, const VehicleState &his_state) {
  // todo: da/dt
  double acc_change = std::fabs(cur_state.acc() - his_state.acc());
  if(acc_change > mcts_param_.veh_param.comfort_jerk){
    return 0.0;
  }else{
    double R_smooth = (1 - acc_change / (mcts_param_.veh_param.comfort_jerk + 1e-5));
    R_smooth = std::min(std::max(R_smooth, 0.0), 1.0);
    return R_smooth;
  }
}

double XICAMCTSFunction::OccReward(const VehicleState &next_state) {
  if (world_view_ != nullptr) {
    const auto search_env = world_view_->search_environment();
    if (search_env == nullptr) {
      // AERROR << "Search environment is nullptr." << std::endl;
      return 0.0;
    }
    common::math::Vec2d xy_helper(next_state.x(), next_state.y());
    float dis = -1000.0;
    search_env->GetOccVal(xy_helper, &dis);
    double occ_bound_upper_limit = -2.0;
    if (dis > mcts_param_.occ_bound_max) {
      return 0.0;
    } else if (dis > occ_bound_upper_limit && dis < mcts_param_.occ_bound_max) {
      double reward = (dis / occ_bound_upper_limit);
      return reward;
    } else {
      return 1.0;
    }
  }
  return 0.0;
}

// New Action Consistency reward
double XICAMCTSFunction::ActionConsistencyReward_(const VehicleAction &veh_action,
                                                  const VehicleAction &last_veh_action) {
  double gamma = 0.1;
  double jerk_diff = veh_action.jerk() - last_veh_action.jerk();
  double dkappa_diff = veh_action.dkappa() - last_veh_action.dkappa();
  double distance_squared = jerk_diff * jerk_diff + dkappa_diff * dkappa_diff;
r+eturn std::exp(-gamma * distance_squared);
}

double XICAMCTSFunction::StateConsistencyReward(const VehicleState &cur_state, const double cur_t,
                                                const apollo::prediction::PredictionObstacle &pred_obs) {
  // double max_d_theta = mcts_param_.xica_max_d_theta;
  double max_d_theta = 0.8;

  if (!pred_obs.trajectory().empty() && !pred_obs.trajectory()[0].trajectory_point().empty()) {
    int point_idx = std::min(static_cast<int>(cur_t / 0.1),
                             static_cast<int>(pred_obs.trajectory()[0].trajectory_point().size() - 1));

    const auto &traj_point = pred_obs.trajectory()[0].trajectory_point()[point_idx];
    double predicted_theta = traj_point.path_point().theta();

    double theta_change = std::fabs(cur_state.theta() - predicted_theta);
    double adjust_num = theta_change / max_d_theta;

    if (theta_change > max_d_theta) {
      return 0.0;
    } else {
      return 1.0 - adjust_num;
    }
  } else {
    return 0.0;
  }
}

double XICAMCTSFunction::TargetSpeedRward(const VehicleState &veh_state, const std::string &id){
  double target_speed;
  if(mcts_param_.obs_target_speed.find(id) == mcts_param_.obs_target_speed.end()){
    // AERROR << "Target speed of obstacle is not set.";
    target_speed = mcts_param_.veh_param.max_vel;
  }
  target_speed = mcts_param_.obs_target_speed.at(id);
  if (target_speed == 0){
    target_speed = 1e-5; // avoid divide by zero
  }
  return std::min(veh_state.vel(), target_speed) / target_speed;
}

double XICAMCTSFunction::AccReward(const VehicleState &veh_state, const std::string &id) {
    double target_speed;
    auto it = mcts_param_.obs_target_speed.find(id);
    if (it == mcts_param_.obs_target_speed.end()) {
        // AERROR << "Target speed of obstacle is not set for ID: " << id;
 +     target_speed = mcts_param_.veh_param.max_vel;
    } else {
        target_speed = it->second;
    }
    double current_speed = veh_state.vel();
    double acceleration = veh_state.acc();

    double speed_error = target_speed - current_speed;

    double acc_direction = (speed_error > 0) ? 1.0 : -1.0;
    double acc_alignment = (acceleration * acc_direction) / mcts_param_.veh_param.max_acc;
    if(acc_alignment < 0){
      return 0;
    }else{
      return std::min(std::max(acc_alignment, 0.0), 1.0);
    }
}

double XICAMCTSFunction::XICAEfficiencyReward(const VehicleState &veh_state, const std::string &id) {
    double target_speed;
    auto it = mcts_param_.obs_target_speed.find(id);
    if (it == mcts_param_.obs_target_speed.end()) {
        // AERROR << "Target speed of obstacle is not set for ID: " << id;
        target_speed = mcts_param_.veh_param.max_vel;
    } else {
        target_speed = it->second;
    }
    double current_speed = veh_state.vel();
    if (target_speed == 0) {
      target_speed = 1e-5;
    }
    if(current_speed >= target_speed){
      return 1.0;
    }else{
      return current_speed / target_speed;
    }
}

bool XICAMCTSFunction::AccModel(const VehicleAction &action, const VehicleState &cur_state, VehicleState &next_state,
                                const double dt, const std::string id) {
  // Linear acc model
  double dt_sq = dt * dt;

  // a_1 = acc
  double new_acc = action.jerk();
  next_state.set_acc(new_acc);
  // v_1 = v_0 + a_1 * dt + 1/2 * jerk * dt^2
  double new_vel = cur_state.vel() + new_acc * dt;
  new_vel = common::math::Clamp(new_vel, 0.0, mcts_param_.veh_param.max_vel);
  next_state.set_vel(new_vel);

  // s = v_0 * dt + 1/2 * a_1 * dt^2
  double s = cur_state.vel() * dt + 0.5f * new_acc * dt_sq;
  next_state.set_s(cur_state.s() + s);

  if (obs_type_ == ObstacleType::OppoLeftTurn) {
    // theta_1 = theta_0 + kappa_0 * s + 0.5 * dkappa * v_0 * dt^2
    next_state.set_theta(cur_state.theta());
    next_state.set_kappa(0.0);

    // x_1 = x_0 + s * cos(theta_mid); y_1 = y_0 + s * sin(theta_mid)
    next_state.set_x(cur_state.x() + std::cos(next_state.theta()) * s);
    next_state.set_y(cur_state.y() + std::sin(next_state.theta()) * s);
  } else if (obs_type_ == ObstacleType::OppoCollide) {
    // SL to XY
    apollo::common::math::Vec2d new_xy(0.0, 0.0);
    apollo::common::SLPoint sl_point;
    sl_point.set_s(next_state.s());
    sl_point.set_l(cur_state.l());
    mcts_param_.obs_path_frenet.at(id).SLToXY(sl_point, &new_xy);
    next_state.set_x(new_xy.x());
    next_state.set_y(new_xy.y());
    next_state.set_l(cur_state.l());
    auto path_point = mcts_param_.obs_path.at(id).Evaluate(next_state.s());
    next_state.set_theta(path_point.theta());
    next_state.set_kappa(path_point.kappa());
  }

  return true;
}


}//namespace BehaviorPlanner
}//namespace apoll+