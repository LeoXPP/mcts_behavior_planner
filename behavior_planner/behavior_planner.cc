#include "behavior_planner.h"
#include <algorithm>
#include <chrono>
#include <iostream>

namespace apollo {
namespace BehaviorPlanner {

void Planner::Init() {
  if (LoadParams()) {
    std::cout << "LoadParams success!" << std::endl;
  }
}

bool Planner::MakeDecision() {
  // 1. build gaming info
  if (!BuildGamingInfo()) {
    return false;
  }

  // 2. set gaming agent info
  if (!SetGamingAgentInfo()) {
    return false;
  }

  for (auto &obstacle : pred_obs_) {
    if (!UpdateDecisionParams(obstacle.second, obstacle.first, OppoCollide)) {
      return false;
    }

    // 4. construct MCTree
    if (!ConstructMCTree()) {
      return false;
    }

    auto start_t = std::chrono::high_resolution_clock::now();

    if (!mcts_tree_->UctSearch()) {
      std::cout << "UctSearch failed!" << std::endl;
    }
    auto end_t = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = end_t - start_t;
    std::cout << "UctSearch time(ms): " << diff.count() * 1000 << std::endl;

    // // 4.Modify Trajectory
    // if (!InterpolateResult(true)) {
    //   std::cout << "Failed interpolation";
    //   continue;
    // }
  }
  return true;
}

bool Planner::InterpolateResult(bool need_extend) {
  // Only for decision obstacles
  if (!mcts_tree_->SearchBestNodeSeq()) {
    return false;
  }
  mcts_tree_->SaveTreeVisualization("xiepanpan_mcst_tree.json", 1);

  return true;
}



bool Planner::UpdateDecisionParams(const ObstacleInfo &obstacle,
                                   const std::string &id,
                                   const ObstacleType &obs_type) {
  mcts_tree_ = nullptr;

  // Target Speed
  mcts_param_.obs_target_speed[id] = obstacle.trajectory()[0].trajectory_point()[0].v();

  // Prediction obstacles
  // pred_obs_.clear();
  pred_obs_[id] = obstacle;
  if (pred_obs_.at(id).trajectory().size() == 0 ||
      pred_obs_.at(id).trajectory()[0].trajectory_point().size() == 0) {
    // AERROR << "Obstacle trajectory is empty.";
    return false;
  }

  // time_step & w_pred
  double vel_factor = 0.0;
  mcts_param_.time_step.clear();

  mcts_param_.time_step = {0.5, 0.5, 1.0, 1.0, 2.0};
  mcts_param_.max_iter = mcts_param_.time_step.size();
  mcts_param_.reward_info.w_pred =
      std::min(mcts_param_.reward_info.w_pred, 0.8);
  vel_factor = 1.0;

  // ego_traj_points
  mcts_param_.ego_traj_points.clear();
  double cur_t = 0.0;
  TrajectoryPoint previous_ego_point = ego_traj_.Evaluate(cur_t);
  mcts_param_.ego_traj_points.push_back(ego_traj_.Evaluate(cur_t));
  for (size_t i = 0; i < mcts_param_.time_step.size(); ++i) {
    cur_t += mcts_param_.time_step[i];
    if (ego_traj_.TrajectoryPointAt(ego_traj_.NumOfPoints() - 1)
            .relative_time() < cur_t) {
      // std::cout << "cur_t: " << cur_t << "relative_time: "
      //  << ego_traj_.TrajectoryPointAt(ego_traj_.NumOfPoints() - 1)
      //         .relative_time();
      // std::cout << "xiepanpan: Ego traj length is not sufficient. Using constant
      // velocity model to extend.";
      auto last_valid_point = previous_ego_point;
      double last_x = last_valid_point.path_point().x();
      double last_y = last_valid_point.path_point().y();
      double heading = last_valid_point.path_point().theta();
      double velocity = last_valid_point.v();

      double delta_time = mcts_param_.time_step[i];
      double new_x = last_x + velocity * delta_time * std::cos(heading);
      double new_y = last_y + velocity * delta_time * std::sin(heading);

      TrajectoryPoint new_point;
      new_point.set_path_point().set_x(new_x);
      new_point.set_path_point().set_y(new_y);
      new_point.set_path_point().set_theta(heading);
      new_point.set_v(velocity);
      new_point.set_relative_time(cur_t);

      previous_ego_point = new_point;
      mcts_param_.ego_traj_points.push_back(new_point);
    } else {
      previous_ego_point = ego_traj_.Evaluate(cur_t);
      mcts_param_.ego_traj_points.push_back(ego_traj_.Evaluate(cur_t));
    }

    common::math::Vec2d ego_point(
        mcts_param_.ego_traj_points.back().path_point().x(),
        mcts_param_.ego_traj_points.back().path_point().y());
    double accumulate_s = 0.0;
    double lateral = 0.0;
    // if (!reference_line_info_->GetProjection(ego_point, &accumulate_s,
    //                                          &lateral)) {
    //   // AERROR << "Failed to get ego path projection.";
    //   return false;
    // }
    if (std::fabs(lateral) > mcts_param_.veh_param.max_delta_l) {
      // std::cout << "Ego lateral is too large.";
      return false;
    }
  }

  // Distance Judgement
  double decision_distance =
      std::hypot(mcts_param_.ego_traj_points[0].path_point().x() -
                     pred_obs_.at(id)
                         .trajectory()[0]
                         .trajectory_point()[0]
                         .path_point()
                         .x(),
                 mcts_param_.ego_traj_points[0].path_point().y() -
                     pred_obs_.at(id)
                         .trajectory()[0]
                         .trajectory_point()[0]
                         .path_point()
                         .y());

  // Reaction Time
  double agent_speed = obstacle.trajectory()[0].trajectory_point()[0].v();
  double ego_speed = mcts_param_.ego_traj_points[0].v();
  double interval_speed = ego_speed - agent_speed;
  // std::cout << "agent speed: " << agent_speed << ", ego speed: " << ego_speed
  //  << ", interval speed: " << interval_speed;
  if (interval_speed == 0.0) {
    // std::cout << "Interval speed is zero.";
    interval_speed = 0.1;
  } else if (interval_speed < 0) {
    // std::cout << "Interval speed is negative."
    //  << ", orginal interval_speed: " << interval_speed;
    interval_speed = 0.1;
  }
  double reaction_time = decision_distance / interval_speed;
  if (reaction_time < mcts_param_.min_decision_delta_t) {
    // std::cout << "Obs " << id << "is too near to make decision."
    //  << ", reaction_time: " << reaction_time;
    return false;
  }

  // Obstacle prediction path
  mcts_param_.obs_path.clear();
  // mcts_param_.obs_path_frenet.clear();
  std::vector<PathPoint> path_points;
  std::vector<common::math::Vec2d> path_points_xy;
  double s = 0.0;
  double max_vel = 10.0;
  // ego frenet path
  std::vector<PathPoint> path_points_ego;
  std::vector<common::math::Vec2d> path_points_xy_ego;

  for (auto ego_traj_point : ego_traj_) {
    path_points_ego.emplace_back(ego_traj_point.path_point());
    path_points_xy_ego.emplace_back(ego_traj_point.path_point().x(),
                                    ego_traj_point.path_point().y());
  }
  mcts_param_.obs_path["ego"] = DiscretizedPath(path_points_ego);

  double kMathEpsilon = 1e-6;
  mcts_param_.veh_param.max_vel =
      std::min<double>(vel_factor * max_vel + kMathEpsilon,
                       mcts_dynamics_.max_v()) +
      20.0;

  // Decision type
  decision_type_.clear();
  // decision_type_["ego"] = DecisionType::EgoPlanning;

  if (mcts_param_.xica_need_ego_idm) {
    decision_type_["ego"] = DecisionType::EgoIDM;
    // std::cout << "xiepanpan: EgoIDM";
  } else {
    decision_type_["ego"] = DecisionType::EgoSearch;
    //decision_type_["ego"] = DecisionType::EgoPlanning;
    // std::cout << "xiepanpan: EgoSearch";
  }

  decision_type_[id] = DecisionType::ObsSearch;

  // CIPV
  for (auto &[cipv_id, cipv_pred] : pred_cipv_) {
    pred_obs_[cipv_id] = cipv_pred;
    decision_type_[cipv_id] = DecisionType::ObsPrediction;
  }
  mcts_param_.pred_obs = pred_obs_;
  mcts_param_.decision_type = decision_type_;

  // Action Space
  mcts_param_.norm_action.clear();
  mcts_param_.lk_action.clear();
  mcts_param_.pred_action.clear();

  // TODO: set as config
  mcts_param_.ego_lat_action.clear();
  std::vector<double> ego_lat_action_set = {0.1, 0.0, -0.1, -0.2};
  for (size_t i = 0; i < ego_lat_action_set.size(); ++i) {
    mcts_param_.ego_lat_action.push_back(
        VehicleAction(0, ego_lat_action_set[i]));
  }

  std::vector<double> jerk_action{-2.0, -1.0, 0.0, 1.0};
  std::vector<double> dkappa_action{ -0.75, -0.25, 0, 0.1};
  // std::vector<double> jerk_action{-2.0, -1.0, 0.0};
  // std::vector<double> dkappa_action{-1.0, -0.75, -0.5, -0.25, 0, 0.25};

  for (size_t i = 0; i < jerk_action.size(); ++i) {
    for (size_t j = 0; j < dkappa_action.size(); ++j) {
      mcts_param_.norm_action.push_back(
          VehicleAction(jerk_action[i], dkappa_action[j]));
    }
    mcts_param_.lk_action.push_back(VehicleAction(jerk_action[i], 0.f));
    mcts_param_.pred_action.push_back(VehicleAction(jerk_action[i], 0.f));
  }

  mcts_param_.ego_agent_action.clear();
  for (size_t i = 0; i < ego_lat_action_set.size(); ++i) {
    for (size_t j = 0; j < jerk_action.size(); ++j) {
      for (size_t k = 0; k < dkappa_action.size(); ++k) {
        std::unordered_map<std::string, VehicleAction> node_action;
        node_action["ego"] = VehicleAction(0.f, ego_lat_action_set[i]);
        node_action[id] = VehicleAction(jerk_action[j], dkappa_action[k]);
        mcts_param_.ego_agent_action.push_back(node_action);
      }
    }
  }

  // Shuffle ego_agent_action
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(mcts_param_.ego_agent_action.begin(),
               mcts_param_.ego_agent_action.end(), gen);

  mcts_func_ = new XICAMCTSFunction(mcts_param_, obs_type);
  // mcts_func_->InitWorldView(world_view_);
  // mcts_func_->InitReferenceLineInfo(reference_line_info_);

  return true;
}

bool Planner::ConstructMCTree() {
  // init state map
  std::unordered_map<std::string, VehicleState> init_state;

  // init state: ego
  auto init_ego_traj = ego_traj_.Evaluate(0.0);
  VehicleState ego_state(init_ego_traj.path_point().x(),
                         init_ego_traj.path_point().y(),
                         init_ego_traj.path_point().theta(), init_ego_traj.v(),
                         init_ego_traj.a(), init_ego_traj.path_point().kappa());
  init_state["ego"] = ego_state;

  // init state: obs
  for (auto &[id, obstacle] : pred_obs_) {
    if (obstacle.trajectory().size() == 0 ||
        obstacle.trajectory()[0].trajectory_point().size() == 0) {
      return false;
    }
    TrajectoryPoint init_obs_traj =
        obstacle.trajectory()[0].trajectory_point()[0];
    VehicleState obs_state(
        init_obs_traj.path_point().x(), init_obs_traj.path_point().y(),
        init_obs_traj.path_point().theta(), init_obs_traj.v(),
        init_obs_traj.a(), init_obs_traj.path_point().kappa());
    init_state[id] = obs_state;
  }

  // construct MCTSNode
  MCTSNode *root_node = new MCTSNode(0, init_state);
  if (root_node == nullptr) {
    std::cout << "Failed to construct MCTSNode";
    return false;
  }
  // Constrcut MCTSTree
  mcts_tree_ = std::make_shared<MCTSTree>(root_node, mcts_func_);
  if (mcts_tree_ == nullptr) {
    std::cout << "Failed to construct MCTSTree";
    return false;
  }

  // Preconstruct MCTS tree
  mcts_func_->PreConstructTree(root_node, mcts_param_.xica_need_preconstruct);
  return true;
}

bool Planner::SetGamingAgentInfo() {
  opposite_collision_obs_set_.clear();
  std::vector<std::string> opposite_collision_obs = {"1111", "2222"};
  for (auto obs_id : opposite_collision_obs) {
    opposite_collision_obs_set_.insert(obs_id);
  }

  return true;
}

bool Planner::BuildGamingInfo() {
  if (is_first_entry_) {
    is_first_entry_ = false;
  }
  mcts_param_.last_modified_trajectories.clear();
  // xiepanpan: temp set constant value
  mcts_param_.ego_target_speed = std::min(mcts_param_.veh_param.max_vel, 10.0);
  return true;
}

bool Planner::LoadParams() {
  // 1. 设置 MCTS 动力学参数
  // 注意：这里仅赋值了代码中用到的参数，其它（如 sampler 部分）可按需要补充
  MCTSDynamics mcts_dynamics;
  mcts_dynamics.max_ddkappa_ = 100.0;
  mcts_dynamics.max_jerk_ = 4.0;
  mcts_dynamics.max_kappa_ = 0.2;
  mcts_dynamics.max_lat_acc_ = 4.0 * 2;
  mcts_dynamics.max_v_ = 30.0;

  // 2. 车辆参数
  VehConfig veh_param;
  veh_param.length = 5.0;                             // veh_ego_length: 5.0
  veh_param.width = 2.0;                              // veh_ego_width: 2.0
  veh_param.max_vel = mcts_dynamics.max_v_;           // 30.0
  veh_param.max_lat_acc = mcts_dynamics.max_lat_acc_; // 3.0
  veh_param.max_ddkappa = mcts_dynamics.max_ddkappa_; // 100.0
  veh_param.max_kappa = mcts_dynamics.max_kappa_;     // 0.2
  veh_param.max_acc = 6.0;                            // veh_max_acc: 6.0
  veh_param.min_acc = -5.0;                           // veh_min_acc: -5.0
  veh_param.max_delta_l = 3;                          // max_delta_l: 3

  // 3. MCTS 参数
  XICAMCTSParam mcts_param;
  mcts_param.max_search_iter = 1000; // mcts_max_search_iter: 1000
  mcts_param.max_search_time = 3.0;  // mcts_max_search_time: 3.0
  mcts_param.pool_size = 1000;       // mcts_node_pool_size: 1000
  mcts_param.veh_param = veh_param;
  mcts_param.gamma = 0.8;                  // gamma: 0.8
  mcts_param.c = 1.41;                     // c: 1.41
  mcts_param.left_turn_range = 10.0;       // left_turn_range: 10.0
  mcts_param.heading_range = 0.17;         // heading_range: 0.17
  mcts_param.min_valid_node_num = 30;      // min_valid_node_num: 30
  mcts_param.lk_lat_range = 6.0;           // lk_lat_range: 6.0
  mcts_param.lat_expand_factor = 1.0;      // lat_expand_factor: 1.0
  mcts_param.long_expand_factor = 1.0;     // long_expand_factor: 1.0
  mcts_param.min_decision_distance = 10.0; // min_decision_distance: 10.0
  mcts_param.min_decision_delta_t = 0.5;   // min_decision_delta_t: 0.5
  mcts_param.xica_diff_dis_max = 5.0;      // xica_diff_dis_max: 5.0
  mcts_param.xica_diff_v_max = 3.0;        // xica_diff_v_max: 3.0
  mcts_param.xica_diff_a_max = 2.0;        // xica_diff_a_max: 2.0

  mcts_param.ego_agent_reward_adjust = 1.0; // ego_agent_reward_adjust: 1.0

  mcts_param.use_ref_pre_construct = false; // use_ref_pre_construct: false
  mcts_param.xica_need_preconstruct = false; // xica_need_preconstruct: true
  mcts_param.xica_need_ego_idm = false;      // xica_need_ego_idm: true

  mcts_param.occ_bound_max = -1.0; // occ_bound_max: -1.0

  // 4. Reward 信息参数（两处均赋相同的值）
  mcts_param.reward_info.w_acc = 0.0;            // w_acc: 0.0
  mcts_param.reward_info.w_ref = 0.3;            // w_ref: 0.3
  mcts_param.reward_info.w_eff = 0.3;            // w_eff: 0.3
  mcts_param.reward_info.w_safe = 1.0;           // w_safe: 1.0
  mcts_param.reward_info.w_pred = 0.4;           // w_pred: 0.8
  mcts_param.reward_info.w_cons_act = 0.2;       // w_cons_act: 0.2
  mcts_param.reward_info.w_cons_his = 0.2;       // w_cons_his: 0.2
  mcts_param.reward_info.invalid_penalty = -0.5; // invalid_penalty: -0.5

  mcts_param.xica_reward_info.w_acc = 0.0; // 同上
  mcts_param.xica_reward_info.w_ref = 0.3;
  mcts_param.xica_reward_info.w_eff = 0.3;
  mcts_param.xica_reward_info.w_safe = 1.0;
  mcts_param.xica_reward_info.w_pred = 0.8;
  mcts_param.xica_reward_info.w_cons_act = 0.2;
  mcts_param.xica_reward_info.w_cons_his = 0.2;
  mcts_param.xica_reward_info.invalid_penalty = -0.5;

  mcts_param.xica_reward_info.xica_w_eff = 0.6;      // xica_w_eff: 0.6
  mcts_param.xica_reward_info.xica_w_acc = 0.5;      // xica_w_acc: 0.5
  mcts_param.xica_reward_info.xica_w_safe = 1.0;     // xica_w_safe: 1.0
  mcts_param.xica_reward_info.xica_w_occ = 0.4;      // xica_w_occ: 0.4
  mcts_param.xica_reward_info.xica_w_cons_his = 0.1; // xica_w_cons_his: 0.1

  // 5. IDM 参数
  mcts_param.xica_idm_param.max_acc_ = 1.3;          // max_acc_: 1.3
  mcts_param.xica_idm_param.comfort_acc_ = 1.0;      // comfort_acc_: 1.0
  mcts_param.xica_idm_param.acc_exp_ = 2.0;          // acc_exp_: 2.0
  mcts_param.xica_idm_param.idm_min_dist_ = 2.5;     // idm_min_dist_: 2.5
  mcts_param.xica_idm_param.idm_desired_time_ = 0.9; // idm_desired_time_: 0.9
  mcts_param.xica_idm_param.idmepsilon = 1e-5;       // idmepsilon: 1e-5

  // 6. 其它参数
  mcts_param.xica_reward_info.xica_w_refline = 2.0; // xica_w_refline: 1.0

  // 将配置保存到成员变量中
  mcts_param_ = mcts_param;
  return true;
}

bool Planner::ConstructTestInput(const TestInputParams& params) {
  // ----- 构造自车（ego）的规划轨迹点 -----
  for (int i = 0; i < params.total_points; ++i) {
    PathPoint pp;
    const double t = i * params.delta_time;  // 当前时间

    // 计算自车轨迹点：x轴匀速直线运动
    pp.set_x(params.ego_initial_x + params.ego_speed * t);
    pp.set_y(params.ego_initial_y);  // y 坐标保持不变
    pp.set_theta(params.ego_initial_theta);
    pp.set_s(params.ego_initial_s + params.ego_speed * t);
    pp.set_kappa(params.ego_initial_kappa);

    TrajectoryPoint tp(pp, params.ego_speed, params.ego_acceleration, 0.0, t);
    mcts_param_.ego_traj_points.push_back(tp);
    ego_traj_.AddTrajectoryPoint(tp);
  }

  // ----- 构造障碍车（他车）的预测轨迹 -----
  PredictionObstacle obs;
  obs.AddTrajectory(Trajectory()); // 初始化第一条轨迹
  for (int i = 0; i < params.total_points; ++i) {
    PathPoint pp;
    const double t = i * params.delta_time;  // 当前时间

    // 计算障碍车轨迹点：x轴匀速运动（可正向或反向，根据传入的 obs_speed）
    pp.set_x(params.obs_initial_x + params.obs_speed * t);
    pp.set_y(params.obs_initial_y);  // y 坐标保持不变
    pp.set_theta(params.obs_initial_theta);
    pp.set_s(params.obs_initial_s + params.obs_speed * t);
    pp.set_kappa(params.obs_initial_kappa);

    TrajectoryPoint tp(pp, params.obs_speed, params.obs_acceleration, 0.0, t);
    obs.trajectories[0].mutable_trajectoryPoint().push_back(tp);
  }
  pred_obs_["01"] = obs;

  // ----- 其他参数（例如车辆动作）保持不变 -----
  std::unordered_map<std::string, VehicleAction> action_sample;
  action_sample["ego"] = VehicleAction(0.0, 0.0);
  action_sample["01"] = VehicleAction(0.0, 0.0);
  
  return true;
}

} // namespace BehaviorPlanner
} // namespace apollo
