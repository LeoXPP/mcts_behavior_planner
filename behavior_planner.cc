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
  auto start_t = std::chrono::high_resolution_clock::now();

  // 1. build gaming info
  if (!BuildGamingInfo()) {
    return false;
  }

  // 2. set gaming agent info
  if (!SetGamingAgentInfo()) {
    return false;
  }

  // 3. update decision params
  if (!UpdateDecisionParams()) {
    return false;
  }

  // 4. construct MCTree
  if (!ConstructMCTree()) {
    return false;
  }

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

  // update decision params
  if (!UpdateDecisionParams()) {
    return false;
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
  mcts_dynamics.max_lat_acc_ = 3.0;
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
  mcts_param.xica_need_preconstruct = true; // xica_need_preconstruct: true
  mcts_param.xica_need_ego_idm = true;      // xica_need_ego_idm: true

  mcts_param.occ_bound_max = -1.0; // occ_bound_max: -1.0

  // 4. Reward 信息参数（两处均赋相同的值）
  mcts_param.reward_info.w_acc = 0.0;            // w_acc: 0.0
  mcts_param.reward_info.w_ref = 0.3;            // w_ref: 0.3
  mcts_param.reward_info.w_eff = 0.3;            // w_eff: 0.3
  mcts_param.reward_info.w_safe = 1.0;           // w_safe: 1.0
  mcts_param.reward_info.w_pred = 0.8;           // w_pred: 0.8
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
  mcts_param.xica_reward_info.xica_w_refline = 1.0; // xica_w_refline: 1.0

  // 将配置保存到成员变量中
  mcts_param_ = mcts_param;
  return true;
}

bool Planner::ConstructTestInput() {
  // 设置 ego 车辆的目标速度与车辆参数（仅作为示例）

  // ----- 构造主车（ego）的规划轨迹点 -----
  // 从 (0,0) 出发，速度 10 m/s，加速度 0，逐步左转（航向角 theta 和 y
  // 坐标逐渐增加）
  for (int i = 0; i < 5; ++i) {
    // 先创建并初始化 PathPoint 对象

    PathPoint pp;
    pp.set_x(10.0 * i * 0.1);
    pp.set_y(0.2);
    pp.set_theta(0.0);

    // 假设 s 与 x 同值，这里直接设置 s 与 x 相关
    pp.set_s(10.0 * i * 0.1);

    // 设置曲率 kappa，与 i 相关，例如：kappa = 0.01 * i
    pp.set_kappa(0.00);

    // 此时 pp 已经根据 i 的值进行了初始化，可以继续使用 pp 进行后续操作

    // 使用初始化好的 PathPoint 对象来构造 TrajectoryPoint 对象
    TrajectoryPoint tp(pp, 10.0, 0.0, 0.0, i); // 速度 10 m/s，加速度 0 m/s²
    mcts_param_.ego_traj_points.push_back(tp);
    ego_traj_.AddTrajectoryPoint(tp);
  }

  // ----- 构造障碍车（他车）的预测轨迹 ----- 
  // 他车从 (50,4) 出发，假设“向右”对应 x 坐标减少，每 0.1 秒以 -15 m/s 移动（即
  // x 每步减少 1.5 米）
  PredictionObstacle obs;
  for (int i = 0; i < 5; ++i) {
    // 先创建并初始化 PathPoint 对象
    PathPoint pp;
    pp.set_x(50.0 - 15.0 * i * 0.1); // 例如：50, 48.5, 47, 45.5, 44
    pp.set_y(4.0);                   // y 保持不变
    pp.set_theta(0.0);               // 假设朝向保持水平
    pp.set_s(50.0 - 15.0 * i * 0.1); // 简单假设：s 与 x 同值
    pp.set_kappa(0.0);               // 曲率为 0

    // 使用 pp 对象和速度、加速度来构造 TrajectoryPoint 对象
    TrajectoryPoint tp(pp, -15.0, 0.0, 0.0, i); // 负速 -15.0, 加速度 0.0
    if (obs.trajectories.empty()) {
      obs.AddTrajectory(Trajectory());
    }
    // 将构造好的轨迹点加入到轨迹中
    obs.trajectories[0].mutable_trajectoryPoint().push_back(tp);
  }
  pred_obs_["01"] = obs;

  // // ----- 设置各车辆的决策类型 -----
  // // 例如：ego 为 EgoSearch（或 EgoPlanning，根据具体测试需求），障碍车为
  // // ObsSearch（或 ObsPrediction）
  // mcts_param.decision_type["ego"] = DecisionType::EgoSearch;
  // mcts_param.decision_type["obs1"] = DecisionType::ObsSearch;

  // ----- 构造候选动作（动作样本）-----
  // 此处简单设置一个动作样本：对于 ego 和障碍车，动作均为零 jerk 与零 dkappa
  std::unordered_map<std::string, VehicleAction> action_sample;
  action_sample["ego"] = VehicleAction(0.0, 0.0);
  action_sample["obs1"] = VehicleAction(0.0, 0.0);
  // 假设候选动作有多个（这里简单复制两份相同动作）
  // mcts_param.ego_agent_action.push_back(action_sample);
  // mcts_param.ego_agent_action.push_back(action_sample);

  // ----- 设置障碍车的目标速度 -----
  // mcts_param.obs_target_speed["obs1"] = -15.0;
}

int main() {
  Planner planner;
  planner.Init();
  planner.MakeDecision();
  planner.ConstructTestInput();
  

  return 0;
}

} // namespace BehaviorPlanner
} // namespace apollo
