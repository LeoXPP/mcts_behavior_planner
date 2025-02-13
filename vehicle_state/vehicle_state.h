# pragma once

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <unordered_map>


namespace apollo{
namespace BehaviorPlanner {

struct RewardItem {
  double weight;
  double value;
};

struct VehicleReward {
  RewardItem eff_r;
  RewardItem acc_r;
  RewardItem safe_r;
  RewardItem pred_r;
  RewardItem hist_r;
  RewardItem act_r;
  RewardItem int_r;
  RewardItem total_r;
};

struct VehConfig {
  // Size parameters
  double length, width;
  // Motion parameters
  double max_vel, max_acc, min_acc, max_lat_acc, max_snap;
  double max_ddkappa, max_kappa, max_delta_l;
  double comfort_jerk = 5.0;
};

class VehicleAction {
 public:
  VehicleAction() = default;
  VehicleAction(double jerk, double dkappa) : jerk_(jerk), dkappa_(dkappa) {}
  VehicleAction(const VehicleAction& other) : jerk_(other.jerk_), dkappa_(other.dkappa_) {}
  ~VehicleAction() = default;

 public:
  void DebugString() const {}
  inline double jerk() const { return jerk_; }
  inline double dkappa() const { return dkappa_; }
  inline void set_jerk(double jerk) { jerk_ = jerk; }
  inline void set_dkappa(double dkappa) { dkappa_ = dkappa; }

  bool operator==(const VehicleAction& other) const { return jerk_ == other.jerk() && dkappa_ == other.dkappa(); };
  bool operator!=(const VehicleAction& other) const { return !(*this == other); }

 private:
  double jerk_ = 0.0;
  double dkappa_ = 0.0;
};

class VehicleState {
 public:
  VehicleState() = default;
  VehicleState(double x, double y, double theta, double vel, double acc, double kappa)
      : x_(x), y_(y), theta_(theta), vel_(vel), acc_(acc), kappa_(kappa) {}
  ~VehicleState() = default;

 public:
  void DebugString() const {
    // ATRACE << "VehicleState: x = " << x_ << ", y = " << y_ << ", theta = " << theta_ << ", vel = " << vel_
    //        << ", acc = " << acc_ << ", kappa = " << kappa_;
  };
  inline double x() const { return x_; }
  inline double y() const { return y_; }
  inline double s() const { return s_; }
  inline double l() const { return l_; }
  inline double theta() const { return theta_; }
  inline double vel() const { return vel_; }
  inline double acc() const { return acc_; }
  inline double kappa() const { return kappa_; }

  inline void set_x(double x) { x_ = x; }
  inline void set_y(double y) { y_ = y; }
  inline void set_s(double s) { s_ = s; }
  inline void set_l(double l) { l_ = l; }
  inline void set_theta(double theta) { theta_ = theta; }
  inline void set_vel(double vel) { vel_ = vel; }
  inline void set_acc(double acc) { acc_ = acc; }
  inline void set_kappa(double kappa) { kappa_ = kappa; }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double s_ = 0.0;
  double l_ = 0.0;
  double theta_ = 0.0;
  double vel_ = 0.0;
  double acc_ = 0.0;
  double kappa_ = 0.0;
};

}// namespace BehaviorPlanner
}// namespace apollo