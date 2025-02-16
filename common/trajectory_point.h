#pragma once

#include <vector>

class PathPoint {
public:
  // 默认构造函数
  PathPoint()
      : x_(0.0), y_(0.0), s_(0.0), theta_(0.0), kappa_(0.0), dkappa_(0.0) {}

  // 带参数的构造函数
  PathPoint(double x, double y, double s, double theta, double kappa,
            double dkappa = 0.0)
      : x_(x), y_(y), s_(s), theta_(theta), kappa_(kappa), dkappa_(dkappa) {}

  // Getter 方法
  double x() const { return x_; }
  double y() const { return y_; }
  double s() const { return s_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double dkappa() const { return dkappa_; }

  // Setter 方法
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  void set_s(double s) { s_ = s; }
  void set_theta(double theta) { theta_ = theta; }
  void set_kappa(double kappa) { kappa_ = kappa; }
  void set_dkappa(double dkappa) { dkappa_ = dkappa; }

private:
  double x_;
  double y_;
  double s_;
  double theta_;
  double kappa_;
  double dkappa_; // New member for change in curvature
};

class TrajectoryPoint {
private:
  PathPoint path_point_; // 内部的路径点
  double v_;             // 速度
  double a_;             // 加速度
  double da_;            // Change in acceleration (新增成员)

public:
  // 默认构造函数
  TrajectoryPoint() : path_point_(), v_(0.0), a_(0.0), da_(0.0) {}

  // 带参数的构造函数
  TrajectoryPoint(const PathPoint &path_point, double v, double a,
                  double da = 0.0)
      : path_point_(path_point), v_(v), a_(a), da_(da) {}

  // 获取内部的 PathPoint 对象（只读）
  const PathPoint &path_point() const { return path_point_; }

  // 获取速度、加速度和加速度变化
  double v() const { return v_; }
  double a() const { return a_; }
  double da() const { return da_; }

  // 设置内部的 PathPoint 对象
  void set_pathPoint(const PathPoint &pathPoint) { path_point_ = pathPoint; }

  // 设置速度、加速度和加速度变化
  void set_v(double v) { v_ = v; }
  void set_a(double a) { a_ = a; }
  void set_da(double da) { da_ = da; } // 新增setter方法
};

// 表示单条轨迹，即一系列的轨迹点
class Trajectory {
public:
  Trajectory() = default;

  // 返回轨迹点序列（只读版本）
  const std::vector<TrajectoryPoint> &trajectoryPoint() const { return points; }

  // 如果需要修改轨迹点序列，可以通过此方法获得非 const 引用
  std::vector<TrajectoryPoint> &mutable_trajectoryPoint() { return points; }

  // 添加一个轨迹点到轨迹末尾
  void add_trajectoryPoint(const TrajectoryPoint &point) {
    points.push_back(point);
  }

  const std::vector<TrajectoryPoint> &trajectory_point() const {
    return points;
  }

  // 清空当前轨迹
  void clear() { points.clear(); }

public:
  std::vector<TrajectoryPoint> points;
};

// 可选：如果需要保存多条轨迹，可以使用该 Trajectories 类
// 此类支持空判断、大小查询，以及通过重载 operator() 来索引单条轨迹
class Trajectories {
public:
  bool empty() const { return trajectories.empty(); }
  std::size_t size() const { return trajectories.size(); }

  // 通过 operator() 获取某一条轨迹（范围检查使用 at()）
  const Trajectory &operator()(std::size_t index) const {
    return trajectories.at(index);
  }
  Trajectory &operator()(std::size_t index) { return trajectories.at(index); }

  // 添加一条轨迹
  void add_trajectory(const Trajectory &traj) { trajectories.push_back(traj); }

private:
  std::vector<Trajectory> trajectories;
};

struct PredictionObstacle {
  // 存储多个预测轨迹
  std::vector<Trajectory> trajectories;

  // 障碍物尺寸信息（例如长和宽）
  double length;
  double width;

  // 其他可选信息（例如预测置信度）
  double confidence;

  // 默认构造函数
  PredictionObstacle() : length(0.0), width(0.0), confidence(1.0) {}

  // 带参数的构造函数
  PredictionObstacle(double len, double wid, double conf = 1.0)
      : length(len), width(wid), confidence(conf) {}

  // 返回所有预测轨迹
  const std::vector<Trajectory> &trajectory() const { return trajectories; }

  // 添加一条预测轨迹
  void AddTrajectory(const Trajectory &traj) { trajectories.push_back(traj); }
};

enum class DecisionType {
  EgoSearch,
  ObsSearch,
  EgoPlanning,
  EgoIDM,
  ObsPrediction
};