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
  double relative_time_;

public:
  // 默认构造函数
  TrajectoryPoint() : path_point_(), v_(0.0), a_(0.0), da_(0.0) {}

  // 带参数的构造函数
  TrajectoryPoint(const PathPoint &path_point, double v, double a,
                  double da = 0.0)
      : path_point_(path_point), v_(v), a_(a), da_(da) {}

  // 用于插值的构造函数
  TrajectoryPoint(double relative_time, const PathPoint &path_point)
      : path_point_(path_point), v_(0.0), a_(0.0), da_(0.0),
        relative_time_(relative_time) {}

  TrajectoryPoint(const PathPoint &path_point, double v, double a,
                  double da = 0.0, double relative_time = 0.0)
      : path_point_(path_point), v_(v), a_(a), da_(da),
        relative_time_(relative_time) {}

  // 获取内部的 PathPoint 对象（只读）
  const PathPoint &path_point() const { return path_point_; }

  // 获取速度、加速度和加速度变化
  double v() const { return v_; }
  double a() const { return a_; }
  double da() const { return da_; }
  double relative_time() const { return relative_time_; }

  // 设置内部的 PathPoint 对象
  void set_pathPoint(const PathPoint &pathPoint) { path_point_ = pathPoint; }

  // 设置速度、加速度和加速度变化
  void set_v(double v) { v_ = v; }
  void set_a(double a) { a_ = a; }
  void set_da(double da) { da_ = da; } // 新增setter方法
  void set_relative_time(double relative_time) {
    relative_time_ = relative_time;
  }
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

class DiscretizedTrajectory {
public:
  // 定义迭代器类型，方便支持范围 for 循环
  using iterator = std::vector<TrajectoryPoint>::iterator;
  using const_iterator = std::vector<TrajectoryPoint>::const_iterator;

  // 默认构造函数
  DiscretizedTrajectory() = default;

  // 重载构造函数：根据传入的路径点集合自动初始化轨迹，
  // 为每个点自动设置 relative_time（0秒、1秒、2秒……）
  DiscretizedTrajectory(const std::vector<PathPoint> &path_points,
                        const std::vector<double> &velocities = {},
                        const std::vector<double> &accelerations = {},
                        const std::vector<double> &d_accelerations = {}) {
    for (std::size_t i = 0; i < path_points.size(); ++i) {
      double relative_time = static_cast<double>(i);
      // 如果速度、加速度、加速度变化数组有足够的数据，使用它们；否则使用默认值
      double v = (i < velocities.size()) ? velocities[i] : 0.0;
      double a = (i < accelerations.size()) ? accelerations[i] : 0.0;
      double da = (i < d_accelerations.size()) ? d_accelerations[i] : 0.0;

      trajectory_points_.push_back(
          TrajectoryPoint(path_points[i], v, a, da, relative_time));
    }
  }

  // 添加一个完整的轨迹点（用户已设置好 relative_time）
  void AddTrajectoryPoint(const TrajectoryPoint &point) {
    trajectory_points_.push_back(point);
  }

  // 添加一个路径点，自动设置 relative_time 为当前轨迹点数量（每秒一个点）
  void AddPathPoint(const PathPoint &point, double v = 0.0, double a = 0.0,
                    double da = 0.0) {
    double relative_time = static_cast<double>(trajectory_points_.size());
    trajectory_points_.push_back(
        TrajectoryPoint(point, v, a, da, relative_time));
  }

  // 返回轨迹点数量
  std::size_t NumOfPoints() const { return trajectory_points_.size(); }

  // 根据索引返回轨迹点（只读）
  const TrajectoryPoint &TrajectoryPointAt(std::size_t index) const {
    if (index >= trajectory_points_.size()) {
      throw std::out_of_range("TrajectoryPointAt: index out of range");
    }
    return trajectory_points_[index];
  }

  // 根据索引返回轨迹点（可写）
  TrajectoryPoint &TrajectoryPointAt(std::size_t index) {
    if (index >= trajectory_points_.size()) {
      throw std::out_of_range("TrajectoryPointAt: index out of range");
    }
    return trajectory_points_[index];
  }

  // 支持迭代器 begin() / end()
  iterator begin() { return trajectory_points_.begin(); }
  iterator end() { return trajectory_points_.end(); }
  const_iterator begin() const { return trajectory_points_.begin(); }
  const_iterator end() const { return trajectory_points_.end(); }
  const_iterator cbegin() const { return trajectory_points_.cbegin(); }
  const_iterator cend() const { return trajectory_points_.cend(); }

  // 清空轨迹数据
  void Clear() { trajectory_points_.clear(); }

  // Evaluate 方法：根据输入时间 cur_t 计算（或插值）对应的轨迹点
  TrajectoryPoint Evaluate(double cur_t) const {
    if (trajectory_points_.empty()) {
      throw std::runtime_error("Evaluate: trajectory is empty");
    }

    // 若 cur_t 小于等于第一个点的时间，则返回第一个点
    if (cur_t <= trajectory_points_.front().relative_time()) {
      return trajectory_points_.front();
    }

    // 若 cur_t 大于等于最后一个点的时间，则返回最后一个点
    if (cur_t >= trajectory_points_.back().relative_time()) {
      return trajectory_points_.back();
    }

    // 遍历轨迹点，找到 cur_t 所在的区间
    for (std::size_t i = 1; i < trajectory_points_.size(); ++i) {
      const auto &prev_point = trajectory_points_[i - 1];
      const auto &next_point = trajectory_points_[i];
      if (prev_point.relative_time() <= cur_t &&
          cur_t <= next_point.relative_time()) {
        double dt = next_point.relative_time() - prev_point.relative_time();
        double ratio = (cur_t - prev_point.relative_time()) / dt;

        // 对路径点的 x 和 y 进行线性插值
        double x = (1 - ratio) * prev_point.path_point().x() +
                   ratio * next_point.path_point().x();
        double y = (1 - ratio) * prev_point.path_point().y() +
                   ratio * next_point.path_point().y();
        double s = (1 - ratio) * prev_point.path_point().s() +
                   ratio * next_point.path_point().s();
        double theta = (1 - ratio) * prev_point.path_point().theta() +
                       ratio * next_point.path_point().theta();
        double kappa = (1 - ratio) * prev_point.path_point().kappa() +
                       ratio * next_point.path_point().kappa();
        double dkappa = (1 - ratio) * prev_point.path_point().dkappa() +
                        ratio * next_point.path_point().dkappa();

        PathPoint interpolated_point(x, y, s, theta, kappa, dkappa);

        // 插值速度、加速度、加速度变化
        double v = (1 - ratio) * prev_point.v() + ratio * next_point.v();
        double a = (1 - ratio) * prev_point.a() + ratio * next_point.a();
        double da = (1 - ratio) * prev_point.da() + ratio * next_point.da();

        // 返回插值后的轨迹点，保持参数对齐
        return TrajectoryPoint(interpolated_point, v, a, da, cur_t);
      }
    }
    // 理论上不可能执行到这里，但为保险起见返回最后一个点
    return trajectory_points_.back();
  }

private:
  std::vector<TrajectoryPoint> trajectory_points_;
};