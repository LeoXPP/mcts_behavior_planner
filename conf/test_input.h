// 定义测试输入参数结构体
struct TestInputParams {
  // 时间参数
  double delta_time = 0.1; // 时间间隔（秒）
  int total_points = 60;   // 采样点总数

  // 自车（ego）的参数
  double ego_initial_x = 0.0;     // 初始x坐标
  double ego_initial_y = 0.0;     // 初始y坐标
  double ego_speed = 10.0;        // 自车速度（m/s）
  double ego_acceleration = 0.0;  // 自车加速度（m/s²）
  double ego_initial_theta = 0.0; // 初始航向角（弧度）
  double ego_initial_s = 0.0;     // 累计路径距离起始值
  double ego_initial_kappa = 0.0; // 曲率

  // 障碍车（他车）的参数
  double obs_initial_x = 50.0; // 初始x坐标
  double obs_initial_y = 4.0;  // 初始y坐标
  double obs_speed = -7.0; // 障碍车速度（m/s），负值表示沿x轴反向运动
  double obs_acceleration = 0.0;  // 障碍车加速度（m/s²）
  double obs_initial_theta = 0.0; // 初始航向角（弧度）
  double obs_initial_s = 50.0;    // 累计路径距离起始值
  double obs_initial_kappa = 0.0; // 曲率
};
