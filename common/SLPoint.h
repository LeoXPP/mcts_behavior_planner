#include <cmath>
#include <iostream>
#include <string>

namespace apollo {
namespace common {

#include <cmath>
#include <string>

#include <string>
#include <cmath>

class SLPoint {
public:
  // 属性：s_ 和 l_
  double s_;
  double l_;

  // 默认构造函数
  SLPoint() : s_(0.0), l_(0.0) {}

  // 带参数的构造函数
  SLPoint(double s, double l) : s_(s), l_(l) {}

  // Getter 方法，获取 s_ 和 l_ 的值
  double s() const { return s_; }
  double l() const { return l_; }

  // 创建一个 SLPoint 实例
  static SLPoint create(double s, double l) { return SLPoint(s, l); }

  // 验证 SLPoint 是否有效
  static bool verify(const SLPoint &point) {
    // 简单验证：s_ 和 l_ 都是数值
    return !std::isnan(point.s_) && !std::isnan(point.l_);
  }

  // 将 SLPoint 转换为字符串
  std::string toString() const {
    return "SLPoint(s: " + std::to_string(s_) + ", l: " + std::to_string(l_) + ")";
  }

  // 从字符串解析 SLPoint（假设字符串格式是 "s,l"）
  static SLPoint fromString(const std::string &str) {
    size_t comma_pos = str.find(',');
    double s = std::stod(str.substr(0, comma_pos));
    double l = std::stod(str.substr(comma_pos + 1));
    return SLPoint(s, l);
  }

  // 转换为 JSON 格式（简单的示例）
  std::string toJSON() const {
    return "{\"s\": " + std::to_string(s_) + ", \"l\": " + std::to_string(l_) + "}";
  }

  // 获取默认类型 URL
  static std::string getTypeUrl(const std::string &typeUrlPrefix = "type.googleapis.com") {
    return typeUrlPrefix + "/apollo.common.SLPoint";
  }
};


} // namespace common
} // namespace apollo
