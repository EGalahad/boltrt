// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace boltrt {
using Vec = std::vector<float>;
using Names = std::vector<std::string>;
using Quat = Eigen::Quaterniond; // wxyz at serialization boundaries
using V3 = Eigen::Vector3d;
struct RobotState {
  Vec q, dq, torque;
  V3 root_position = V3::Zero(), root_velocity = V3::Zero(), angular_velocity = V3::Zero();
  Quat orientation = Quat::Identity();
  bool root_position_valid = false, root_velocity_valid = false;
  uint64_t tick = 0;
  double time = 0;
};
struct JointCommand {
  Vec q, dq, tau, kp, kd;
};
enum class DType { Float32, Int64, Bool };
struct Tensor {
  std::vector<int64_t> shape;
  std::variant<Vec, std::vector<int64_t>, std::vector<uint8_t>> data = Vec{};
  DType dtype() const { return static_cast<DType>(data.index()); }
  size_t size() const {
    return std::visit([](auto &v) { return v.size(); }, data);
  }
  static Tensor floats(Vec v) {
    Tensor t;
    t.shape = {1, static_cast<int64_t>(v.size())};
    t.data = std::move(v);
    return t;
  }
  void validate() const;
};
using TensorMap = std::map<std::string, Tensor>;
inline Quat yaw(const Quat &q) {
  const double a =
      std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  return Quat(Eigen::AngleAxisd(a, V3::UnitZ()));
}
inline V3 rotate(const Quat &q, const V3 &v) {
  return v * (2 * q.w() * q.w() - 1) + 2 * q.w() * q.vec().cross(v) + 2 * q.vec() * q.vec().dot(v);
}
inline Quat projected_yaw(const Quat &q) {
  auto x = rotate(q, V3::UnitX()), z = rotate(q, V3::UnitZ());
  auto heading =
      x.head<2>().norm() > 0.1 ? V3(x.x(), x.y(), 0) : V3(z.x(), z.y(), 0) * (x.z() < 0 ? 1 : -1);
  return Quat(Eigen::AngleAxisd(std::atan2(heading.y(), heading.x()), V3::UnitZ()));
}
inline void append(Vec &out, const V3 &v) {
  for (int i = 0; i < 3; ++i)
    out.push_back(static_cast<float>(v[i]));
}
inline void append_rotation6(Vec &out, const Quat &q) {
  auto m = q.normalized().toRotationMatrix();
  for (int r = 0; r < 2; ++r)
    for (int c = 0; c < 3; ++c)
      out.push_back(static_cast<float>(m(r, c)));
}
size_t index_of(const Names &, const std::string &);
void finite(const Vec &, const std::string &);
} // namespace boltrt
