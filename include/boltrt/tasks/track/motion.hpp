// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/config/config.hpp"
#include <algorithm>
#include <memory>
namespace boltrt {
inline std::vector<int> reference_offsets(const PolicyConfig &policy) {
  const auto config = policy.yaml["motion"];
  std::vector<int> result{0};
  if (config && config["future_steps"])
    result = config["future_steps"].as<std::vector<int>>();
  if (result.empty())
    result = {0};
  if (std::find(result.begin(), result.end(), 0) == result.end())
    throw std::runtime_error("motion.future_steps must include 0");
  return result;
}
struct MotionFrame {
  Vec q, dq;
  std::vector<V3> body_position;
  std::vector<Quat> body_orientation;
  std::vector<V3> body_linear_velocity, body_angular_velocity;
  std::vector<V3> smpl_joints;
  Quat smpl_orientation = Quat::Identity();
  bool smpl_valid = false;
};
struct MotionWindow {
  Names joints, bodies;
  std::vector<MotionFrame> frames;
  std::vector<int> offsets;
  int64_t frame = 0;
  int64_t last_frame = -1;
  double dt = 0.02;
  bool ended = false;
};
class MotionSource {
public:
  virtual ~MotionSource() = default;
  virtual void reset() = 0;
  virtual MotionWindow sample(bool advance, bool paused) = 0;
  virtual const Names &joint_names() const = 0;
  virtual const Names &body_names() const = 0;
};
std::unique_ptr<MotionSource> pose_motion(const RobotSpec &, const Vec &, const std::vector<int> &,
                                          double);
std::unique_ptr<MotionSource> file_motion(const std::filesystem::path &, const RobotSpec &,
                                          const std::vector<int> &, double, bool fp16 = true);
std::unique_ptr<MotionSource> zmq_motion(const std::string &, const RobotSpec &,
                                         const std::vector<int> &, double);
std::unique_ptr<MotionSource> smpl_motion(const std::string &, const RobotSpec &,
                                          const std::vector<int> &, double);
} // namespace boltrt
