// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/motion.hpp"
#include "boltrt/utils/mujoco/model.hpp"
namespace boltrt {
namespace {
class PoseMotion final : public MotionSource {
public:
  PoseMotion(const RobotSpec &robot, const Vec &q, std::vector<int> offsets, double rate) {
    RobotModel fk(robot);
    fk.set_pose(q, robot.initial_position, Quat::Identity());
    window_.joints = robot.joints;
    window_.bodies = robot.bodies;
    window_.offsets = std::move(offsets);
    window_.dt = 1 / rate;
    MotionFrame frame;
    frame.q = q;
    frame.dq = Vec(q.size());
    for (int body : fk.bodies) {
      frame.body_position.emplace_back(Eigen::Map<const V3>(fk.data->xpos + 3 * body));
      const auto *quat = fk.data->xquat + 4 * body;
      frame.body_orientation.emplace_back(quat[0], quat[1], quat[2], quat[3]);
      frame.body_linear_velocity.push_back(V3::Zero());
      frame.body_angular_velocity.push_back(V3::Zero());
    }
    window_.frames.assign(window_.offsets.size(), frame);
  }
  void reset() override {}
  MotionWindow sample(bool, bool) override { return window_; }
  const Names &joint_names() const override { return window_.joints; }
  const Names &body_names() const override { return window_.bodies; }

private:
  MotionWindow window_;
};
} // namespace
std::unique_ptr<MotionSource> pose_motion(const RobotSpec &robot, const Vec &q,
                                          const std::vector<int> &offsets, double rate) {
  return std::make_unique<PoseMotion>(robot, q, offsets, rate);
}
} // namespace boltrt
