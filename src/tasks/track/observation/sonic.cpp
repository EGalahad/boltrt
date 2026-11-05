// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class SonicHistory final : public Observation {
  Selection select_;
  std::string kind_;
  std::vector<int> steps_;
  History history_;
  bool initialized_ = false;
  Vec value(const ObservationContext &c) const {
    if (kind_ == "root_ang_vel_history") {
      Vec v;
      append(v, c.state.angular_velocity);
      return v;
    }
    if (kind_ == "projected_gravity_history") {
      Vec v;
      append(v, gravity(c.state.orientation));
      return v;
    }
    if (kind_ == "joint_pos_rel_history")
      return select_.q(c, false, true);
    if (kind_ == "joint_vel_history")
      return select_.q(c, true);
    return c.previous_action;
  }

public:
  SonicHistory(const YAML::Node &n, const ObservationContext &c, std::string kind)
      : select_(n, c, true), kind_(std::move(kind)),
        steps_(n["history_steps"].as<std::vector<int>>()) {
    if (steps_.empty() || *std::min_element(steps_.begin(), steps_.end()) < 0)
      throw std::runtime_error("invalid SONIC history");
    std::sort(steps_.begin(), steps_.end(), std::greater<int>());
  }
  void reset(const ObservationContext &) override { initialized_ = false; }
  void update(const ObservationContext &c) override {
    auto v = value(c);
    if (!initialized_) {
      history_.reset(steps_.front() + 1, v);
      initialized_ = true;
    } else
      history_.push(v);
  }
  Tensor compute(const ObservationContext &) const override {
    return Tensor::floats(history_.selected(steps_));
  }
};
class SonicMotion final : public Observation {
  Selection select_;
  std::vector<int> offsets_;
  int kind_;
  Quat alignment_ = Quat::Identity();

public:
  SonicMotion(const YAML::Node &n, const ObservationContext &c, int kind)
      : select_(n, c), offsets_(offsets(n, c)), kind_(kind) {
    auto names = select_names(n["joint_names"], tracking_motion(c).joints, c.policy.actions);
    select_.motion_ids.clear();
    for (auto &s : names)
      select_.motion_ids.push_back(index_of(tracking_motion(c).joints, s));
  }
  void reset(const ObservationContext &c) override {
    alignment_ = projected_yaw(c.state.orientation) *
                 projected_yaw(select_.at(c, 0).body_orientation.at(select_.root)).conjugate();
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    if (kind_ == 0) {
      for (int step : offsets_)
        add(v, select_.ref(select_.at(c, step)));
      for (int step : offsets_)
        add(v, select_.ref(select_.at(c, step), true));
    } else
      for (int step : offsets_) {
        auto q = select_.at(c, step).body_orientation.at(select_.root);
        columns(v, kind_ == 2 ? projected_yaw(c.state.orientation).conjugate() * q
                              : c.state.orientation.conjugate() * alignment_ * q);
      }
    return Tensor::floats(std::move(v));
  }
};
class SonicSmpl final : public Observation {
  std::vector<int> offsets_;
  std::vector<size_t> wrists_;
  int kind_;
  Quat alignment_ = Quat::Identity();
  const MotionFrame &at(const ObservationContext &c, int offset) const {
    auto i =
        std::find(tracking_motion(c).offsets.begin(), tracking_motion(c).offsets.end(), offset);
    if (i == tracking_motion(c).offsets.end())
      throw std::runtime_error("SMPL future offset missing");
    auto &f = tracking_motion(c).frames.at(size_t(i - tracking_motion(c).offsets.begin()));
    if (!f.smpl_valid || f.smpl_joints.size() != 24)
      throw std::runtime_error(
          "SONIC SMPL policy requires an SMPL motion source; robot qpos cannot "
          "be substituted for human joints");
    return f;
  }

public:
  SonicSmpl(const YAML::Node &n, const ObservationContext &c, int kind)
      : offsets_(offsets(n, c)), kind_(kind) {
    for (auto name : {"left_wrist_roll_joint", "right_wrist_roll_joint", "left_wrist_pitch_joint",
                      "right_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint"})
      wrists_.push_back(index_of(tracking_motion(c).joints, name));
  }
  void reset(const ObservationContext &c) override {
    alignment_ = projected_yaw(c.state.orientation) *
                 projected_yaw(at(c, offsets_.front()).smpl_orientation).conjugate();
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    for (int step : offsets_) {
      auto &f = at(c, step);
      if (kind_ == 0)
        for (auto &j : f.smpl_joints)
          append(v, j);
      else if (kind_ == 1)
        columns(v, c.state.orientation.conjugate() * alignment_ * f.smpl_orientation);
      else
        for (auto id : wrists_)
          v.push_back(f.q.at(id));
    }
    return Tensor::floats(std::move(v));
  }
};
} // namespace
void register_sonic_observations(ObservationRegistry &r) {
  for (std::string kind : {"root_ang_vel_history", "projected_gravity_history",
                           "joint_pos_rel_history", "joint_vel_history", "prev_actions_history"})
    r.add("sonic.sonic_" + kind,
          [kind](auto &n, auto &c) { return std::make_unique<SonicHistory>(n, c, kind); });
  for (auto [name, kind] :
       std::vector<std::pair<std::string, int>>{{"command_multi_future_nonflat", 0},
                                                {"motion_anchor_ori_b_mf_nonflat", 1},
                                                {"motion_anchor_ori_heading_mf_nonflat", 2}})
    r.add("sonic.sonic_" + name,
          [kind](auto &n, auto &c) { return std::make_unique<SonicMotion>(n, c, kind); });
  for (auto [name, kind] :
       std::vector<std::pair<std::string, int>>{{"smpl_joints_multi_future_local", 0},
                                                {"smpl_root_ori_b_multi_future", 1},
                                                {"joint_pos_multi_future_wrist_for_smpl", 2}})
    r.add("sonic.sonic_" + name,
          [kind](auto &n, auto &c) { return std::make_unique<SonicSmpl>(n, c, kind); });
}
} // namespace boltrt
