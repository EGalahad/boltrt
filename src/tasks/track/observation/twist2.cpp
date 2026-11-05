// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class Twist final : public Observation {
  Selection select_;
  YAML::Node config_;
  int current_, future_, length_;
  std::string component_;
  History history_;
  std::map<std::string, Vec> parts_;
  Vec mimic(const ObservationContext &c, int step) const {
    auto &f = select_.at(c, step);
    auto q = f.body_orientation.at(select_.root).normalized();
    auto lv = q.conjugate() * linear(f, select_.root);
    auto av = q.conjugate() * angular(f, select_.root);
    Vec v{float(lv.x()), float(lv.y()), float(f.body_position.at(select_.root).z())};
    add(v, roll_pitch(q));
    v.push_back(float(av.z()));
    add(v, select_.ref(f));
    return v;
  }
  Vec proprio(const ObservationContext &c, bool zero = false) const {
    Vec v;
    append(v, c.state.angular_velocity * get<float>(config_, "ang_vel_scale", .25f));
    add(v, roll_pitch(c.state.orientation));
    auto p = select_.q(c, false, true), dq = select_.q(c, true);
    for (auto &x : p)
      x *= get<float>(config_, "dof_pos_scale", 1);
    Names ankles = get<Names>(config_, "ankle_joint_names",
                              {"left_ankle_pitch_joint", "left_ankle_roll_joint",
                               "right_ankle_pitch_joint", "right_ankle_roll_joint"});
    for (size_t i = 0; i < dq.size(); ++i)
      dq[i] = std::find(ankles.begin(), ankles.end(), select_.names[i]) != ankles.end()
                  ? 0
                  : dq[i] * get<float>(config_, "dof_vel_scale", .05f);
    add(v, p);
    add(v, dq);
    add(v, zero ? Vec(c.previous_action.size()) : c.previous_action);
    return v;
  }
  void assemble(const ObservationContext &c, bool reset) {
    auto m = mimic(c, current_), p = proprio(c, reset);
    parts_ = {{"current_motion", m},
              {"proprioception", p},
              {"observation_history", history_.all(true)},
              {"future_motion", mimic(c, future_)}};
  }

public:
  Twist(const YAML::Node &n, const ObservationContext &c)
      : select_(n, c, true), config_(n), current_(get<int>(n, "current_step", 0)),
        future_(get<int>(n, "future_step", 0)),
        length_(bounded(get<int>(n, "history_len", 10), "TWIST history")),
        component_(get<std::string>(n, "component", "")) {}
  void reset(const ObservationContext &c) override {
    auto v = mimic(c, current_);
    add(v, proprio(c, true));
    history_.reset(length_, v);
    assemble(c, true);
  }
  void update(const ObservationContext &c) override {
    assemble(c, false);
    auto v = parts_.at("current_motion");
    add(v, parts_.at("proprioception"));
    history_.push(v);
  }
  Tensor compute(const ObservationContext &) const override {
    if (!component_.empty())
      return Tensor::floats(parts_.at(component_));
    Vec v;
    for (auto key : {"current_motion", "proprioception", "observation_history", "future_motion"})
      add(v, parts_.at(key));
    return Tensor::floats(v);
  }
};
} // namespace
void register_twist2_observations(ObservationRegistry &r) {
  for (auto name : {"twist2.twist2_component_input", "twist2.twist2_input"})
    r.add(name, [](auto &n, auto &c) { return std::make_unique<Twist>(n, c); });
}
} // namespace boltrt
