// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class TeleopIT final : public Observation {
  Selection select_;
  NativeFk fk_;
  size_t anchor_;
  int length_;
  double hz_;
  bool align_, history_output_, initialized_ = false, aligned_ = false, have_previous_ = false;
  Quat align_q_ = Quat::Identity(), prev_anchor_q_ = Quat::Identity();
  V3 pivot_ = V3::Zero(), prev_anchor_p_ = V3::Zero();
  Vec prev_q_, defaults_, current_;
  History history_;

public:
  TeleopIT(const YAML::Node &n, const ObservationContext &c, bool history)
      : select_(n, c),
        fk_(resolve_policy_asset(n["xml_path"].as<std::string>(), c.policy), select_.names),
        anchor_(fk_.body(get<std::string>(n, "anchor_body_name", "torso_link"))),
        length_(bounded(get<int>(n, "history_length", 10), "TeleopIT history")),
        hz_(get<double>(n, "policy_hz", 50)), align_(get<bool>(n, "align_reference_yaw", true)),
        history_output_(history) {
    defaults_ = n["default_dof_pos"] ? n["default_dof_pos"].as<Vec>() : Vec{};
    if (defaults_.empty())
      for (auto id : select_.state_ids)
        defaults_.push_back(c.policy.default_q.at(id));
    if (defaults_.size() != select_.names.size() || hz_ <= 0)
      throw std::runtime_error("invalid TeleopIT layout/rate");
  }
  void reset(const ObservationContext &) override {
    initialized_ = aligned_ = have_previous_ = false;
    current_ = Vec(166);
    history_.reset(length_, current_);
  }
  void update(const ObservationContext &c) override {
    auto &f = select_.at(c, 0);
    V3 p = f.body_position.at(select_.root);
    Quat q = f.body_orientation.at(select_.root);
    auto joints = select_.ref(f);
    if (align_) {
      if (!aligned_) {
        align_q_ = yaw(c.state.orientation * q.conjugate());
        pivot_ = p;
        aligned_ = true;
      }
      p = align_q_ * (p - pivot_) + pivot_;
      q = align_q_ * q;
    }
    Vec dq(joints.size());
    if (have_previous_)
      for (size_t i = 0; i < dq.size(); ++i)
        dq[i] = float((joints[i] - prev_q_[i]) * hz_);
    fk_.pose(p, q, joints);
    auto anchor_q = fk_.orientation(int(anchor_));
    auto anchor_p = fk_.position(int(anchor_));
    V3 lv = V3::Zero(), av = V3::Zero();
    if (have_previous_) {
      lv = (anchor_p - prev_anchor_p_) * hz_;
      {
        // Preserve the original float32 acos threshold for nearly-zero
        // rotations.
        auto a = anchor_q.cast<float>(), b = prev_anchor_q_.conjugate().cast<float>();
        float w = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
        Eigen::Vector3f vector(a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
                               a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
                               a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w());
        if (w < 0) {
          w = -w;
          vector = -vector;
        }
        float half = float(std::acos(double(std::clamp(w, -1.f, 1.f)))), sine = std::sin(half);
        if (sine > 1e-6f) {
          float dt = float(1 / hz_);
          for (int axis = 0; axis < 3; ++axis) {
            float value = vector[axis] / sine;
            value *= 2.f;
            value *= half;
            value /= dt;
            av[axis] = value;
          }
        }
      }
    }
    auto robotq = select_.q(c);
    fk_.pose(V3::Zero(), c.state.orientation, robotq);
    auto robot_anchor = fk_.orientation(int(anchor_));
    Vec obs;
    add(obs, joints);
    add(obs, dq);
    columns(obs, robot_anchor.conjugate() * anchor_q);
    append(obs, c.state.angular_velocity);
    for (size_t i = 0; i < robotq.size(); ++i)
      obs.push_back(robotq[i] - defaults_[i]);
    add(obs, select_.q(c, true));
    add(obs, c.previous_action);
    append(obs, gravity(c.state.orientation));
    append(obs, robot_anchor.conjugate() * lv);
    append(obs, robot_anchor.conjugate() * av);
    append(obs, gravity(anchor_q));
    if (obs.size() != 166)
      throw std::runtime_error("TeleopIT observation must have 166 values");
    current_ = obs;
    if (!initialized_) {
      history_.reset(length_, obs);
      initialized_ = true;
    } else
      history_.push(obs);
    prev_q_ = joints;
    prev_anchor_q_ = anchor_q;
    prev_anchor_p_ = anchor_p;
    have_previous_ = true;
  }
  Tensor compute(const ObservationContext &) const override {
    if (history_output_)
      return tensor(history_.all(true), {1, length_, 166});
    return Tensor::floats(current_);
  }
};
} // namespace
void register_teleopit_observations(ObservationRegistry &r) {
  r.add("teleopit.teleopit_velcmd_obs",
        [](auto &n, auto &c) { return std::make_unique<TeleopIT>(n, c, false); });
  r.add("teleopit.teleopit_velcmd_obs_history",
        [](auto &n, auto &c) { return std::make_unique<TeleopIT>(n, c, true); });
}
} // namespace boltrt
