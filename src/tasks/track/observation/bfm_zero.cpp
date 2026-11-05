// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
Vec scaled_action(const Vec &v) {
  Vec out = v;
  for (auto &x : out)
    x = std::clamp(5 * x, -5.f, 5.f);
  return out;
}
class BfmActor final : public Observation {
  Selection select_;
  int kind_, length_;
  std::vector<History> history_;
  Vec previous_history_;
  std::vector<Vec> values(const ObservationContext &c) const {
    Vec av, g;
    append(av, .25 * c.state.angular_velocity);
    append(g, gravity(c.state.orientation));
    return {scaled_action(c.previous_action), av, select_.q(c, false, true), select_.q(c, true), g};
  }

public:
  BfmActor(const YAML::Node &n, const ObservationContext &c, int kind)
      : select_(n, c), kind_(kind),
        length_(bounded(get<int>(n, "history_length", 4), "BFM-Zero history")) {}
  void reset(const ObservationContext &c) override {
    history_.clear();
    for (auto &v : values(c)) {
      History h;
      h.reset(length_, Vec(v.size()));
      history_.push_back(h);
    }
    previous_history_ = Vec(size_t(length_) * 93);
  }
  void update(const ObservationContext &c) override {
    if (kind_ == 2) {
      previous_history_.clear();
      for (auto &h : history_)
        add(previous_history_, h.all());
      auto v = values(c);
      for (size_t i = 0; i < v.size(); ++i)
        history_[i].push(v[i]);
    }
  }
  Tensor compute(const ObservationContext &c) const override {
    if (kind_ == 1)
      return Tensor::floats(scaled_action(c.previous_action));
    if (kind_ == 2)
      return Tensor::floats(previous_history_);
    Vec v = select_.q(c, false, true);
    add(v, select_.q(c, true));
    append(v, gravity(c.state.orientation));
    append(v, .25 * c.state.angular_velocity);
    return Tensor::floats(v);
  }
};
class BfmWindow final : public Observation {
  Selection select_;
  Names names_;
  std::vector<size_t> bodies_;
  size_t torso_;
  int length_, offset_, kind_;
  double fps_;
  bool clamp_;
  struct Poses {
    std::vector<V3> pos;
    std::vector<Quat> ori;
  };
  Poses pose(const MotionFrame &f) const {
    Poses p;
    for (auto id : bodies_) {
      p.pos.push_back(f.body_position.at(id));
      p.ori.push_back(f.body_orientation.at(id));
    }
    if (p.pos.size() == 30) {
      p.pos.push_back(f.body_position.at(torso_) +
                      f.body_orientation.at(torso_).normalized() * V3(0, 0, .35));
      p.ori.push_back(f.body_orientation.at(torso_));
    }
    return p;
  }
  static V3 local_delta(const Quat &a, const Quat &b, double fps) {
    Quat d = (a.normalized().conjugate() * b.normalized()).normalized();
    if (d.w() < 0)
      d.coeffs() *= -1;
    Eigen::AngleAxisd aa(d);
    return aa.axis() * aa.angle() * fps;
  }

public:
  BfmWindow(const YAML::Node &n, const ObservationContext &c, int kind)
      : select_(n, c), torso_(index_of(tracking_motion(c).bodies, "torso_link")),
        length_(bounded(get<int>(n, "seq_length", 8), "BFM window")),
        offset_(get<int>(n, "motion_t_offset", -1)), kind_(kind),
        fps_(get<double>(n, "target_fps", 50)), clamp_(get<bool>(n, "clamp_to_final", true)) {
    names_ = {"pelvis",
              "left_hip_pitch_link",
              "left_hip_roll_link",
              "left_hip_yaw_link",
              "left_knee_link",
              "left_ankle_pitch_link",
              "left_ankle_roll_link",
              "right_hip_pitch_link",
              "right_hip_roll_link",
              "right_hip_yaw_link",
              "right_knee_link",
              "right_ankle_pitch_link",
              "right_ankle_roll_link",
              "waist_yaw_link",
              "waist_roll_link",
              "torso_link",
              "left_shoulder_pitch_link",
              "left_shoulder_roll_link",
              "left_shoulder_yaw_link",
              "left_elbow_link",
              "left_wrist_roll_link",
              "left_wrist_pitch_link",
              "left_wrist_yaw_link",
              "right_shoulder_pitch_link",
              "right_shoulder_roll_link",
              "right_shoulder_yaw_link",
              "right_elbow_link",
              "right_wrist_roll_link",
              "right_wrist_pitch_link",
              "right_wrist_yaw_link"};
    for (auto &name : names_)
      bodies_.push_back(index_of(tracking_motion(c).bodies, name));
    if (std::find(tracking_motion(c).bodies.begin(), tracking_motion(c).bodies.end(),
                  "head_link") != tracking_motion(c).bodies.end())
      bodies_.push_back(index_of(tracking_motion(c).bodies, "head_link"));
    if (fps_ <= 0 || (!clamp_ && kind_ != 2))
      throw std::runtime_error("invalid BFM window settings");
  }
  Tensor compute(const ObservationContext &c) const override {
    Vec v;
    if (kind_ == 2) {
      int64_t prev = -1;
      for (int i = 0; i < length_; ++i) {
        int offset = offset_ + i;
        auto frame = std::max<int64_t>(0, tracking_motion(c).frame + offset);
        if (tracking_motion(c).last_frame >= 0)
          frame = std::min(frame, tracking_motion(c).last_frame);
        v.push_back(clamp_ && i > 0 && offset > 0 && frame == prev ? 0.f : 1.f);
        prev = frame;
      }
      return tensor(v, {length_, 1});
    }
    for (int i = 0; i < length_; ++i) {
      auto &f = select_.at(c, offset_ + i), &prev = select_.at(c, offset_ + i - 1);
      auto p = pose(f), old = pose(prev);
      if (kind_ == 0) {
        auto q = select_.ref(f), pq = select_.ref(prev);
        for (size_t j = 0; j < q.size(); ++j)
          v.push_back(q[j] - c.policy.default_q.at(select_.state_ids[j]));
        for (size_t j = 0; j < q.size(); ++j)
          v.push_back(float((q[j] - pq[j]) * fps_));
        append(v, gravity(p.ori[0]));
        append(v, local_delta(old.ori[0], p.ori[0], fps_));
      } else {
        auto inv = yaw(p.ori[0]).conjugate();
        v.push_back(float(p.pos[0].z()));
        for (size_t b = 1; b < p.pos.size(); ++b)
          append(v, inv * (p.pos[b] - p.pos[0]));
        for (auto &q : p.ori) {
          auto rot = inv * q;
          append(v, rotate(rot, V3::UnitX()));
          append(v, rotate(rot, V3::UnitZ()));
        }
        for (size_t b = 0; b < p.pos.size(); ++b)
          append(v, inv * ((p.pos[b] - old.pos[b]) * fps_));
        for (size_t b = 0; b < p.pos.size(); ++b)
          append(v, inv * local_delta(old.ori[b], p.ori[b], fps_));
      }
    }
    return tensor(v, {length_, kind_ == 0 ? 64 : 463});
  }
};
} // namespace
void register_bfm_zero_observations(ObservationRegistry &r) {
  for (auto [name, kind] : std::vector<std::pair<std::string, int>>{
           {"state", 0}, {"last_action", 1}, {"history_actor", 2}})
    r.add("bfm_zero.bfm_zero_" + name,
          [kind](auto &n, auto &c) { return std::make_unique<BfmActor>(n, c, kind); });
  for (auto [name, kind] :
       std::vector<std::pair<std::string, int>>{{"minimal_encoder_state_future", 0},
                                                {"minimal_privileged_state_future", 1},
                                                {"minimal_future_window_weight", 2}})
    r.add("bfm_zero.bfm_zero_" + name,
          [kind](auto &n, auto &c) { return std::make_unique<BfmWindow>(n, c, kind); });
}
} // namespace boltrt
