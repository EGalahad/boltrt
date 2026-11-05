// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class HoloActor final : public Observation {
  Selection select_;
  int count_;
  mutable Quat alignment_ = Quat::Identity();
  mutable bool aligned_ = false;

public:
  HoloActor(const YAML::Node &n, const ObservationContext &c)
      : select_(n, c),
        count_(bounded(get<int>(n, "num_future_frames", 10), "HoloMotion future frames")) {}
  void reset(const ObservationContext &) override { aligned_ = false; }
  Tensor compute(const ObservationContext &c) const override {
    if (tracking_motion(c).frames.size() < size_t(count_ + 1))
      throw std::runtime_error("HoloMotion future horizon too short");
    auto &first = tracking_motion(c).frames[0];
    auto q = first.body_orientation.at(select_.root).normalized();
    auto robot = c.state.orientation.normalized();
    if (!aligned_) {
      alignment_ = yaw(robot) * yaw(q).conjugate();
      aligned_ = true;
    }
    auto aligned = alignment_ * q;
    auto ye = yaw(aligned) * yaw(robot).conjugate();
    double yawerror = 2 * std::atan2(ye.z(), ye.w());
    Vec v;
    append(v, gravity(q));
    append(v, q.conjugate() * linear(first, select_.root));
    append(v, q.conjugate() * angular(first, select_.root));
    add(v, select_.ref(first));
    v.push_back(float(first.body_position.at(select_.root).z()));
    add(v, {float(std::sin(yawerror)), float(std::cos(yawerror))});
    append(v, gravity(robot));
    append(v, c.state.angular_velocity);
    add(v, select_.q(c, false, true));
    add(v, select_.q(c, true));
    add(v, c.previous_action);
    for (int i = 1; i <= count_; ++i)
      add(v, select_.ref(tracking_motion(c).frames[i]));
    for (int i = 1; i <= count_; ++i)
      v.push_back(float(tracking_motion(c).frames[i].body_position.at(select_.root).z()));
    for (int i = 1; i <= count_; ++i)
      append(v, gravity(tracking_motion(c).frames[i].body_orientation.at(select_.root)));
    for (int i = 1; i <= count_; ++i) {
      auto &f = tracking_motion(c).frames[i];
      append(v, f.body_orientation.at(select_.root).normalized().conjugate() *
                    linear(f, select_.root));
    }
    for (int i = 1; i <= count_; ++i) {
      auto &f = tracking_motion(c).frames[i];
      append(v, f.body_orientation.at(select_.root).normalized().conjugate() *
                    angular(f, select_.root));
    }
    for (int i = 1; i <= count_; ++i) {
      auto d = yaw(tracking_motion(c).frames[i].body_orientation.at(select_.root).normalized()) *
               yaw(q).conjugate();
      double a = 2 * std::atan2(d.z(), d.w());
      add(v, {float(std::sin(a)), float(std::cos(a))});
    }
    for (int i = 1; i <= count_; ++i)
      columns(v, robot.conjugate() * alignment_ *
                     tracking_motion(c).frames[i].body_orientation.at(select_.root));
    return tensor(v, {1, 134 + 47 * count_});
  }
};
class HoloState final : public Observation {
  Tensor zero_;
  std::string key_;
  int64_t reset_at_ = 0;
  mutable int64_t step_ = 0;

public:
  HoloState(const YAML::Node &n, const ObservationContext &)
      : key_(get<std::string>(n, "state_key", "past_key_values")) {
    zero_.shape = n["shape"].as<std::vector<int64_t>>();
    size_t size = 1;
    for (auto d : zero_.shape) {
      if (d <= 0 || size > 100000000 / size_t(d))
        throw std::runtime_error("invalid HoloMotion KV shape");
      size *= size_t(d);
    }
    zero_.data = Vec(size);
    if (get<std::string>(n, "dtype", "float32") != "float32")
      throw std::runtime_error("HoloMotion KV cache must be float32");
    reset_at_ = get<int>(n, "rope_max_seq_len", 0);
    if (reset_at_ > 0) {
      auto t = reset_at_ - get<int>(n, "rope_reset_margin", 64);
      if (t > 0)
        reset_at_ = t;
    }
  }
  void reset(const ObservationContext &) override { step_ = 0; }
  Tensor compute(const ObservationContext &c) const override {
    bool reset = step_ == 0 || (reset_at_ > 0 && step_ % reset_at_ == 0);
    ++step_;
    if (!reset && c.previous_output) {
      auto it = c.previous_output->find("next/" + key_);
      if (it != c.previous_output->end()) {
        if (it->second.size() != zero_.size())
          throw std::runtime_error("HoloMotion KV cache size mismatch");
        auto t = it->second;
        t.shape = zero_.shape;
        return t;
      }
    }
    return zero_;
  }
};
class HoloStep final : public Observation {
  mutable int64_t step_ = 0;
  int64_t reset_at_ = 0;

public:
  HoloStep(const YAML::Node &n, const ObservationContext &) {
    if (get<std::string>(n, "dtype", "int64") != "int64")
      throw std::runtime_error("HoloMotion step_idx must be int64");
    reset_at_ = get<int>(n, "rope_max_seq_len", 0);
    if (reset_at_ > 0) {
      auto t = reset_at_ - get<int>(n, "rope_reset_margin", 64);
      if (t > 0)
        reset_at_ = t;
    }
  }
  void reset(const ObservationContext &) override { step_ = 0; }
  Tensor compute(const ObservationContext &) const override {
    if (reset_at_ > 0 && step_ >= reset_at_)
      step_ = 0;
    Tensor t;
    t.shape = {1};
    t.data = std::vector<int64_t>{step_++};
    return t;
  }
};
} // namespace
void register_holomotion_observations(ObservationRegistry &r) {
  r.add("holomotion.holomotion_motion_actor_obs",
        [](auto &n, auto &c) { return std::make_unique<HoloActor>(n, c); });
  r.add("holomotion.holomotion_zero_state",
        [](auto &n, auto &c) { return std::make_unique<HoloState>(n, c); });
  r.add("holomotion.holomotion_step_idx",
        [](auto &n, auto &c) { return std::make_unique<HoloStep>(n, c); });
}
} // namespace boltrt
