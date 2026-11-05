// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class ScaleBFM final : public Observation {
  Selection select_;
  std::vector<size_t> bodies_;
  std::vector<int> future_;
  int length_, mode_;
  bool forcing_;
  std::string output_;
  History history_;
  Quat alignment_ = Quat::Identity();
  V3 origin_ = V3::Zero();
  Vec frame(const ObservationContext &c, bool zero = false) const {
    Vec v;
    quat(v, c.state.orientation.normalized());
    append(v, c.state.angular_velocity);
    add(v, select_.q(c));
    add(v, select_.q(c, true));
    add(v, zero ? Vec(c.previous_action.size()) : c.previous_action);
    return v;
  }

public:
  ScaleBFM(const YAML::Node &n, const ObservationContext &c, std::string output)
      : select_(n, c), future_(n["future_idx"].as<std::vector<int>>()),
        length_(bounded(get<int>(n, "history_buffer_size", 3), "ScaleBFM history")),
        mode_(get<int>(n, "control_mode", 7)), forcing_(get<bool>(n, "reference_forcing", true)),
        output_(std::move(output)) {
    for (auto &name : n["selected_body_names"].as<Names>())
      bodies_.push_back(index_of(tracking_motion(c).bodies, name));
    if (bodies_.empty() || future_.empty())
      throw std::runtime_error("empty ScaleBFM reference selection");
    if (n["action_names"].as<Names>() != c.policy.actions)
      throw std::runtime_error("ScaleBFM action layout mismatch");
  }
  void reset(const ObservationContext &c) override {
    history_.reset(length_, frame(c, true));
    auto &f = select_.at(c, future_.front());
    alignment_ = projected_yaw(c.state.orientation.normalized()) *
                 projected_yaw(f.body_orientation.at(bodies_[0]).normalized()).conjugate();
    origin_ = f.body_position.at(bodies_[0]);
    origin_.z() = 0;
  }
  void update(const ObservationContext &c) override { history_.push(frame(c)); }
  Tensor compute(const ObservationContext &c) const override {
    if (output_ == "control_mode") {
      Tensor t;
      t.shape = {1};
      t.data = std::vector<int64_t>{mode_};
      return t;
    }
    if (output_ == "future_time_offsets") {
      Tensor t;
      t.shape = {1, int64_t(future_.size()), 1};
      t.data = std::vector<int64_t>(future_.begin(), future_.end());
      return t;
    }
    if (output_.rfind("target_body_", 0) == 0) {
      bool orientation = output_.find("_rot_") != std::string::npos;
      V3 root =
          forcing_
              ? V3(alignment_ * (select_.at(c, future_[0]).body_position.at(bodies_[0]) - origin_))
              : c.state.root_position;
      if (!forcing_ && !c.state.root_position_valid)
        throw std::runtime_error(
            "ScaleBFM reference_forcing=false requires root position feedback");
      Vec v;
      auto inv = c.state.orientation.normalized().conjugate();
      for (int step : future_) {
        auto &f = select_.at(c, step);
        for (auto id : bodies_)
          if (orientation)
            quat(v, (inv * alignment_ * f.body_orientation.at(id).normalized()).normalized());
          else
            append(v, inv * (alignment_ * (f.body_position.at(id) - origin_) - root));
      }
      return tensor(v, {1, int64_t(future_.size()), int64_t(bodies_.size()), orientation ? 4 : 3});
    }
    size_t start = 0, width = 0;
    size_t n = select_.names.size(), a = c.previous_action.size();
    if (output_ == "root_quat_buffer")
      width = 4;
    else if (output_ == "base_ang_vel_buffer") {
      start = 4;
      width = 3;
    } else if (output_ == "dof_pos_buffer") {
      start = 7;
      width = n;
    } else if (output_ == "dof_vel_buffer") {
      start = 7 + n;
      width = n;
    } else if (output_ == "actions_buffer") {
      start = 7 + 2 * n;
      width = a;
    } else
      throw std::runtime_error("unknown ScaleBFM output");
    Vec v;
    for (auto i = history_.frames.rbegin(); i != history_.frames.rend(); ++i)
      v.insert(v.end(), i->begin() + start, i->begin() + start + width);
    return tensor(v, {1, length_, int64_t(width)});
  }
};
} // namespace
void register_scalebfm_observations(ObservationRegistry &r) {
  for (std::string name :
       {"root_quat_buffer", "base_ang_vel_buffer", "dof_pos_buffer", "dof_vel_buffer",
        "actions_buffer", "target_body_pos_future_to_robot_base",
        "target_body_rot_future_to_robot_base", "control_mode", "future_time_offsets"})
    r.add("scalebfm.scalebfm_" + name,
          [name](auto &n, auto &c) { return std::make_unique<ScaleBFM>(n, c, name); });
}
} // namespace boltrt
