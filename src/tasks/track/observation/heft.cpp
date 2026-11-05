// SPDX-License-Identifier: Apache-2.0
#include "common.hpp"
namespace boltrt {
namespace {
using namespace ext;
class Heft final : public Observation {
  Selection select_;
  std::vector<int> offsets_, angular_steps_, gravity_steps_, pos_steps_, vel_steps_;
  int action_steps_, boot_max_, boot_;
  bool compliance_;
  std::string component_;
  History angular_, gravity_, pos_, vel_, actions_;
  static void init(History &h, const std::vector<int> &steps, size_t width) {
    if (steps.empty() || *std::min_element(steps.begin(), steps.end()) < 0)
      throw std::runtime_error("invalid HEFT history");
    h.reset(*std::max_element(steps.begin(), steps.end()) + 1, Vec(width));
  }

public:
  Heft(const YAML::Node &n, const ObservationContext &c, bool compliance)
      : select_(n, c), offsets_(offsets(n, c)),
        angular_steps_(n["root_angvel_history_steps"].as<std::vector<int>>()),
        gravity_steps_(n["projected_gravity_history_steps"].as<std::vector<int>>()),
        pos_steps_(n["joint_pos_history_steps"].as<std::vector<int>>()),
        vel_steps_(n["joint_vel_history_steps"].as<std::vector<int>>()),
        action_steps_(get<int>(n, "prev_action_steps", 8)),
        boot_max_(bounded(get<int>(n, "boot_indicator_max", 25), "HEFT boot indicator")),
        boot_(boot_max_), compliance_(compliance),
        component_(get<std::string>(n, "component", "")) {
    if (offsets_.empty() || offsets_[0] != 0)
      throw std::runtime_error("HEFT future steps must begin with 0");
  }
  void reset(const ObservationContext &c) override {
    boot_ = boot_max_;
    init(angular_, angular_steps_, 3);
    init(gravity_, gravity_steps_, 3);
    init(pos_, pos_steps_, select_.names.size());
    init(vel_, vel_steps_, select_.names.size());
    actions_.reset(action_steps_, Vec(c.previous_action.size()));
  }
  void update(const ObservationContext &c) override {
    boot_ = std::max(boot_ - 1, 0);
    Vec a, g;
    append(a, c.state.angular_velocity);
    append(g, gravity(c.state.orientation));
    angular_.push(a);
    gravity_.push(g);
    pos_.push(select_.q(c));
    vel_.push(select_.q(c, true));
    actions_.push(c.previous_action);
  }
  Tensor compute(const ObservationContext &c) const override {
    auto &first = select_.at(c, offsets_[0]);
    auto rootq = first.body_orientation.at(select_.root).normalized();
    auto rootp = first.body_position.at(select_.root);
    TensorMap parts;
    parts["context"] = Tensor::floats({float(boot_) / boot_max_});
    Vec command, target;
    for (size_t i = 1; i < offsets_.size(); ++i)
      append(command, rootq.conjugate() *
                          (select_.at(c, offsets_[i]).body_position.at(select_.root) - rootp));
    for (int step : offsets_)
      transposed_columns(command, c.state.orientation.conjugate() *
                                      select_.at(c, step).body_orientation.at(select_.root));
    for (int step : offsets_)
      add(target, select_.ref(select_.at(c, step)));
    auto current = select_.q(c);
    for (int step : offsets_) {
      auto q = select_.ref(select_.at(c, step));
      for (size_t i = 0; i < q.size(); ++i)
        target.push_back(q[i] - current[i]);
    }
    for (int step : offsets_)
      target.push_back(float(select_.at(c, step).body_position.at(select_.root).z()));
    for (int step : offsets_)
      append(target, rotate(select_.at(c, step).body_orientation.at(select_.root).conjugate(),
                            V3(0, 0, -1)));
    Vec proprio;
    for (auto v : {angular_.selected(angular_steps_), gravity_.selected(gravity_steps_),
                   pos_.selected(pos_steps_), vel_.selected(vel_steps_), actions_.all()})
      add(proprio, v);
    parts["motion_command"] = Tensor::floats(command);
    parts["target_motion"] = Tensor::floats(target);
    parts["proprioception"] = Tensor::floats(proprio);
    if (compliance_)
      parts["compliance_context"] = Tensor::floats({0, 0, 0});
    if (!component_.empty())
      return parts.at(component_);
    Vec all;
    add(all, {float(boot_) / boot_max_});
    add(all, command);
    if (compliance_)
      add(all, {0, 0, 0});
    add(all, target);
    add(all, proprio);
    return Tensor::floats(all);
  }
};
} // namespace
void register_heft_observations(ObservationRegistry &r) {
  r.add("heft.heft_component_obs", [](auto &n, auto &c) {
    auto variant = get<std::string>(n, "policy_variant", "pmg");
    if (variant != "pmg" && variant != "compliance")
      throw std::runtime_error("unknown HEFT variant");
    return std::make_unique<Heft>(n, c, variant == "compliance");
  });
  r.add("heft.heft_policy_obs",
        [](auto &n, auto &c) { return std::make_unique<Heft>(n, c, false); });
  r.add("heft.heft_compliance_policy_obs",
        [](auto &n, auto &c) { return std::make_unique<Heft>(n, c, true); });
}
} // namespace boltrt
