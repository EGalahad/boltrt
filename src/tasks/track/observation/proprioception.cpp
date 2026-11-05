// SPDX-License-Identifier: Apache-2.0
#include "../../../observation/history.hpp"
#include "boltrt/tasks/track/observation.hpp"
namespace boltrt {
using namespace observation_detail;
namespace {
void check_common(const YAML::Node &n) {
  require_keys(n,
               {"_target_", "history_steps", "joint_names", "noise_std", "bias_noise_std", "steps"},
               "observation");
}
} // namespace
void register_tracking_proprioception(ObservationRegistry &reg) {
  const std::string ns = "mimic_lite.";
  reg.add(ns + "root_ang_vel_history", [](auto &n, auto &) {
    check_common(n);
    return std::make_unique<History>(n["history_steps"].template as<std::vector<int>>(),
                                     [](auto &c) {
                                       Vec v;
                                       append(v, c.state.angular_velocity);
                                       return v;
                                     });
  });
  reg.add(ns + "projected_gravity_history", [](auto &n, auto &) {
    check_common(n);
    return std::make_unique<History>(n["history_steps"].template as<std::vector<int>>(),
                                     [](auto &c) {
                                       Vec v;
                                       append(v, c.state.orientation.conjugate() * V3(0, 0, -1));
                                       return v;
                                     });
  });
  for (bool vel : {false, true})
    reg.add(ns + (vel ? "joint_vel_history" : "joint_pos_history"),
            [vel](const YAML::Node &n, const ObservationContext &ctx) {
              check_common(n);
              auto chosen = select_names(n["joint_names"], ctx.robot.joints, ctx.policy.joints);
              std::vector<size_t> ids;
              for (auto &j : chosen)
                ids.push_back(index_of(ctx.robot.joints, j));
              return std::make_unique<History>(n["history_steps"].as<std::vector<int>>(),
                                               [ids, vel](auto &c) {
                                                 Vec v;
                                                 for (auto i : ids)
                                                   v.push_back(vel ? c.state.dq[i] : c.state.q[i]);
                                                 return v;
                                               });
            });
  reg.add(ns + "prev_actions", [](auto &n, auto &) {
    check_common(n);
    return std::make_unique<PreviousActions>(n["steps"].template as<int>());
  });
  // hdmi is the historical namespace for exactly these classes.
  auto names = reg.names();
  for (auto &name : names)
    if (name.rfind(ns, 0) == 0) {
      reg.alias("hdmi." + name.substr(ns.size()), name);
    }
}
} // namespace boltrt
