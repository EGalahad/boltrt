// SPDX-License-Identifier: Apache-2.0
#include "boltrt/observation/observation.hpp"
namespace boltrt {
namespace {
class StateObservation final : public Observation {
public:
  explicit StateObservation(std::string kind) : kind_(std::move(kind)) {}
  Tensor compute(const ObservationContext &c) const override {
    if (kind_ == "joint_pos")
      return Tensor::floats(c.state.q);
    if (kind_ == "joint_vel")
      return Tensor::floats(c.state.dq);
    if (kind_ == "prev_action")
      return Tensor::floats(c.previous_action);
    Vec result;
    if (kind_ == "root_ang_vel")
      append(result, c.state.angular_velocity);
    else
      append(result, c.state.orientation.conjugate() * V3(0, 0, -1));
    return Tensor::floats(std::move(result));
  }

private:
  std::string kind_;
};
} // namespace
void register_common_observations(ObservationRegistry &registry) {
  for (const std::string name :
       {"joint_pos", "joint_vel", "root_ang_vel", "projected_gravity", "prev_action"})
    registry.add(name, [name](const auto &node, const auto &) {
      require_keys(node, {"_target_"}, "common observation");
      return std::make_unique<StateObservation>(name);
    });
}
} // namespace boltrt
