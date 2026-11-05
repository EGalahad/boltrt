// SPDX-License-Identifier: Apache-2.0
#include "boltrt/observation/observation.hpp"
#include <iostream>

class JointPosition final : public boltrt::Observation {
public:
  boltrt::Tensor compute(const boltrt::ObservationContext &context) const override {
    return boltrt::Tensor::floats(context.state.q);
  }
};

int main() {
  boltrt::ObservationRegistry registry;
  registry.add("custom.joint_position",
               [](const auto &, const auto &) { return std::make_unique<JointPosition>(); });
  boltrt::RobotSpec robot;
  boltrt::PolicyConfig policy;
  boltrt::RobotState state;
  state.q = {0.1f, -0.2f};
  boltrt::Vec previous_action;
  // No motion or tracking context is needed by this observation.
  boltrt::ObservationContext context{robot, policy, state, previous_action};
  boltrt::ObservationGraph graph(
      registry, YAML::Load("actor: {joint_position: {_target_: custom.joint_position}}"), context);
  graph.reset(context);
  const auto result = graph.compute(context);
  std::cout << "actor width: " << result.at("actor").size() << '\n';
}
